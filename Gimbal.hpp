#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: gimbal_test
constructor_args:
    task_stack_depth: 4096
    param_pid_yaw_speed:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
    param_pid_yaw_angle:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
    param_pid_pitch_speed:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
    param_pid_pitch_angle:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
    motor_yaw: '@&motor_yaw'
    motor_pitch: '@&motor_pit'
    cmd: '@&cmd'
    gimbal_param:
      pitch_min: 0.0
      pitch_max: 0.0
      motor_is_reverse: false
      yaw_offset: 0.0
template_args: []
required_hardware:
  - cmd
  - motor_can1
  - motor_can2
  - bmi088
depends:
  - qdu-future/BMI088
  - qdu-future/RMMotor
  - qdu-future/CMD
  - xrobot-org/MadgwickAHRS
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "BMI088.hpp"
#include "CMD.hpp"
#include "Eigen/Core"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include "transform.hpp"

enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_LOB
};

class Gimbal : public LibXR::Application {
 public:
  typedef enum : uint8_t { RELAX, COMMON, LOB } GimbalMode;

  struct GimbalParam {
    /* pitch的上下限，单位和GetAngle()一样，弧度 */
    LibXR::CycleValue<float> pitch_min;
    LibXR::CycleValue<float> pitch_max;
    bool motor_is_reverse;
    float yaw_offset;
  };

  /**
   * @brief gimbal 类的构造函数
   *
   * @param hw LibXR::HardwareContainer 的引用，用于查找硬件资源
   * @param app LibXR::ApplicationManager 的引用
   * @param task_stack_depth 任务堆栈深度
   * @param param_pid_yaw_speed 云台yaw轴速度环PID参数
   * @param param_pid_yaw_angle 云台yaw轴位置环PID参数
   * @param param_pid_pitch_speed 云台pitch轴速度环PID参数
   * @param param_pid_pitch_angle 云台pitch轴位置环PID参数
   * @param motor_yaw 指向yaw电机实例的指针
   * @param motor_pitch 指向pitch电机实例的指针
   * @param gimbal_param 云台物理参数
   * @param gimbal_cmd_name 云台控制命令主题名称
   * @param accl_name 加速度计数据主题名称
   * @param euler_name 欧拉角数据主题名称
   * @param gyro_name 陀螺仪数据主题名称
   */
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param param_pid_yaw_speed,
         LibXR::PID<float>::Param param_pid_yaw_angle,
         LibXR::PID<float>::Param param_pid_pitch_speed,
         LibXR::PID<float>::Param param_pid_pitch_angle, RMMotor *motor_pitch,
         RMMotor *motor_yaw,CMD *cmd,GimbalParam &&gimbal_param)
      : pid_yaw_speed_(param_pid_yaw_speed),
        pid_yaw_angle_(param_pid_yaw_angle),
        pid_pitch_speed_(param_pid_pitch_speed),
        pid_pitch_angle_(param_pid_pitch_angle),
        motor_yaw_(motor_yaw),
        motor_pitch_(motor_pitch),
        GIMBALPARAM(std::move(gimbal_param)),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(RELAX);
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->EventHandler(event_id);
        },
        this);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON),
                           callback);

    this->thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                         LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 线程函数
   *
   * @param gimbal
   */
  static void ThreadFunc(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> gimbal_cmd_subs("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> gimbal_euler_subs(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gimbal_gyro_subs(
        "bmi088_gyro");
    gimbal_cmd_subs.StartWaiting();
    gimbal_euler_subs.StartWaiting();
    gimbal_gyro_subs.StartWaiting();

    while (true) {
      if (gimbal_cmd_subs.Available()) {
        gimbal->gimbal_cmd_ = gimbal_cmd_subs.GetData();
        gimbal_cmd_subs.StartWaiting();
      }
      if (gimbal_euler_subs.Available()) {
        gimbal->euler_ = gimbal_euler_subs.GetData();
        gimbal_euler_subs.StartWaiting();
      }
      if (gimbal_gyro_subs.Available()) {
        gimbal->gyro_ = gimbal_gyro_subs.GetData();
        gimbal_gyro_subs.StartWaiting();
      }
      gimbal->mutex_.Lock();
      gimbal->UpdateFeedBack();
      gimbal->Caculate();

      gimbal->mutex_.Unlock();
      gimbal->Control();

      LibXR::Thread::Sleep(2); /* 1KHz电流环 */
    }
  }

  /**
   * @brief 更新反馈和dt
   *
   */
  void UpdateFeedBack() {
    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_wakeup_).ToSecondf();
    this->last_wakeup_ = now;

    this->motor_yaw_->Update();
    this->motor_pitch_->Update();
    /*注意极性*/
    this->pit_ = euler_.Pitch();
    this->yaw_ = euler_.Yaw();

    this->pit_dot_ = gyro_.y();
    this->yaw_dot_ = gyro_.z();

    float gimbal_pit_cmd = 0.0f;
    float gimbal_yaw_cmd = 0.0f;

    if (cmd_->GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      gimbal_pit_cmd = this->gimbal_cmd_.pit * this->dt_ * 5.0f;
      gimbal_yaw_cmd = this->gimbal_cmd_.yaw * this->dt_ * 5.0f;
      if (current_mode_ == LOB) {
        gimbal_pit_cmd = this->gimbal_cmd_.pit * this->dt_ * 0.1f;
        gimbal_yaw_cmd = this->gimbal_cmd_.yaw * this->dt_ * 0.1f;
      }
    } else {
      gimbal_pit_cmd = gimbal_cmd_.pit - this->pit_;
      gimbal_yaw_cmd = gimbal_cmd_.yaw - this->yaw_;
    }

    /* 处理pitch控制命令，软件限位 */
    if (GIMBALPARAM.pitch_max != GIMBALPARAM.pitch_min) {
      if (GIMBALPARAM.motor_is_reverse) {
        const float ENCODER_DELTA_MAX_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->GIMBALPARAM.pitch_max;
        const float ENCODER_DELTA_MIN_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->GIMBALPARAM.pitch_min;
        const float PIT_ERR = this->setpoint_pit_ - this->pit_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        gimbal_pit_cmd =
            std::clamp(gimbal_pit_cmd, DELTA_MIN_PIT, DELTA_MAX_PIT);
      } else {
        const float ENCODER_DELTA_MAX_PIT =
            this->GIMBALPARAM.pitch_max -
            LibXR::CycleValue(motor_pitch_->GetAngle());
        const float ENCODER_DELTA_MIN_PIT =
            this->GIMBALPARAM.pitch_min -
            LibXR::CycleValue(motor_pitch_->GetAngle());
        const float PIT_ERR = this->setpoint_pit_ - this->pit_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        gimbal_pit_cmd =
            std::clamp(gimbal_pit_cmd, DELTA_MIN_PIT, DELTA_MAX_PIT);
      }
    }

    this->setpoint_pit_ += gimbal_pit_cmd;

    this->setpoint_yaw_ += gimbal_yaw_cmd;
  }
  void Caculate() {
    switch (current_mode_) {
      case RELAX:
        out_pit_ = 0;
        out_yaw_ = 0;
        break;
      case COMMON:
        out_pit_dot_ = pid_pitch_angle_.Calculate(setpoint_pit_, this->pit_,
                                                  this->pit_dot_, dt_);
        out_pit_ =
            pid_pitch_speed_.Calculate(out_pit_dot_, this->pit_dot_, dt_);
        out_yaw_dot_ = pid_yaw_angle_.Calculate(setpoint_yaw_, this->yaw_,
                                                this->yaw_dot_, dt_);
        out_yaw_ = pid_yaw_speed_.Calculate(out_yaw_dot_, this->yaw_dot_, dt_);
        break;
      default:
        break;
    }
  }

  /**
   * @brief 这个函数用来控制云台运动，内部只负责跟随pos_aim_这个变量
   *
   */
  void Control() {
    switch (current_mode_) {
      case RELAX:
      case COMMON:
        this->motor_pitch_->VoltageControl(out_pit_, 0x205);
        this->motor_yaw_->VoltageControl(out_yaw_, 0x20A);
        break;
      default:
        break;
    }
  }

  /**
   * @brief 获取底盘的事件处理器
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event &GetEvent() { return gimbal_event_; }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) {
    switch (static_cast<GimbalEvent>(event_id)) {
      case GimbalEvent::SET_MODE_RELAX:
        SetMode(GimbalMode::RELAX);
        break;
      case GimbalEvent::SET_MODE_COMMON:
        SetMode(GimbalMode::COMMON);
        break;
      default:
        break;
    }
  }

  void SetMode(GimbalMode mode) {
    if (current_mode_ == RELAX) {
      if (mode == COMMON or mode == LOB) {
        setpoint_yaw_ = yaw_;
        setpoint_pit_ = pit_;
      }
    }
    pid_pitch_angle_.Reset();
    pid_pitch_speed_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_speed_.Reset();
    mutex_.Lock();
    current_mode_ = mode;
    mutex_.Unlock();
  }
  float AngleDistance(float a, float b) {
    float diff = b - a;
    // 规范化到 [-π, π]
    while (diff > M_PI) {diff -= 2 * M_PI;};
    while (diff < -M_PI) {diff += 2 * M_PI;};
    return diff;
  }
  void OnMonitor() override {}

 private:
  GimbalMode current_mode_ = GimbalMode::RELAX;
  LibXR::Event gimbal_event_;
  CMD::GimbalCMD gimbal_cmd_;
  LibXR::EulerAngle<float> euler_;
  Eigen::Matrix<float, 3, 1> gyro_;
  float pit_, pit_dot_, yaw_, yaw_dot_;
  float setpoint_pit_, setpoint_yaw_;
  float out_pit_, out_pit_dot_;
  float out_yaw_, out_yaw_dot_;

  /* PID */
  LibXR::PID<float> pid_yaw_speed_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_pitch_speed_;
  LibXR::PID<float> pid_pitch_angle_;

  /* 电机 */
  RMMotor *motor_yaw_;
  RMMotor *motor_pitch_;

  /* 云台定义 */
  const GimbalParam GIMBALPARAM;

  /* 线程管理 */
  LibXR::MicrosecondTimestamp last_wakeup_;
  float dt_ = 0.0;
  LibXR::Topic topic_yaw_;
  LibXR::Topic topic_yaw_dot_;
  CMD *cmd_;

  LibXR::Mutex mutex_;
  LibXR::Thread thread_;
};
