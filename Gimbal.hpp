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
      I_pitch: 0.0
      I_yaw: 0.0
      M_pitch: 0.0
      G_pitch: 0.0
      pitch_min: 0.0
      pitch_max: 0.0
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

#include <math.h>

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
class Gimbal : public LibXR::Application {
 public:
  enum class GimbalEvent : uint8_t {
    SET_MODE_RELAX,
    SET_MODE_RELATIVE,
  };
  typedef enum : uint8_t {
    RELAX,
    RELATIVE,
  } GimbalMode;
  /**
   * @brief 用于定义云台物理参数的结构体
   *
   */
  struct GimbalParam {
    /* pitch和yaw的转动惯量 */
    float I_pitch;
    float I_yaw;
    float M_pitch; /* pitch的质心质量 */
    float
        G_pitch; /* pitch的质心位置，从pitch电机开始计算，模型是简化过的，用线密度估算*/

    /* pitch的上下限，单位和GetAngle()一样，弧度 */
    float pitch_min;
    float pitch_max;
  };

  /**
   * @brief 电机动力学反馈参数
   *
   */
  struct Feedback {
    float angle = 0.f;
    float speed = 0.f;
    float torque = 0.f;
  };

  /**
   * @brief 储存扭矩的结构体
   *
   */
  struct TorqueFeedForward {
    float yaw = 0.f;
    float pitch = 0.f;
  };

  /**
   * @brief 用于描述云台状态的结构体
   *
   */
  struct State {
    Eigen::Matrix<float, 3, 1> accl = {0, 0, 0};
    LibXR::EulerAngle<float> euler = {0, 0, 0};
    Eigen::Matrix<float, 3, 1> gyro = {0, 0, 0};

    Feedback yaw_feedback_;
    Feedback pitch_feedback_;
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
         RMMotor *motor_yaw, CMD *cmd, GimbalParam gimbal_param)
      : pid_yaw_speed_(param_pid_yaw_speed),
        pid_yaw_angle_(param_pid_yaw_angle),
        pid_pitch_speed_(param_pid_pitch_speed),
        pid_pitch_angle_(param_pid_pitch_angle),
        motor_yaw_(motor_yaw),
        motor_pitch_(motor_pitch),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(gimbal_param);
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event) {
          UNUSED(in_isr);
          gimbal->EventHandler(event);
        },
        this);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(RELAX);
        },
        this);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_RELATIVE), callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    this->thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                         LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 线程函数
   *
   * @param gimbal
   */
  static void ThreadFunc(Gimbal *gimbal) {
    // auto last_time = LibXR::Timebase::GetMilliseconds();
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> gimbal_cmd_subs("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gimbal_accl_subs(
        "bmi088_accl");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> gimbal_euler_subs(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gimbal_gyro_subs(
        "bmi088_gyro");
    gimbal_cmd_subs.StartWaiting();
    gimbal_accl_subs.StartWaiting();
    gimbal_euler_subs.StartWaiting();
    gimbal_gyro_subs.StartWaiting();

    while (true) {
      if (gimbal_cmd_subs.Available()) {
        gimbal->gimbal_cmd_ = gimbal_cmd_subs.GetData();
        gimbal_cmd_subs.StartWaiting();
      }
      if (gimbal_accl_subs.Available()) {
        gimbal->current_state_.accl = gimbal_accl_subs.GetData();
        gimbal_accl_subs.StartWaiting();
      }
      if (gimbal_euler_subs.Available()) {
        gimbal->current_state_.euler = gimbal_euler_subs.GetData();
        gimbal_euler_subs.StartWaiting();
      }
      if (gimbal_gyro_subs.Available()) {
        gimbal->current_state_.gyro = gimbal_gyro_subs.GetData();
        gimbal_gyro_subs.StartWaiting();
      }

      gimbal->mutex_.Lock();

      gimbal->UpdateFeedBack();
      // TODO: 加上判断控制源
      gimbal->mutex_.Unlock();
      gimbal->Control();

      LibXR::Thread::Sleep(3); /* 1KHz电流环 */
    }
  }

  /**
   * @brief 更新反馈和dt
   *
   */
  void UpdateFeedBack() {
    // TODO: 通过旋转矩阵调整pit，yaw极性
    auto now = LibXR::Timebase::GetMilliseconds();
    this->dt_ = (now - this->last_wakeup_).ToSecondf();
    this->last_wakeup_ = now;

    this->motor_yaw_->Update();
    this->motor_pitch_->Update();

    this->current_state_.pitch_feedback_.speed =
      this->motor_pitch_->GetOmega();
  }

  /**
   * @brief 用来获取yaw和pitch重力补偿的函数
   *
   * @param tff_buf 存放重力补偿扭矩的地方
   * @param current_state 描述云台当前的状态
   */
  void GetGravityFeedForward(TorqueFeedForward &tff_buf,
                             const State &current_state) {
    // TODO:力控云台用的，在这里算重力前馈
  }

  /**
   * @brief 用来获取yaw和pitch向心力补偿的函数（可能不需要这个函数？）
   *
   * @param tff_buf 存放补偿扭矩的地方
   * @param current_state 描述云台当前的状态
   */
  void GetMotionalFeedForward(TorqueFeedForward &tff_buf,
                              const State &current_state) {
    // TODO:力控云台用的，在这里算运动带来的前馈扭矩
  }

  /**
   * @brief 这个函数用来控制云台运动，内部只负责跟随pos_aim_这个变量
   *
   */
  void Control() {
    static float pitch_speed_cmd = 0.0f;
    switch (current_mode_) {
      case RELAX:
        this->motor_yaw_->TorqueControl(0, 1);
        this->motor_pitch_->TorqueControl(0, 1);

        break;
      case RELATIVE: {
        pitch_speed_cmd = this->gimbal_cmd_.pit;

        // pitch_torque = this->pid_pitch_speed_.Calculate(
        //     pitch_speed_cmd, motor_pitch_->GetOmega(), this->dt_);

        this->motor_yaw_->TorqueControl(0, 1);
        this->motor_pitch_->TorqueControl(pitch_speed_cmd, 1);

        ozone_pitch_ = this->current_state_.euler.Pitch();
        ozone_gyro_y_ = motor_pitch_->GetOmega();
        ozone_aimp_ = pitch_speed_cmd;

        break;
      }
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
      case GimbalEvent::SET_MODE_RELATIVE:
        SetMode(GimbalMode::RELATIVE);
        break;
      default:
        break;
    }
  }

  void SetMode(GimbalMode mode) {
    mutex_.Lock();
    current_mode_ = mode;
    mutex_.Unlock();
  }
  void OnMonitor() override {}

 private:
  float ozone_gyro_y_ = 0.0f;
  float ozone_pitch_ = 0.0f;
  float ozone_aimp_ = 0.0f;
  GimbalMode current_mode_ = GimbalMode::RELAX;
  LibXR::Event gimbal_event_;

  /* PID */
  LibXR::PID<float> pid_yaw_speed_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_pitch_speed_;
  LibXR::PID<float> pid_pitch_angle_;
  LibXR::CycleValue<float> pitch_imu_;
  LibXR::CycleValue<float> pitch_aim_;
  /* 扭矩补偿类 */
  TorqueFeedForward tff_gravity_; /* 重力矩 */
  TorqueFeedForward tff_motion_;  /* I * a */

  /* 电机 */
  RMMotor *motor_yaw_;
  RMMotor *motor_pitch_;

  CMD *cmd_;

  /* 云台定义 */
  // const GimbalParam GIMBALPARAM;

  /* 控制相关的变量 */
  LibXR::EulerAngle<float> pos_aim_; /* CycleValue */
  State current_state_;              /* 当前云台的状态 */
  CMD::GimbalCMD gimbal_cmd_;

  /* 线程管理 */
  LibXR::MillisecondTimestamp last_wakeup_ = 0;
  float dt_ = 0.0;

  LibXR::Mutex mutex_;
  LibXR::Thread thread_;

  /* 工具函数 */

  /**
   * @brief 用来把x映射到[lo_after, hi_after]的函数
   *
   * @param x
   * @param lo_before 映射前的下限
   * @param hi_before 映射前的上限999
   * @param lo_after
   * @param hi_after
   * @return float
   */
  float LinearMap(float x, float lo_before, float hi_before, float lo_after,
                  float hi_after) {
    return ((x - lo_before) / (hi_before - lo_before)) * (hi_after - lo_after) +
           lo_after;
  }
};
