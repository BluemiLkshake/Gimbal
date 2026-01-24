#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 4096
  - pid_yaw_angle_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 50.0
      cycle: true
  - pid_pitch_angle_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 50.0
      cycle: true
  - pid_yaw_omega_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 3.0
      cycle: false
  - pid_pitch_omega_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 3.0
      cycle: false
  - motor_pitch: '@&motor_pit'
  - motor_yaw: '@&motor_yaw'
  - limit:
      max_pitch_: 0.0
      min_pitch_: 0.0
      max_yaw_: 0.0
      min_yaw_: 0.0
      reverse_pitch_: false
      reverse_yaw_: false
      J_pitch_: 0.0
      J_yaw_: 0.0
  - now_param:
      now_pitch_angle_: 0.0
      now_yaw_angle_: 0.0
      now_pitch_omega_: 0.0
      now_yaw_omega_: 0.0
      now_pitch_current_: 0.0
      now_yaw_current_: 0.0
  - tar_param:
      target_yaw_angle_: 0.0
      target_pitch_angle_: 0.0
      target_yaw_omega_: 0.0
      target_pitch_omega_: 0.0
      target_yaw_current_: 0.0
      target_pitch_current_: 0.0
  - gimbal_cmd_topic_name: gimbal_cmd
  - accl_topic_name: bmi088_accl
  - euler_topic_name: ahrs_euler
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

/**
 * @brief 云台
 * @author BluemiLk
 * @date 2026/1/24
 * @note 前馈+pid双环控制
 * @email 有问题请联系2317235934@qq.com
 * @warning 使用前重新配置旋转矩阵wxyz
 * @warning 确保电机位置gyro_data.y真的对应的是你的pitch(yaw轴同理)
 * @warning yaw,pitch轴限位的值是用的6020反馈的值(0~M_2PI)
 * @warning 如果无法精确测量云台的转动惯量J从小开始调(比如0.00001)
 */
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "BMI088.hpp"
#include "CMD.hpp"
#include "Eigen/Core"
#include "MadgwickAHRS.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "inertia.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

// 遥控器灵敏度
#define GIMBAL_MAX_SPEED (static_cast<float>(M_2PI) * 1.5f)
// 转矩常数 0.741N*m/A
#define TORQUE_CONSTANT (0.741f)
// GM6020电机最大输出电流值 3A
#define MAX_CURRENT (3.0f)
// DM4310电机最大输出扭矩值 10N*m
#define MAX_TORQUE (10.0f)
// 滤波器滤波系数
#define FILTER_SIZE (6)

template <typename MotorTypePitch, typename MotorTypeYaw>
class Gimbal : public LibXR::Application {
 public:
  enum class GimbalEvent : uint8_t {
    SET_MODE_RELAX,
    SET_MODE_INDEPENDENT,
    SET_MODE_AUTOAIM,
  };

  typedef enum : uint8_t { RELAX, INDEPENDENT, AUTOAIM } GimbalMode;

  struct NowParam {
    // pitch轴当前角度(rad)
    float now_pitch_angle_ = 0.0f;
    // yaw轴当前角度(rad)
    float now_yaw_angle_ = 0.0f;
    // pitch轴当前角速度(rad/s)
    float now_pitch_omega_ = 0.0f;
    // yaw轴当前角速度(rad/s)
    float now_yaw_omega_ = 0.0f;
    // pitch轴当前电流(A)
    float now_pitch_current_ = 0.0f;
    // yaw轴当前电流(A)
    float now_yaw_current_ = 0.0f;
  };

  struct TarParam {
    // yaw轴目标角度(rad)
    float target_yaw_angle_ = 0.0f;
    // pitch轴目标角度(rad)
    float target_pitch_angle_ = 0.0f;
    // yaw轴目标角速度(rad/s)
    float target_yaw_omega_ = 0.0f;
    // pitch轴目标角速度(rad/s)
    float target_pitch_omega_ = 0.0f;
    // yaw轴目标输出电流(A)
    float target_yaw_current_ = 0.0f;
    // pitch轴目标电流(A)
    float target_pitch_current_ = 0.0f;
  };

  struct Limit {
    // pitch轴最大值(0~2π)
    float max_pitch_ = 0.0f;
    // pitch轴最小值(0~2π)
    float min_pitch_ = 0.0f;
    // yaw轴最大值(0~2π)
    float max_yaw_ = 0.0f;
    // yaw轴最小值(0~2π)
    float min_yaw_ = 0.0f;
    // 与限位有关pitch轴转动方向
    bool reverse_pitch_ = false;
    // 与限位有关yaw轴转动方向
    bool reverse_yaw_ = false;
    // pitch轴转动惯量(kg·m²)
    float J_pitch_ = 0.0f;
    // yaw轴转动惯量(kg·m²)
    float J_yaw_ = 0.0f;
  };

  /**
   * @brief 构造函数初始化数据成员
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
   * @param task_stack_depth 任务堆栈深度
   * @param pid_yaw_angle_param   Yaw轴角度环PID参数
   * @param pid_pitch_angle_param Pitch轴角度环PID参数
   * @param pid_yaw_omega_param   Yaw轴角速度环PID参数
   * @param pid_pitch_omega_param Pitch轴角速度环PID参数
   * @param gimbal_cmd_topic_name 云台命令Topic名称
   * @param accl_topic_name  加速度计数据Topic名称
   * @param euler_topic_name 欧拉角数据Topic名称
   */
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param pid_yaw_angle_param,
         LibXR::PID<float>::Param pid_pitch_angle_param,
         LibXR::PID<float>::Param pid_yaw_omega_param,
         LibXR::PID<float>::Param pid_pitch_omega_param,
         MotorTypePitch *motor_pitch,
         MotorTypeYaw *motor_yaw,
         Limit limit,
         NowParam now_param,
         TarParam tar_param, const char *gimbal_cmd_topic_name,
         const char *accl_topic_name, const char *euler_topic_name)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle_param),
        pid_pitch_angle_(pid_pitch_angle_param),
        pid_yaw_omega_(pid_yaw_omega_param),
        pid_pitch_omega_(pid_pitch_omega_param),
        motor_yaw_(motor_yaw),
        motor_pitch_(motor_pitch),
        gimbal_cmd_name_(gimbal_cmd_topic_name),
        accl_name_(accl_topic_name),
        euler_name_(euler_topic_name),
        limit_(limit),
        now_param_(now_param),
        tar_param_(tar_param) {
    UNUSED(hw);
    UNUSED(app);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(RELAX);
        },
        this);

    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->EventHandler(event_id);
        },
        this);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_INDEPENDENT),
                           callback);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOAIM),
                           callback);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 云台控制线程函数
   */
  static void ThreadFunc(Gimbal *gimbal) {
    // 订阅要用到的数据
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(
        gimbal->gimbal_cmd_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        gimbal->accl_name_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->euler_name_);
    // 陀螺仪数据
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "bmi088_gyro");

    cmd_suber.StartWaiting();
    accl_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    while (true) {
      auto last_time = LibXR::Timebase::GetMilliseconds();

      gimbal->mutex_.Lock();
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (accl_suber.Available()) {
        gimbal->accl_data_ = accl_suber.GetData();
        accl_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        gimbal->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gyro_suber.StartWaiting();
      }

      gimbal->Update();
      gimbal->SetpointFromCMD();
      gimbal->mutex_.Unlock();
      gimbal->OutputToDynamics();
      gimbal->thread_.SleepUntil(last_time, 2.0f);
    }
  }

  /**
   * @brief 更新函数
   */
  void Update() {
    // 更新电机状态
    motor_yaw_->Update();
    motor_pitch_->Update();
    // 更新dt_
    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    // 处理Yaw的IMU角度
    // euler.Yaw()数值变化规律: 0 ~ PI ~ -PI ~ -0 ~ 0
    now_param_.now_yaw_angle_ = euler_.Yaw();
    // 处理Pit的IMU角度
    // euler.Pit()数值变化规律: 0 ~ PI ~ -PI ~ -0 ~ 0
    now_param_.now_pitch_angle_ = -euler_.Pitch();

    // 获取的角速度(正负值 rad/s)
    now_param_.now_yaw_omega_ = gyro_data_.z();
    now_param_.now_pitch_omega_ = -gyro_data_.y();

    // pitch陀螺仪角速度数据滤波
    float gyro_buffer[FILTER_SIZE];
    float filtered_gyro = 0;
    // 更新滤波
    for (int i = FILTER_SIZE - 1; i > 0; i--) {
      gyro_buffer[i] = gyro_buffer[i - 1];
    }
    gyro_buffer[0] = now_param_.now_pitch_omega_;
    // 计算平均值
    filtered_gyro = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      filtered_gyro += gyro_buffer[i];
    }
    filtered_gyro /= FILTER_SIZE;
    now_param_.now_pitch_omega_ = filtered_gyro;

    // 更新上一状态电机状态
    last_yaw_angle_ = now_param_.now_yaw_angle_;
    last_pitch_angle_ = now_param_.now_pitch_angle_;
    last_yaw_omega_ = now_param_.now_yaw_omega_;
    last_pitch_omega_ = now_param_.now_pitch_omega_;

    // 发布云台当前角度给底盘
    topic_yaw_angle_.Publish(now_param_.now_yaw_angle_);
    topic_pitch_angle_.Publish(now_param_.now_pitch_angle_);
  }

  /**
   * @brief 处理遥控器的数据
   */
  void SetpointFromCMD() {
    // 归一化之后的遥控器值-1 ~ +1之间
    float gimbal_yaw_cmd = 0.0f;
    float gimbal_pitch_cmd = 0.0f;
    // 操作员控制模式
    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      // Gimbal_XXX_Cmd 角度(rad/s) | Gimbal_MAX_SPEED 灵敏度系数
      gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      gimbal_pitch_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      // 累加得到目标角度
      tar_param_.target_yaw_angle_ =
          tar_param_.target_yaw_angle_ + gimbal_yaw_cmd;
      tar_param_.target_pitch_angle_ =
          tar_param_.target_pitch_angle_ + gimbal_pitch_cmd;
    }
    // 自动控制模式
    else {
      // 查看AI云台是否在线
      // AI在线(开自瞄了，云台完全交给AI控制)
      if (cmd_.GetAIGimbalStatus()) {
        tar_param_.target_yaw_angle_ = cmd_data_.yaw;
        tar_param_.target_pitch_angle_ = cmd_data_.pit;
      }
      // AI离线(操作手介入，用遥控器数据)
      else {
        gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 0.05f;
        gimbal_pitch_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 0.05f;
        // 累加得到目标角度
        tar_param_.target_yaw_angle_ =
            tar_param_.target_yaw_angle_ + gimbal_yaw_cmd;
        tar_param_.target_pitch_angle_ =
            tar_param_.target_pitch_angle_ + gimbal_pitch_cmd;
      }
    }
    // pitch轴限位
    if (limit_.max_pitch_ != limit_.min_pitch_) {
      if (limit_.reverse_pitch_ == false) {
        const float ENCODER_DELTA_MAX_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.max_pitch_;
        const float ENCODER_DELTA_MIN_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.min_pitch_;
        const float PIT_ERR =
            tar_param_.target_pitch_angle_ - now_param_.now_pitch_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        tar_param_.target_pitch_angle_ = std::clamp(
            tar_param_.target_pitch_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      } else {
        const float ENCODER_DELTA_MAX_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.max_pitch_;
        const float ENCODER_DELTA_MIN_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.min_pitch_;
        const float PIT_ERR =
            tar_param_.target_pitch_angle_ - now_param_.now_pitch_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        tar_param_.target_pitch_angle_ = std::clamp(
            tar_param_.target_pitch_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      }
    }
  }

  /**
   *@brief 控制函数
   *
   */
  void OutputToDynamics() {
    /*达妙电机使能标志位*/
    /*让使能命令尽量只发一遍*/
    if (this->enable_flag_ == true) {
      if (this->dm_motor_flag_ == false) {
        for (int i = 0; i < 2; i++) {
          // 达妙电机使能
          // 在这里添加达妙电机使能命令
          this->dm_motor_flag_ = true;
        }
      }
    }
    switch (current_mode_) {
      case RELAX: {
        motor_yaw_->Relax();
        motor_pitch_->Relax();
        // 达妙电机失能
        // 在这里添加达妙电机失能命令
        enable_flag_ = false;
        dm_motor_flag_ = false;
      } break;
      case INDEPENDENT: {
        // 串级PID位置外环 角速度内环 + 力矩前馈
        // 位置环+前馈
        tar_param_.target_yaw_omega_ = pid_yaw_angle_.Calculate(
            tar_param_.target_yaw_angle_, now_param_.now_yaw_angle_, dt_);

        output_yaw_ =
            FeedforwardControl(tar_param_.target_yaw_omega_,
                               now_param_.now_yaw_omega_, dt_, limit_.J_yaw_);

        tar_param_.target_pitch_omega_ = pid_pitch_angle_.Calculate(
            tar_param_.target_pitch_angle_, now_param_.now_pitch_angle_, dt_);

        output_pitch_ = FeedforwardControl(tar_param_.target_pitch_omega_,
                                           now_param_.now_pitch_omega_, dt_,
                                           limit_.J_pitch_);

        // 速度环计算
        output_yaw_ += (pid_yaw_omega_.Calculate(tar_param_.target_yaw_omega_,
                                                 gyro_data_.z(), dt_));

        output_pitch_ += (pid_pitch_omega_.Calculate(
            tar_param_.target_pitch_omega_, -gyro_data_.y(), dt_));

        // 限幅
        output_yaw_ =
            std::clamp((output_yaw_), -(MAX_CURRENT * TORQUE_CONSTANT),
                       (MAX_CURRENT * TORQUE_CONSTANT));
        output_pitch_ =
            std::clamp((output_pitch_), -(MAX_CURRENT * TORQUE_CONSTANT),
                       (MAX_CURRENT * TORQUE_CONSTANT));
        // 力矩输出到电机([输出扭矩],[减速比])
        motor_yaw_->TorqueControl((-1.0f * output_yaw_), 1.0f);
        motor_pitch_->TorqueControl((1.0f * output_pitch_), 1.0f);
        enable_flag_ = true;
      } break;
      case AUTOAIM: {
        // 串级PID角度外环,角速度内环
        // yaw轴角度环
        tar_param_.target_yaw_omega_ = pid_yaw_angle_.Calculate(
            tar_param_.target_yaw_angle_, now_param_.now_yaw_angle_, dt_);
        // pitch轴角度环
        tar_param_.target_pitch_omega_ = pid_pitch_angle_.Calculate(
            tar_param_.target_pitch_angle_, now_param_.now_pitch_angle_, dt_);

        // yaw轴速度环
        output_yaw_ = (pid_yaw_omega_.Calculate(tar_param_.target_yaw_omega_,
                                                gyro_data_.z(), dt_));
        // pitch轴速度环
        output_pitch_ = (pid_pitch_omega_.Calculate(
            tar_param_.target_pitch_omega_, -gyro_data_.y(), dt_));
        // 输出力矩到电机
        motor_yaw_->TorqueControl((-1.0f * output_yaw_), 1.0f);
        motor_pitch_->TorqueControl((1.0f * output_pitch_), 1.0f);
        enable_flag_ = true;
      } break;
      default:
        break;
    }
  }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) {
    switch (static_cast<GimbalEvent>(event_id)) {
      case GimbalEvent::SET_MODE_RELAX:
        SetMode(GimbalMode::RELAX);
        break;
      case GimbalEvent::SET_MODE_INDEPENDENT:
        SetMode(GimbalMode::INDEPENDENT);
        break;
      case GimbalEvent::SET_MODE_AUTOAIM:
        SetMode(GimbalMode::AUTOAIM);
      default:
        break;
    }
  }

  /**
   * @brief 设置云台模式
   * @param mode 云台模式
   */
  void SetMode(GimbalMode mode) {
    if (current_mode_ == RELAX) {
      if (mode == INDEPENDENT) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pitch_angle_ = now_param_.now_pitch_angle_;
      }
    } else if (current_mode_ == INDEPENDENT) {
      if (mode == AUTOAIM || mode == RELAX) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pitch_angle_ = now_param_.now_pitch_angle_;
      }
    }
    pid_pitch_angle_.Reset();
    pid_pitch_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    mutex_.Lock();
    // 更新当前mode状态
    current_mode_ = mode;
    mutex_.Unlock();
  }

  LibXR::Event &GetEvent() { return gimbal_event_; }

  /**
   * @brief  前馈控制feedforward
   * @param J 电机转动惯量 (kg*m^2)
   * @note 用在速度环之前,位置环之后
   * @return 扭矩(N*m)
   */
  float FeedforwardControl(float target_omega, float now_omega, float dt_,
                           float J) {
    float out = 0.0f, delta_omega = 0.0f;
    delta_omega = target_omega - now_omega;
    out = (J * delta_omega / dt_);
    return out;  // N*m
  }

  void OnMonitor() override {}

  /*角度映射到[0, 2π]*/
  inline float WrapTo2PI(float angle) {
    // 将角度映射到[0, 2π]
    angle = static_cast<float>(fmod(angle, M_2PI));
    if (angle < 0) {
      angle += M_2PI;
    }
    return angle;
  }

  /*角度映射到[-π, π]*/
  inline float WrapToPi(float angle) {
    // 先取模到[-2π, 2π]范围内
    angle = static_cast<float>(fmod(angle, M_2PI));
    // 如果大于π，减去2π映射到[-π, π)
    if (angle > M_PI) {
      angle -= M_2PI;
    }
    // 如果小于-π，加上2π映射到(-π, π]
    else if (angle < -M_PI) {
      angle += M_2PI;
    }
    return angle;
  }

 private:
  // 命令模块实例
  CMD &cmd_;
  // PID声明
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_pitch_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pitch_omega_;
  // 电机
  MotorTypeYaw *motor_yaw_;
  MotorTypePitch *motor_pitch_;

  // 云台命令Topic名称
  const char *gimbal_cmd_name_;
  // 加速度计数据Topic名称
  const char *accl_name_;
  // 欧拉角数据Topic名称
  const char *euler_name_;

  // 角度限位
  Limit limit_;
  // 当前值
  NowParam now_param_;
  // 目标值
  TarParam tar_param_;

  // 达妙电机使能标志位
  bool enable_flag_ = false;
  bool dm_motor_flag_ = false;

  // 上一状态yaw轴角度
  float last_yaw_angle_ = 0.0f;
  // 上一状态pitch轴角度
  float last_pitch_angle_ = 0.0f;
  // 上一状态yaw轴角度
  float last_yaw_omega_ = 0.0f;
  // 上一状态pitch轴当前角度
  float last_pitch_omega_ = 0.0f;

  // 云台状态
  GimbalMode current_mode_ = GimbalMode::RELAX;

  float gyro_yaw_ = 0.0f;

  // 云台输出转动力矩
  float output_yaw_ = 0.0f;
  float output_pitch_ = 0.0f;

  // 给底盘传的云台角度
  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("chassis_yaw_");
  LibXR::Topic topic_pitch_angle_ =
      LibXR::Topic::CreateTopic<float>("chassis_pitch_");

  // 云台命令数据
  CMD::GimbalCMD cmd_data_;
  // 云台控制线程
  LibXR::Thread thread_;
  // 云台控制事件
  LibXR::Event gimbal_event_;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  float dt_ = 0.0f;  //(s)

  // 陀螺仪和加速度计数据
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  // 欧拉角数据
  LibXR::EulerAngle<float> euler_;

  LibXR::Mutex mutex_;
};
