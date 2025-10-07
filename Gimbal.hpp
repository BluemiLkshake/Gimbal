#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_angle_param_yaw:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_angle_param_pitch:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_param_yaw:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_param_pitch:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - gimbal_pitch_center: 0.0
  - gimbal_yaw_center: 0.0
  - gimbal_max: 0.0
  - gimbal_min: 0.0
  - gimbal_pitch_mass: 0.0
  - radius_pitch_to_center_of_mass:
      x: 0.0
      y: 0.0
      z: 0.0
  - radius_yaw_to_pitch:
      x: 0.0
      y: 0.0
      z: 0.0
  - rotation_pitch_from_imu:
      m: [[1.0, 0.0, 0.0],
          [0.0, 1.0, 0.0],
          [0.0, 0.0, 1.0]]
  - gimbal_cmd_topic_name: 'gimbal_cmd'
  - accl_topic_name: 'bmi088_accl'
  - euler_topic_name: 'ahrs_euler'
  - gyro_topic_name: 'bmi088_gyro'
template_args:
  - MotorType: RMMotorContainer
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "BMI088.hpp"
#include "CMD.hpp"
#include "Eigen/Core"
#include "MadgwickAHRS.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define GIMBAL_MAX_SPEED (M_2PI * 1.5f)

template <typename MotorType>
class Gimbal : public LibXR::Application {
 public:
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param pid_angle_param_yaw,
         LibXR::PID<float>::Param pid_angle_param_pitch,
         LibXR::PID<float>::Param pid_omega_param_yaw,
         LibXR::PID<float>::Param pid_omega_param_pitch,
         Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
         float gimbal_pitch_center_, float gimbal_yaw_center_,
         float gimbal_max_, float gimbal_min_, float gimbal_pitch_mass_,
         const LibXR::Position<float> &radius_pitch_to_center_of_mass,
         const LibXR::Position<float> &radius_yaw_to_pitch,
         const LibXR::RotationMatrix<float> &rotation_pitch_from_imu,
         const char *gimbal_cmd_topic_name, const char *accl_topic_name,
         const char *euler_topic_name, const char *gyro_topic_name)
      : gimbal_cmd_name_(gimbal_cmd_topic_name),
        accl_name_(accl_topic_name),
        euler_name_(euler_topic_name),
        gyro_name_(gyro_topic_name),
        gimbal_pitch_center_(gimbal_pitch_center_),
        gimbal_yaw_center_(gimbal_yaw_center_),
        gimbal_max_(gimbal_max_),
        gimbal_min_(gimbal_min_),
        gimbal_pitch_mass_(gimbal_pitch_mass_),
        radius_pitch_to_center_of_mass_(radius_pitch_to_center_of_mass),
        radius_yaw_to_pitch_(radius_yaw_to_pitch),
        rotation_pitch_from_imu_(rotation_pitch_from_imu),
        pid_angle_yaw_(pid_angle_param_yaw),
        pid_angle_pitch_(pid_angle_param_pitch),
        pid_omega_yaw_(pid_omega_param_yaw),
        pid_omega_pitch_(pid_omega_param_pitch),
        motor_can1_(motor_can1),
        motor_can2_(motor_can2),
        cmd_(cmd) {
    thread_.Create(this, ThreadFunction, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }
  static void ThreadFunction(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(
        gimbal->gimbal_cmd_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        gimbal->accl_name_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->euler_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        gimbal->gyro_name_);

    auto now_ = LibXR::Timebase::GetMicroseconds();
    gimbal->dt_ = (now_ - gimbal->last_online_time_);
    gimbal->last_online_time_ = now_;
    cmd_suber.StartWaiting();

    while (true) {
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

      gimbal->UpdateSetpointFromCMD();
      gimbal->SelfResolution();
      gimbal->GravityCompensation(gimbal->accl_data_);
      gimbal->OutputToDynamics();
    }
  }

  void UpdateSetpointFromCMD() {
    float gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED;
    float gimbal_pitch_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED;

    const float pitch_err_ = target_angle_pitch_ - euler_.Pitch();
    const float ENCODER_DELTA_MAX_PIT = gimbal_max_ - motor_can2_.GetAngle(5);
    const float ENCODER_DELTA_MIN_PIT = gimbal_min_ - motor_can2_.GetAngle(5);
    const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - pitch_err_;
    const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - pitch_err_;
    gimbal_yaw_cmd = std::clamp(gimbal_yaw_cmd, DELTA_MIN_PIT, DELTA_MAX_PIT);

    target_angle_yaw_ += gimbal_yaw_cmd;
    target_angle_pitch_ += gimbal_pitch_cmd;
  }

  void Update() {
    motor_can1_.Update(6);
    motor_can2_.Update(5);
  }

  void SelfResolution() {
    now_angle_pitch_ = motor_can2_.GetAngle(5);
    now_angle_yaw_ = motor_can1_.GetAngle(6);
    now_omega_pitch_ = motor_can2_.GetSpeed(5);
    now_omega_yaw_ = motor_can1_.GetSpeed(6);
  }

  void GravityCompensation(Eigen::Matrix<float, 3, 1> &accl_data_) {
    // 1. 计算从Pitch坐标系(P)到Yaw坐标系(Y)的旋转矩阵
    const float cos_p = std::cos(now_angle_pitch_);
    const float sin_p = std::sin(now_angle_pitch_);
    LibXR::RotationMatrix<float> rotation_yaw_from_pitch(cos_p, 0, sin_p, 0, 1,
                                                         0, -sin_p, 0, cos_p);

    // 2. 坐标变换 (手动矩阵-向量乘法)
    // 将IMU加速度从IMU坐标系转换到Pitch坐标系(P)
    Eigen::Matrix<float, 3, 1> acceleration_in_pitch(
        rotation_pitch_from_imu_.operator()(0, 0) * accl_data_.x() +
            rotation_pitch_from_imu_.operator()(0, 1) * accl_data_.y() +
            rotation_pitch_from_imu_.operator()(0, 2) * accl_data_.z(),
        rotation_pitch_from_imu_.operator()(1, 0) * accl_data_.x() +
            rotation_pitch_from_imu_.operator()(1, 1) * accl_data_.y() +
            rotation_pitch_from_imu_.operator()(1, 2) * accl_data_.z(),
        rotation_pitch_from_imu_.operator()(2, 0) * accl_data_.x() +
            rotation_pitch_from_imu_.operator()(2, 1) * accl_data_.y() +
            rotation_pitch_from_imu_.operator()(2, 2) * accl_data_.z());

    // 将加速度从Pitch坐标系(P)转换到Yaw坐标系(Y)
    Eigen::Matrix<float, 3, 1> acceleration_in_yaw(
        rotation_yaw_from_pitch(0, 0) * acceleration_in_pitch.x() +
            rotation_yaw_from_pitch(0, 1) * acceleration_in_pitch.y() +
            rotation_yaw_from_pitch(0, 2) * acceleration_in_pitch.z(),
        rotation_yaw_from_pitch(1, 0) * acceleration_in_pitch.x() +
            rotation_yaw_from_pitch(1, 1) * acceleration_in_pitch.y() +
            rotation_yaw_from_pitch(1, 2) * acceleration_in_pitch.z(),
        rotation_yaw_from_pitch(2, 0) * acceleration_in_pitch.x() +
            rotation_yaw_from_pitch(2, 1) * acceleration_in_pitch.y() +
            rotation_yaw_from_pitch(2, 2) * acceleration_in_pitch.z());

    // 3. 计算扰动力矩
    // 扰动力 F_disturbance = -m * a_imu
    LibXR::Position<float> force_in_pitch(
        acceleration_in_pitch.x() * -gimbal_pitch_mass_,
        acceleration_in_pitch.y() * -gimbal_pitch_mass_,
        acceleration_in_pitch.z() * -gimbal_pitch_mass_);
    LibXR::Position<float> force_in_yaw(
        acceleration_in_yaw.x() * -gimbal_pitch_mass_,
        acceleration_in_yaw.y() * -gimbal_pitch_mass_,
        acceleration_in_yaw.z() * -gimbal_pitch_mass_);

    // 在Pitch坐标系(P)下计算Pitch轴的扰动力矩 (手动叉乘 n = r x F)
    LibXR::Position<float> torque_vector_pitch(
        radius_pitch_to_center_of_mass_.y() * force_in_pitch.z() -
            radius_pitch_to_center_of_mass_.z() * force_in_pitch.y(),
        radius_pitch_to_center_of_mass_.z() * force_in_pitch.x() -
            radius_pitch_to_center_of_mass_.x() * force_in_pitch.z(),
        radius_pitch_to_center_of_mass_.x() * force_in_pitch.y() -
            radius_pitch_to_center_of_mass_.y() * force_in_pitch.x());

    // 在Yaw坐标系(Y)下计算Yaw轴的扰动力矩
    // 计算力臂 r: 从Yaw轴心到负载重心的向量
    LibXR::Position<float> radius_pitch_to_center_of_mass_in_yaw(
        rotation_yaw_from_pitch(0, 0) * radius_pitch_to_center_of_mass_.x(),
        rotation_yaw_from_pitch(1, 0) * radius_pitch_to_center_of_mass_.x(),
        rotation_yaw_from_pitch(2, 0) * radius_pitch_to_center_of_mass_.x());
    LibXR::Position<float> radius_yaw_to_center_of_mass(
        radius_yaw_to_pitch_.x() + radius_pitch_to_center_of_mass_in_yaw.x(),
        radius_yaw_to_pitch_.y() + radius_pitch_to_center_of_mass_in_yaw.y(),
        radius_yaw_to_pitch_.z() + radius_pitch_to_center_of_mass_in_yaw.z());

    // 手动叉乘 n = r x F
    LibXR::Position<float> torque_vector_yaw(
        radius_yaw_to_center_of_mass.y() * force_in_yaw.z() -
            radius_yaw_to_center_of_mass.z() * force_in_yaw.y(),
        radius_yaw_to_center_of_mass.z() * force_in_yaw.x() -
            radius_yaw_to_center_of_mass.x() * force_in_yaw.z(),
        radius_yaw_to_center_of_mass.x() * force_in_yaw.y() -
            radius_yaw_to_center_of_mass.y() * force_in_yaw.x());

    // Pitch电机绕其自身的Y轴旋转, 取y分量
    pitch_torque_ = torque_vector_pitch.y();
    // Yaw电机绕其自身的Z轴旋转, 取z分量
    yaw_torque_ = torque_vector_yaw.z();
  }

  float Constrain(float *x, float Min, float Max) {
    if (*x < Min) {
      *x = Min;
    } else if (*x > Max) {
      *x = Max;
    }
    return (*x);
  }

  void MotorNearestTransposition() {
    float tmp_delta_angle_ = 0.0f;
    tmp_delta_angle_ = fmod(target_angle_yaw_ - now_angle_yaw_, 2.0f * M_PI);
    if (tmp_delta_angle_ > M_PI) {
      tmp_delta_angle_ -= 2.0f * M_PI;
    } else if (tmp_delta_angle_ < -M_PI) {
      tmp_delta_angle_ += 2.0f * M_PI;
    }
    target_angle_yaw_ = motor_can1_.GetAngle(6) + tmp_delta_angle_;

    Constrain(&target_angle_pitch_, gimbal_min_, gimbal_max_);
    tmp_delta_angle_ = target_angle_pitch_ - now_angle_pitch_;
    target_angle_pitch_ = -motor_can2_.GetAngle(5) + tmp_delta_angle_;
  }

  void OutputToDynamics() {
    MotorNearestTransposition();

    float target_omega_yaw =
        pid_angle_yaw_.Calculate(target_angle_yaw_, euler_.Pitch(), dt_);
    float target_omega_pitch =
        pid_angle_pitch_.Calculate(target_angle_pitch_, euler_.Yaw(), dt_);

    float feedback_yaw =
        pid_omega_yaw_.Calculate(target_omega_yaw, gyro_data_.z(), dt_);
    float feedback_pitch =
        pid_omega_pitch_.Calculate(target_omega_pitch, gyro_data_.x(), dt_);

    float output_yaw = feedback_yaw + yaw_torque_;
    float output_pitch = feedback_pitch + pitch_torque_;

    output_yaw = std::clamp(output_yaw, -max_current_, max_current_);
    output_pitch = std::clamp(output_pitch, -max_current_, max_current_);

    motor_can1_.SetCurrent(6, output_yaw);
    motor_can2_.SetCurrent(5, output_pitch);
  }

  void OnMonitor() override {}

 private:
  const char *gimbal_cmd_name_;
  const char *accl_name_;
  const char *euler_name_;
  const char *gyro_name_;

  float now_angle_yaw_ = 0.0f;
  float now_angle_pitch_ = 0.0f;
  float target_angle_yaw_ = 0.0f;
  float target_angle_pitch_ = 0.0f;
  float now_omega_yaw_ = 0.0f;
  float now_omega_pitch_ = 0.0f;
  float pitch_torque_ = 0.0f;
  float yaw_torque_ = 0.0f;
  float gimbal_pitch_center_ = 0.0f;
  float gimbal_yaw_center_ = 0.0f;
  float gimbal_max_ = 0.0f;
  float gimbal_min_ = 0.0f;

  // Pitch轴固连负载的质量 (kg)
  const float gimbal_pitch_mass_ = 0.0f;
  // P系下, Pitch轴心->负载重心 的向量
  const LibXR::Position<float> radius_pitch_to_center_of_mass_{0.0f, 0.0f,
                                                               0.0f};
  // Y系下, Yaw轴心->Pitch轴心 的向量
  const LibXR::Position<float> radius_yaw_to_pitch_{0.0f, 0.0f, 0.0f};
  // IMU坐标系到Pitch坐标系的旋转矩阵
  const LibXR::RotationMatrix<float> rotation_pitch_from_imu_{
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const LibXR::Position<float> imu_acceleration_{0.0f, 0.0f, 0.0f};

  const float max_current_ = 1.0f;

  LibXR::PID<float> pid_angle_yaw_;
  LibXR::PID<float> pid_angle_pitch_;
  LibXR::PID<float> pid_omega_yaw_;
  LibXR::PID<float> pid_omega_pitch_;

  Motor<MotorType> &motor_can1_;
  Motor<MotorType> &motor_can2_;

  CMD &cmd_;
  CMD::GimbalCMD cmd_data_;

  LibXR::Thread thread_;

  LibXR::MicrosecondTimestamp::Duration dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::MicrosecondTimestamp now_ = 0;

  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  LibXR::EulerAngle<float> euler_;
};
