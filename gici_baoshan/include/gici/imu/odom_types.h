/**
* @Function: IMU types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <Eigen/Core>

namespace gici {

// Odometer measurement  --sbs
enum class OdomRole {
  None,
  Skid,
};

struct OdomMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp; ///< In seconds.
  int id; ///< Odometer id
  Eigen::Vector3d forward_velocity;
  Eigen::Vector3d bearing_angular_velocity;
  OdomMeasurement() {}
  OdomMeasurement(
      const double timestamp,
      const Eigen::Vector3d forward_velocity,
      const Eigen::Vector3d bearing_angular_velocity)
  : timestamp(timestamp)
  , forward_velocity(forward_velocity)
  , bearing_angular_velocity(bearing_angular_velocity)
  {}


  // static int32_t epoch_cnt_ ;

};
typedef std::deque<OdomMeasurement> OdomMeasurements;

// A simple struct to specify properties of an Odometer.
struct OdomParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double v_max = 50;  ///< Velocity saturation. [m/s]
  double g_max = 7.8;  ///< Gyroscope saturation. [rad/s]
  double sigma_g_c = 1;  ///< Gyroscope noise density. [rad/s*1/sqrt(Hz)]
  double sigma_sg = 1; // Initial gyroscope scaler uncertainty
  ///< Initial gyroscope bias uncertainty. [rad/s*1/sqrt(Hz)]
  double sigma_v_c = 1;  ///< velocity noise density. [m/s*1/sqrt(Hz)]
  double sigma_sv = 0.001; // Initial velocity scaler uncertainty
  ///< Initial accelerometer bias uncertainty. [m/s^2*1/sqrt(Hz)]
  double sigma_gw_c = 2.1e-5; ///< Gyroscope drift noise density. [rad/s^2*1/sqrt(Hz)]
  double sigma_vw_c = 8.4e-4; ///< Accelerometer drift noise density. [m/s^2*1/sqrt(Hz)]
  ///< Mean of the prior acceleration bias. [m/s^2*1/sqrt(Hz)]
  double rate = 100;  ///< Odometer rate. [Hz].
  double delay_odom_cam = 0.0;
  ///< Camera-IMU delay: delay_imu_cam = cam_timestamp - imu_timestamp [s]

  Eigen::Vector3d l_S_Odom = Eigen::Vector3d::Zero(); ///< Odometer position in the IMU frame. [m]
  Eigen::Vector2d scaler = Eigen::Vector2d::Ones(); ///< [velocity, gyro scalers]

  
};

// [velocity, gyro scalers]
typedef Eigen::Matrix<double, 2, 1> OdomScaler;

}
