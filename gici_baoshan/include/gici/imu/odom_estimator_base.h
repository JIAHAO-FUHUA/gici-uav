/**
* @Function: Base class for Odometer estimators
*
* @Author  : Song Baoshan
* @Email   : baoshan.song@connect.polyu.hk
*
* Copyright (C) 2023 by Song Baoshan, All rights reserved.
**/
#pragma once

#include "gici/estimate/estimator_base.h"
// #include "gici/imu/imu_common.h"
#include "gici/imu/odom_types.h"
#include "gici/utility/transform.h"
#include "gici/utility/common.h"

namespace gici {

// Odometer estimator common options
struct OdomEstimatorBaseOptions {
  // Odometer parameters
  OdomParameters odom_parameters;


    // If use odometer
  bool use_forward_velocity = false;
  // (m/s)
  double forward_velocity_std = 0.1;  
  double forward_velocity_rw = 0.0001;

  // (rad/s)
  double bearing_angular_velocity_std = 0.01;  
  double bearing_angular_velocity_rw = 0.00001;


  double estimate_frequency = 1.0;

  Eigen::Vector3d imu_to_odom_leverarm = Eigen::Vector3d::Zero();
  Eigen::Vector2d scaler = Eigen::Vector2d::Ones();
};

// Estimator
class OdomEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdomEstimatorBase(const OdomEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options);
  ~OdomEstimatorBase();

  // Add Odometer meausrement
  virtual void addOdomMeasurement(const OdomMeasurement& odom_measurement);


  // Get lastest Odometer measurement timestamp
  double latestOdomMeasurementTimestamp() 
  { return odom_measurements_.size() == 0 ? 0.0 : odom_measurements_.back().timestamp; }


  // Get current GNSS rover measurement
  inline OdomMeasurement& curOdom() { 
    return getCurrent(odom_measurements_); 
  }

  // (TODO:--sbs) Get last Odom measurement (the end of last pre-integration window)
  // inline OdomMeasurement& lastOdom() { 
  //   // int pass_cnt = 0;
  //   // for (auto it = states_.rbegin(); it != states_.rend(); it++) {
  //   //   State& state = *it;
  //   //   if (!state.valid()) continue;
  //   //   if (state.id.type() == IdType::oScaler ) {
  //   //     pass_cnt++;
  //   //     if (pass_cnt == 2) return state;
  //   //   }
  //   // }
  //   // return null_state_;

  //   return getLast(odom_measurements_); 
  // }

  // Get last GNSS rover measurement
  // inline OdomMeasurement& oldestOdom() { 
  //   return getOldest(odom_measurements_); 
  // }


  // Get last Odom state 
  inline State& lastOdomState() {
    int pass_cnt = 0;
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if ( state.id.type() == IdType::oPose || state.id.type() == IdType::oScaler ) {
        pass_cnt++;
        if (pass_cnt == 2) return state;
      }
    }
    return null_state_;
  }


protected:
  
  void addScalerParameterBlock(const int32_t id, const int odometer_flag);

  // Add Odometer pre-integration block to graph
  void addOdometerResidualBlock(const State& last_state, State& cur_state);


  // Erase a state inside or at the ends of state window
  // The pose block and speed and bias block will be erased here, note that we do not extrinsics
  // blocks here, it should be erased by other sensor bases.
  void eraseScalerState(const State& state, bool reform = true);



protected:
  // Options
  OdomEstimatorBaseOptions odom_base_options_;

  // Measurements
  OdomMeasurements odom_measurements_;
  bool do_not_remove_odom_measurements_ = false;
  std::mutex odom_mutex_, odom_state_mutex_;


  double last_odom_update_timestamp_ =0.0;
  int odom_state_cnt_ = 0;
  // // Flags
  // bool gravity_setted_ = false;

  // // For getXXXAt functions
  // double last_timestamp_;
  // State last_base_state_;
  // Transformation last_T_WS_;
  // OdomScaler last_scaler_;
  // Eigen::Matrix<double, 12, 12> last_covariance_;
  // bool need_covariance_ = false;
};

}