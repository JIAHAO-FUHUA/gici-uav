/**
* @Function: Base class for Odometer estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/imu/odom_estimator_base.h"

#include "gici/estimate/speed_and_bias_parameter_block.h"
#include "gici/imu/odom_error.h"
#include "gici/imu/speed_and_bias_error.h"
#include "gici/estimate/pose_error.h"

#include "gici/utility/transform.h"

#include "gici/estimate/odom_parameter_block.h"  //--sbs

namespace gici {

// The default constructor
OdomEstimatorBase::OdomEstimatorBase(
                    const OdomEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options) :
  odom_base_options_(options), EstimatorBase(base_options)
{
  // Eigen::Quaterniond q_BI = 
  //   eulerAngleToQuaternion(odom_base_options_.body_to_odom_rotation * D2R);
  // T_BI_ = Transformation(Eigen::Vector3d::Zero(), q_BI);
}

// The default destructor
OdomEstimatorBase::~OdomEstimatorBase()
{}

// Add Odometer meausrement
void OdomEstimatorBase::addOdomMeasurement(const OdomMeasurement& odom_measurement)
{
  if (odom_measurements_.size() != 0 && 
      odom_measurements_.back().timestamp > odom_measurement.timestamp) {
    LOG(WARNING) << "Received Odometer with previous timestamp!";
  }
  else {
    odom_mutex_.lock();
    odom_measurements_.push_back(odom_measurement);
    odom_mutex_.unlock();
  }

  // delete used Odometer measurement
  odom_mutex_.lock();
  if (states_.size() > 0 && !do_not_remove_odom_measurements_)
  while (odom_measurements_.front().timestamp < oldestState().timestamp - 1.0) {
    odom_measurements_.pop_front();
  }
  odom_mutex_.unlock();
}



// Add Odometer speed and bias block to graph
void OdomEstimatorBase::addScalerParameterBlock(const int32_t id, const int odometer_flag)
{
  BackendId scaler_id = createOdometerScalerId(id);
  // BackendId scaler_id = id;

  Eigen::Matrix<double, 2, 1> scaler_init;
  scaler_init.setZero();
  std::shared_ptr<ScalerParameterBlock> odom_parameter_block = 
    std::make_shared<ScalerParameterBlock>(scaler_init, scaler_id.asInteger());
  CHECK(graph_->addParameterBlock(odom_parameter_block));

}

//Add Odometer residual block to graph --sbs
void OdomEstimatorBase::addOdometerResidualBlock(
  const State& last_state, State& cur_state)
{
  // double std_forward_velocity = 
  //       imu_base_options_.forward_velocity_std;

  // BackendId pose_id = state.id_in_graph;
  // BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  // BackendId scaler_id = changeIdType(pose_id, IdType::oScaler);

  // double information = 1.0 / square(std_forward_velocity);

  // // Eigen::Vector3d lever_arm = Eigen::Vector3d(0.0, 0.0, 0.0);
  // std::shared_ptr<OdometerError> odometer_error = 
  //   std::make_shared<OdometerError>(wheel_speed, information);

  // ceres::ResidualBlockId residual_id = 
  //   graph_->addResidualBlock(odometer_error, nullptr,
  //     graph_->parameterBlockPtr(pose_id.asInteger()),
  //     graph_->parameterBlockPtr(speed_and_bias_id.asInteger()),
  //     // graph_->parameterBlockPtr(odometer_extrinsics_id_.asInteger()), 
  //     graph_->parameterBlockPtr(scaler_id.asInteger()));



  const double last_timestamp = last_state.timestamp;
  const double timestamp = cur_state.timestamp;
  BackendId last_pose_id = last_state.id_in_graph;
  BackendId cur_pose_id = cur_state.id_in_graph;
  // BackendId last_speed_and_bias_id = changeIdType(last_pose_id, IdType::ImuStates);
  // BackendId speed_and_bias_id = changeIdType(cur_pose_id, IdType::ImuStates);
  // BackendId last_scaler_id = changeIdType(last_pose_id, IdType::oScaler);
  // BackendId scaler_id = changeIdType(cur_pose_id, IdType::oScaler);

  odom_mutex_.lock();
  std::shared_ptr<OdomError> odom_error =
    std::make_shared<OdomError>(odom_measurements_, odom_base_options_.odom_parameters,
                              last_timestamp, timestamp);
  odom_mutex_.unlock();
  // cur_state.imu_residual_to_lhs = 
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(odom_error, nullptr, 
      graph_->parameterBlockPtr(last_pose_id.asInteger()), 
      // graph_->parameterBlockPtr(last_scaler_id.asInteger()), 
      graph_->parameterBlockPtr(cur_pose_id.asInteger())
      // graph_->parameterBlockPtr(scaler_id.asInteger())
      );
    
  // // overlapped state
  State::syncOverlap(cur_state, states_);
  State::syncOverlap(last_state, states_);

}


// (TODO: --sbs) test this:  Erase a state inside or at the ends of windows  --sbs
void OdomEstimatorBase::eraseScalerState(const State& state, bool reform)
{
  const BackendId& parameter_id = state.id;
  BackendId scaler_id = changeIdType(parameter_id, IdType::oScaler);
  if (graph_->parameterBlockExists(scaler_id.asInteger())) {
    graph_->removeParameterBlock(scaler_id.asInteger());
  }

  // Reform connection if needed
  int index_lhs = -1, index_rhs = -1;
  for (size_t i = 0; i < states_.size(); i++) {
    if (!states_[i].valid()) continue;
    if (states_[i].id.type() != IdType::gPosition && 
        states_[i].id.type() != IdType::gPose) continue;
    if (!checkLargerEqual(states_[i].timestamp, state.timestamp)) {
      index_lhs = i;
    }
    if (!checkLessEqual(states_[i].timestamp, state.timestamp)) {
      index_rhs = i; break;
    }
  }
  // if (reform && index_lhs != -1 && index_rhs != -1) {
  //   addRelativeScalerResidualBlock(states_[index_lhs], states_[index_rhs]);
  // }
}





}