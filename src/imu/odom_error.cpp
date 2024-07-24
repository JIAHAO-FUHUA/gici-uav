/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *    Modified: Zurich Eye
 *    Modified: Cheng Chi
 *********************************************************************************/

/**
 * @file OdomError.cpp
 * @brief Source file for the OdomError class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "gici/imu/odom_error.h"

#include <math.h>

#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/transform.h"

namespace gici {

// Construct with measurements and parameters.
OdomError::OdomError(const OdomMeasurements &odom_measurements,
                   const OdomParameters& odom_parameters,
                   const double& t_0, const double& t_1)
{
  std::lock_guard<std::mutex> lock(preintegration_mutex_);
  setT0(t_0 - odom_parameters.delay_odom_cam);
  setT1(t_1 - odom_parameters.delay_odom_cam);
  setOdomParameters(odom_parameters);
  setOdomMeasurements(odom_measurements);

  double dt0 = t0_ - odom_measurements_.front().timestamp;
  double dt1 = t1_ - odom_measurements_.back().timestamp;
  if (dt0 < 0.0) {
    LOG(ERROR) << "First ODOM measurement included in OdomError is not old enough: "
      << std::fixed << t0_ << " vs " << odom_measurements_.front().timestamp;
    odom_measurements_.push_front(odom_measurements_.front());
    odom_measurements_.front().timestamp = t0_;
  }
  if (dt1 > 0.0) {
    LOG(ERROR) << "Last ODOM measurement included in OdomError is not new enough: "
      << std::fixed << t1_ << " vs " << odom_measurements_.back().timestamp;
    odom_measurements_.push_back(odom_measurements_.back());
    odom_measurements_.back().timestamp = t1_;
  }
}

// Propagates pose, speeds and biases with given ODOM measurements.
int OdomError::redoPreintegration(const Transformation& /*T_WS*/
                                //  const SpeedAndBias& speed_and_biases,
                                //  const Scaler& scaler
                                 ) const
{
  // ensure unique access
  std::lock_guard<std::mutex> lock(preintegration_mutex_);

  // now the propagation
  double time = t0_;

  // sanity check:
  CHECK_LE(odom_measurements_.front().timestamp, time);
  CHECK_GE(odom_measurements_.back().timestamp, t1_);

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  // C_doubleintegral_ = Eigen::Matrix3d::Zero();
  vel_integral_ = Eigen::Vector3d::Zero();

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 6, 6>::Zero();

  double delta_t = 0;
  bool has_started = false;
  bool last_iteration = false;
  int n_integrated = 0;
  for (size_t i = 0; i < odom_measurements_.size() - 1; i++)
  {
    if (!has_started && 
        !(odom_measurements_[i].timestamp <= t0_ && 
        odom_measurements_[i + 1].timestamp >= t0_)) continue;

    Eigen::Vector3d omega_S_0 = odom_measurements_[i].bearing_angular_velocity;
    Eigen::Vector3d vel_S_0 = odom_measurements_[i].forward_velocity;
    Eigen::Vector3d omega_S_1 = odom_measurements_[i + 1].bearing_angular_velocity;
    Eigen::Vector3d vel_S_1 = odom_measurements_[i + 1].forward_velocity;
    double nexttime = odom_measurements_[i + 1].timestamp;

    // time delta
    double dt = nexttime - time;

    if (t1_ < nexttime)
    {
      double interval = nexttime - odom_measurements_[i].timestamp;
      nexttime = t1_;
      last_iteration = true;
      dt = nexttime - time;
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      vel_S_1 = ((1.0 - r) * vel_S_0 + r * vel_S_1).eval();
    }

    if (dt <= 0.0)
    {
      continue;
    }
    delta_t += dt;

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - odom_measurements_[i].timestamp);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      vel_S_0 = (r * vel_S_0 + (1.0 - r) * vel_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = odom_parameters_.sigma_g_c;
    double sigma_v_c = odom_parameters_.sigma_v_c;
    Eigen::Matrix<double,2,1> scaler = odom_parameters_.scaler;  //--sbs test
    Eigen::Vector3d l_S_odom = odom_parameters_.l_S_Odom; // Eigen::Vector3d(-0.4805,0,-0.0726);  //--sbs test 20240424
    if (std::abs(omega_S_0[0]) > odom_parameters_.g_max
        || std::abs(omega_S_0[1]) > odom_parameters_.g_max
        || std::abs(omega_S_0[2]) > odom_parameters_.g_max
        || std::abs(omega_S_1[0]) > odom_parameters_.g_max
        || std::abs(omega_S_1[1]) > odom_parameters_.g_max
        || std::abs(omega_S_1[2]) > odom_parameters_.g_max)
    {
      sigma_g_c *= 100;
      LOG(WARNING)<< "bearing angular velocity saturation: " 
                  << omega_S_0.norm() << ", " << omega_S_1.norm();
    }

    if (std::abs(vel_S_0[0]) > odom_parameters_.v_max
        || std::abs(vel_S_0[1]) > odom_parameters_.v_max
        || std::abs(vel_S_0[2]) > odom_parameters_.v_max
        || std::abs(vel_S_1[0]) > odom_parameters_.v_max
        || std::abs(vel_S_1[1]) > odom_parameters_.v_max
        || std::abs(vel_S_1[2]) > odom_parameters_.v_max)
    {
      sigma_v_c *= 100;
      LOG(WARNING) << "forward velocity saturation: " 
                   << vel_S_0.norm() << ", " << vel_S_1.norm();
    }

    // actual propagation
    // rotation integral
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true =
        (0.5 * (omega_S_0 + omega_S_1) * ( scaler[0]));
    const Eigen::Vector3d omega_S_meas = 
        0.5 * (omega_S_0 + omega_S_1) ;
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
    // velocity integral
    const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
    const Eigen::Vector3d vel_S_true =
        (0.5 * (vel_S_0 + vel_S_1) * ( scaler[1]));
    const Eigen::Vector3d vel_S_meas =
        0.5 * (vel_S_0 + vel_S_1) ;        
    const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
    const Eigen::Vector3d vel_integral_1 =
        vel_integral_ + 0.5 * (C + C_1) * (vel_S_true * dt - omega_S_true.cross(l_S_odom)* dt   );   //add lever-arm from imu to odom --sbs

    // covariance propagation
    Eigen::Matrix<double, 6, 6> F_delta =
        Eigen::Matrix<double, 6, 6>::Identity();
    // transform  ??? why not add it??? --sbs
    F_delta.block<3, 3>(0, 3) = -skewSymmetric(0.5 * (C + C_1) * (vel_S_true - omega_S_true.cross(l_S_odom)) * dt); //
    F_delta.block<3, 3>(3, 3) = F_delta.block<3, 3>(3, 3) -skewSymmetric(omega_S_true * dt);

    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // add noise. Note that transformations with rotation matrices can be
    // ignored, since the noise is isotropic.
    const double sigma2_p =  dt * sigma_v_c * sigma_v_c;
    P_delta_(0, 0) += sigma2_p;
    P_delta_(1, 1) += sigma2_p;
    P_delta_(2, 2) += sigma2_p;  

    const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
    P_delta_(3, 3) += sigma2_dalpha;
    P_delta_(4, 4) += sigma2_dalpha;
    P_delta_(5, 5) += sigma2_dalpha;

    // memory shift
    Delta_q_ = Delta_q_1;
    C_integral_ = C_integral_1;
    vel_integral_ = vel_integral_1;
    time = nexttime;

    ++n_integrated;

    if (last_iteration)
      break;

  }
  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();

  return n_integrated;
}

// This evaluates the error term and additionally computes the Jacobians.
bool OdomError::Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool OdomError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const
{
  // get poses
  const Transformation T_WS_0(
        Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3],
      parameters[0][4], parameters[0][5]));

  // const Transformation T_WS_1(
  //       Eigen::Vector3d(parameters[2][0], parameters[2][1], parameters[2][2]),
  //     Eigen::Quaterniond(parameters[2][6], parameters[2][3],
  //     parameters[2][4], parameters[2][5]));


  const Transformation T_WS_1(
        Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2]),
      Eigen::Quaterniond(parameters[1][6], parameters[1][3],
      parameters[1][4], parameters[1][5]));

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.getRotationMatrix();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double delta_t = t1_ - t0_;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
  }
  if (redo_)
  {
    redoPreintegration(T_WS_0);
    redoCounter_++;
    redo_ = false;
    /*if (redoCounter_ > 1) {
      std::cout << "pre-integration no. " << redoCounter_ << std::endl;
    }*/
  }

  // actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
    // this is a bit stupid, but shared read-locks only come in C++14

    // assign Jacobian w.r.t. x0
    Eigen::Matrix<double,6,6> F0 = Eigen::Matrix<double,6,6>::Zero(); 
    const Eigen::Vector3d delta_p_est_W =
        T_WS_0.getPosition() - T_WS_1.getPosition();
    const Eigen::Quaterniond Dq = Delta_q_;
    F0.block<3,3>(0,0) = C_S0_W;
    F0.block<3,3>(0,3) = C_S0_W * skewSymmetric(delta_p_est_W);
    
    F0.block<3,3>(3,3) =
        (quaternionPlusMatrix(Dq * T_WS_1.getEigenQuaternion().inverse()) *
         quaternionOplusMatrix(T_WS_0.getEigenQuaternion())).topLeftCorner<3,3>();

    // assign Jacobian w.r.t. x1
    Eigen::Matrix<double,6,6> F1 = Eigen::Matrix<double,6,6>::Zero(); 
    F1.block<3,3>(0,0) = -C_S0_W;
    F1.block<3,3>(3,3) =
        -(quaternionPlusMatrix(Dq) *
          quaternionOplusMatrix(T_WS_0.getEigenQuaternion()) *
          quaternionPlusMatrix(T_WS_1.getEigenQuaternion().inverse()))
        .topLeftCorner<3,3>();

    // the overall error vector
    Eigen::Matrix<double, 6, 1> error;
    error.segment<3>(0) =
        C_S0_W * delta_p_est_W + vel_integral_ ;
    error.segment<3>(3) =
        2.0 * (Dq * (T_WS_1.getEigenQuaternion().inverse() *
                     T_WS_0.getEigenQuaternion())).vec();

    // error weighting
    Eigen::Map<Eigen::Matrix<double, 6, 1> > weighted_error(residuals);
    weighted_error = (1.0 / down_weight_factor_) * square_root_information_ * error;

    // get the Jacobians
    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 6, 6> J0_minimal =
            (1.0 / down_weight_factor_) * square_root_information_ * F0.block<6, 6>(0, 0);

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > J0(
              jacobians[0]);
        J0 = J0_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[0] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >
                J0_minimal_mapped(jacobians_minimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      if (jacobians[1] != nullptr)
      {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 6, 6> J2_minimal = 
            (1.0 / down_weight_factor_) * square_root_information_
            * F1.block<6, 6>(0, 0);

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[1], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > J2(
              jacobians[1]);
        J2 = J2_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >
                J2_minimal_mapped(jacobians_minimal[1]);
            J2_minimal_mapped = J2_minimal;
          }
        }
      }

    }
  }
  return true;
}

}  // namespace gici
