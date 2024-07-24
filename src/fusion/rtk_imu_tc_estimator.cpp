/**
* @Function: RTK/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"
#include <chrono>  //--sbs
namespace gici {

// The default constructor
RtkImuTcEstimator::RtkImuTcEstimator(
               const RtkImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const RtkEstimatorOptions rtk_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const OdomEstimatorBaseOptions& odom_base_options, //--sbs
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  tc_options_(options), rtk_options_(rtk_options),
  GnssEstimatorBase(gnss_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  OdomEstimatorBase(odom_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::RtkImuTc;
  is_use_phase_ = true;
  shiftMemory();

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));

  // Initialization control
  initializer_sub_estimator_.reset(new RtkEstimator(
    rtk_options, gnss_base_options, base_options, ambiguity_options));
  initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_, initializer_sub_estimator_));
}

// The default destructor
RtkImuTcEstimator::~RtkImuTcEstimator()
{}

// Add measurement
bool RtkImuTcEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{



  // Initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!initializer_->finished()) {
    if (initializer_->getCoordinate() == nullptr) {
      initializer_->setCoordinate(coordinate_);
      initializer_sub_estimator_->setCoordinate(coordinate_);
      initializer_->setGravity(imu_base_options_.imu_parameters.g);
    }
    if (initializer_->addMeasurement(measurement)) {
      initializer_->estimate();
      // set result to estimator
      setInitializationResult(initializer_);
    }
    return false;
  }


	tic_start = std::chrono::system_clock::now();

   // Add Odom  --sbs
  if(odom_base_options_.use_forward_velocity && measurement.odometer) {
    addOdomMeasurement(*measurement.odometer); 
    return addOdomMeasurementAndState();
  } 

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS
  if (measurement.gnss) {
    GnssMeasurement rov, ref;
    meausrement_align_.add(measurement);
    if (meausrement_align_.get(rtk_options_.max_age, rov, ref)) {
      gnss_common::deleteSatelliteToSize(rov, gnss_base_options_.max_satellite_size);
      return addGnssMeasurementAndState(rov, ref);
    }
  }


  return false;
}


// // Add Odometer measurement --sbs
// bool RtkImuTcEstimator::addOdomMeasurement(
//     const OdomMeasurement& measurement)
// {
//   if(odometer_flag != 2)
//     return false;
//   // if (fabs(measurement.bearing_angular_velocity) > 0.1) {
//   //   LOG(INFO)<< "Odometer angular velocity is too large, ignore this measurement.";
//   //   return false;
//   // }
//   addOdometerResidualBlock(states_[latest_state_index_], measurement.forward_velocity);

// }

// Add GNSS measurements and state
bool RtkImuTcEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref)
{
  // Get prior states
  Eigen::Vector3d position_prior = coordinate_->convert(
    getPoseEstimate().getPosition(), GeoType::ENU, GeoType::ECEF);

  // Set to local measurement handle
  curGnssRov() = measurement_rov;
  curGnssRov().position = position_prior;
  curGnssRef() = measurement_ref;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnssRov());
  gnss_common::rearrangePhasesAndCodes(curGnssRef());



  // Form double difference pair
  std::map<char, std::string> system_to_base_prn;
  GnssMeasurementDDIndexPairs phase_index_pairs = gnss_common::formPhaserangeDDPair(
    curGnssRov(), curGnssRef(), system_to_base_prn, gnss_base_options_.common);
  GnssMeasurementDDIndexPairs code_index_pairs = gnss_common::formPseudorangeDDPair(
    curGnssRov(), curGnssRef(), system_to_base_prn, gnss_base_options_.common);

  // Cycle-slip detection
  if (!isFirstEpoch()) {
    cycleSlipDetectionSD(lastGnssRov(), lastGnssRef(), 
      curGnssRov(), curGnssRef(), gnss_base_options_.common);
  }

  // Add parameter blocks
  double timestamp = curGnssRov().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnssRov().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  // CHECK(index == states_.size() - 1);
  states_[index].status = GnssSolutionStatus::Single;
  latest_state_index_ = index;  //--sbs
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());
  // ambiguity blocks
  addSdAmbiguityParameterBlocks(curGnssRov(), 
    curGnssRef(), phase_index_pairs, curGnssRov().id, curAmbiguityState());
  // frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnssRov(), curGnssRov().id, num_valid_doppler_system);


  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addDdPseudorangeResidualBlocks(curGnssRov(), 
    curGnssRef(), code_index_pairs, states_[index], num_valid_satellite);
  
  // We do not need to check if the number of satellites is sufficient in tightly fusion.
  if (!checkSufficientSatellite(num_valid_satellite, 0)) {
    // do nothing
  }

  // test --sbs 20240305
  // if (num_valid_satellite < 12) {
  //   gnss_base_options_.error_parameter.code_to_phase_ratio = 100000.0;
  //   LOG(INFO) << "The number of valid satellites is less than 12, set code to phase ratio to 100000.0";
  // }
  // else{
  //   gnss_base_options_.error_parameter.code_to_phase_ratio = 100.0;

  // }

  num_satellites_ = num_valid_satellite;

  // No satellite
  if (num_satellites_ == 0) {
    // erase parameters in current state
    eraseFrequencyParameterBlocks(states_[index]);
    eraseImuState(states_[index]);
    eraseAmbiguityParameterBlocks(curAmbiguityState());


    return false;
  }

  // Add phaserange residual blocks
  addDdPhaserangeResidualBlocks(
    curGnssRov(), curGnssRef(), phase_index_pairs, states_[index]);

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnssRov(), states_[index], num_valid_satellite, 
    false, getImuMeasurementNear(timestamp).angular_velocity);

  // Add relative errors
  if (!isFirstEpoch()) {
    // frequency

    //--sbs
    if (lastGnssState().valid()) {  // maybe invalid here because of long term GNSS absent
      // frequency
      addRelativeFrequencyResidualBlock(lastGnssState(), states_[index]);
    }

    // addRelativeFrequencyResidualBlock(lastState(), states_[index]);
    // ambiguity
    addRelativeAmbiguityResidualBlock(
      lastGnssRov(), curGnssRov(), lastAmbiguityState(), curAmbiguityState());

  }

  // ZUPT
  addZUPTResidualBlock(states_[index]);

  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(states_[index]);
    // non-holonomic constraint
    addNHCResidualBlock(states_[index]);
  }

  // Compute DOP
  updateGdop(curGnssRov(), code_index_pairs);
  updatePdop(curGnssRov(), code_index_pairs);  //--sbs

  return true;
}



// Add image measurements and state
bool RtkImuTcEstimator::addOdomMeasurementAndState()
{
  // If initialized, we are supposed not at the first epoch
  if(states_.size() < 3) return false;
  if(last_odom_update_timestamp_ == 0.0){

    odom_state_cnt_++;
    const int32_t odom_id = odom_state_cnt_;//*85692234+462525574 ;//(static_cast<int32_t>((odom_state_cnt_  << 16) & 0xFFFFFFFF));

    BackendId pose_id = createOdomPoseId(odom_id);

    // Add new IMU state
    size_t index;
    double timestamp = curOdom().timestamp;
    index = insertImuState(timestamp, pose_id);

    states_[index].status = latestGnssState().status;
    latest_state_index_ = index;

    // Add Odometer State Block --sbs
    // addScalerParameterBlock( odom_id, odometer_flag);
    last_odom_update_timestamp_ = curOdom().timestamp;

    return false;
  }
  else{

    // Check frequency
    if ( odom_measurements_.size() > 1) {
      const double t_last = last_odom_update_timestamp_;
      const double t_cur = curOdom().timestamp;
      if ((t_cur - t_last) < 1.0 / odom_base_options_.estimate_frequency) 
      {
        return false;
      }


    }
    else{
      return false;
    }


    odom_state_cnt_++;
    const int32_t odom_id = odom_state_cnt_;//*85692234+462525574 ;//(static_cast<int32_t>((odom_state_cnt_  << 16) & 0xFFFFFFFF));
    BackendId pose_id = createOdomPoseId(odom_id);

    // Add new IMU state
    size_t index;
    double timestamp = curOdom().timestamp;
    index = insertImuState(timestamp, pose_id);

    states_[index].status = latestGnssState().status;
    latest_state_index_ = index;

    // Add Odometer State Block --sbs
    // addScalerParameterBlock( odom_id, odometer_flag);
    // check if curent state is valid for odometer update --sbs
    // if(!checkHmcValid(states_[index]))
    // {
    //   return false;
    // }

    if(lastOdomState().timestamp == 0.0){
        return false;
    }

    // Add Odometer Pre-integration Residual Block --sbs
    if(odom_state_cnt_ > 1)
    {

      addOdometerResidualBlock(lastOdomState(), states_[index]);
    }

    last_odom_update_timestamp_ = timestamp;

    return true;
  }
}


// Solve current graph
bool RtkImuTcEstimator::estimate()
{

  optimize();

    // Get current sensor type
  IdType new_state_type = states_[latest_state_index_].id.type();

  // GNSS processes
  double gdop = 0.0;
  if (new_state_type == IdType::gPose)
  {
    // Optimize with FDE
    size_t n_pseudorange = numPseudorangeError(states_[latest_state_index_]);
    size_t n_phaserange = numPhaserangeError(states_[latest_state_index_]);
    size_t n_doppler = numDopplerError(states_[latest_state_index_]);
    if (gnss_base_options_.use_outlier_rejection)
    {
      rejectPseudorangeOutlier(states_[latest_state_index_], curAmbiguityState());
      rejectDopplerOutlier(states_[latest_state_index_]);
      rejectPhaserangeOutlier(states_[latest_state_index_], curAmbiguityState());
    }
    // while (1)
    // {
    //   optimize();

    //   // reject outlier
    //   if (!rejectPseudorangeOutlier(states_[latest_state_index_], curAmbiguityState(),
    //       gnss_base_options_.reject_one_outlier_once) && 
    //       !rejectDopplerOutlier(states_[latest_state_index_], 
    //       gnss_base_options_.reject_one_outlier_once) &&
    //       !rejectPhaserangeOutlier(states_[latest_state_index_], curAmbiguityState(),
    //       gnss_base_options_.reject_one_outlier_once)) break;
    //   if (!gnss_base_options_.reject_one_outlier_once) break;  
    // }
    // // Optimize without FDE
    // else {
    //   optimize();
    // }

    // Check if we rejected too many GNSS residuals
    double ratio_pseudorange = n_pseudorange == 0.0 ? 0.0 : 1.0 - 
      getDivide(numPseudorangeError(states_[latest_state_index_]), n_pseudorange);
    double ratio_phaserange = n_phaserange == 0.0 ? 0.0 : 1.0 - 
      getDivide(numPhaserangeError(states_[latest_state_index_]), n_phaserange);
    double ratio_doppler = n_doppler == 0.0 ? 0.0 : 1.0 - 
      getDivide(numDopplerError(states_[latest_state_index_]), n_doppler);
    const double thr = gnss_base_options_.diverge_max_reject_ratio;
    if (isGnssGoodObservation() && 
        (ratio_pseudorange > thr || ratio_phaserange > thr || ratio_doppler > thr)) {
      num_cotinuous_reject_gnss_++;
    }
    else num_cotinuous_reject_gnss_ = 0;
    if (num_cotinuous_reject_gnss_ > 
        gnss_base_options_.diverge_min_num_continuous_reject) {
      LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
      status_ = EstimatorStatus::Diverged;
      num_cotinuous_reject_gnss_ = 0;
    }

    // Ambiguity resolution
    states_[latest_state_index_].status = GnssSolutionStatus::Float;
    if (rtk_options_.use_ambiguity_resolution) {
      // get covariance of ambiguities
      Eigen::MatrixXd ambiguity_covariance;
      std::vector<uint64_t> parameter_block_ids;
      for (auto id : curAmbiguityState().ids) {
        parameter_block_ids.push_back(id.asInteger());
      }
      bool covariance_result = graph_->computeCovariance(parameter_block_ids, ambiguity_covariance);

      if(covariance_result)
      { 
        // LOG(INFO)<<"ambiguity_covariance: "<<ambiguity_covariance<<std::endl; //--sbs
        // if(ambiguity_covariance.cols() < 10)
        // {
        //   gnss_base_options_.error_parameter.code_to_phase_ratio = 100000.0;
        //   LOG(INFO) << "The number of valid ambiguities is less than 10, set code to phase ratio to 100000.0";
        // }
        // else{
        //   gnss_base_options_.error_parameter.code_to_phase_ratio = 100.0;
        // }


        // solve
        AmbiguityResolution::Result ret = ambiguity_resolution_->solveRtk(
          states_[latest_state_index_].id, curAmbiguityState().ids, 
          ambiguity_covariance, gnss_measurement_pairs_.back());
        if (ret == AmbiguityResolution::Result::NlFix) {
          states_[latest_state_index_].status = GnssSolutionStatus::Fixed;
        }
      }
    }

    // Check if we continuously cannot fix ambiguity, while we have good observations
    if (rtk_options_.use_ambiguity_resolution) {
      const double thr = gnss_base_options_.good_observation_max_reject_ratio;
      if (isGnssGoodObservation() && ratio_pseudorange < thr && 
          ratio_phaserange < thr && ratio_doppler < thr) {
        if (states_[latest_state_index_].status != GnssSolutionStatus::Fixed) num_continuous_unfix_++;
        else num_continuous_unfix_ = 0; 
      }
      else num_continuous_unfix_ = 0;
      if (num_continuous_unfix_ > 
          gnss_base_options_.reset_ambiguity_min_num_continuous_unfix) {
        LOG(INFO) << "Continuously unfix under good observations. Clear current ambiguities.";
        resetAmbiguityEstimation();
        num_continuous_unfix_ = 0;
      }
    }

  }


  if (new_state_type == IdType::oPose) {
    // // update landmarks to frontend
    // updateLandmarks();
    // // update states to frontend
    // updateFrameStateToFrontend(states_[latest_state_index_], curFrame());
    // // reject landmark outliers
    // size_t n_reprojection = numReprojectionError(curFrame());
    // rejectReprojectionErrorOutlier(curFrame());
    // // check if we rejected too many reprojection errors
    // double ratio_reprojection = n_reprojection == 0.0 ? 0.0 : 1.0 - 
    //   getDivide(numReprojectionError(curFrame()), n_reprojection);
    // if (ratio_reprojection > visual_base_options_.diverge_max_reject_ratio) {
    //   num_cotinuous_reject_visual_++;
    // }
    // else num_cotinuous_reject_visual_ = 0;
    // if (num_cotinuous_reject_visual_ > 
    //     visual_base_options_.diverge_min_num_continuous_reject) {
    //   LOG(WARNING) << "Estimator diverge: Too many visual outliers rejected!";
    //   status_ = EstimatorStatus::Diverged;
    //   num_cotinuous_reject_visual_ = 0;
    // }
  
  }
  


  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sensor type: " << std::setw(1) <<
      static_cast<int>(BackendId::sensorType(states_[latest_state_index_].id.type()))
      // << ", Time cost: " << std::setw(2) <<  time_cost
      << " s, Sat number: " << std::setw(2) << num_satellites_
      << ", Fix status: " << std::setw(1) << static_cast<int>(states_[latest_state_index_].status);
  }

  double cur_time = states_[latest_state_index_].timestamp;

  // Apply marginalization
  marginalization(new_state_type);

  tic_end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tic_end - tic_start);
	double time_cost = double(duration.count()) * std::chrono::microseconds::period::num 
                  / std::chrono::microseconds::period::den;  // s


  if (base_options_.verbose_output) {
    LOG(INFO)  
      << std::setprecision(13) 
      << "Timestamp: " << std::setw(18) <<  cur_time
      << ", Sensor type: " << std::setw(1) <<
      static_cast<int>(BackendId::sensorType(states_[latest_state_index_].id.type()))
      << std::setprecision(3)

      << ", Ceres Time cost: " << std::setw(2) <<  time_cost
      << " s"      
      ;

  }
  // Shift memory for states and measurements
  // shiftMemory();
  states_.push_back(State());
  if(new_state_type == IdType::gPose){
    ambiguity_states_.push_back(AmbiguityState());
    gnss_measurement_pairs_.push_back(
      std::make_pair(GnssMeasurement(), GnssMeasurement()));

  }    
  // if (states_.size() > tc_options_.max_window_length) {
  //     states_.pop_front();
  //     // if(new_state_type == IdType::gPose){
  //     //   ambiguity_states_.pop_front();
  //     //   gnss_measurement_pairs_.pop_front();
  //     // }
  //   }
  // IdType new_state_type = states_[latest_state_index_].id.type();

  // if (new_state_type == IdType::oScaler) odom_.push_back(nullptr);


  odometer_flag = 1;  //--sbs

  return true;
}

// Set initializatin result
void RtkImuTcEstimator::setInitializationResult(
  const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer = 
    std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);
  
  // Arrange to window length
  ImuMeasurements imu_measurements;
  std::deque<GnssSolution> gnss_solution_measurements;
  gnss_imu_initializer->arrangeToEstimator(
    tc_options_.max_window_length, marginalization_error_, states_, 
    marginalization_residual_id_, gnss_extrinsics_id_, 
    gnss_solution_measurements, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }
  gnss_measurement_pairs_.resize(gnss_solution_measurements.size());
  ambiguity_states_.resize(states_.size());

  // Shift memory for states and measurements
  shiftMemory();

  // Set flags
  can_compute_covariance_ = true;
}

// Marginalization
bool RtkImuTcEstimator::marginalization(const IdType& type)
{
  if (type == IdType::oPose) return odomMarginalization();
  else if (type == IdType::gPose) return gnssMarginalization();
  else return false;
}

bool RtkImuTcEstimator::odomMarginalization()
{
  // Check if we need marginalization
  // if (states_.size() < 3)
  if (isFirstEpoch()) return true;

  // Check if we need marginalization
  if (states_.size() < tc_options_.max_window_length) {
    return true;
  }

  // Marginalize the GNSS states that in front of the oldest keyframe
  // We do this at both here and the visual margin step to smooth computational load
  for (auto it = states_.begin(); it != states_.end(); it ++) {
    State& state = *it;
    // reached the oldest keyframe
    // if (state.is_keyframe) break;
    // check if GNSS state
    if (state.id.type() != IdType::oPose) continue;
    // Erase old marginalization item
    if (!eraseOldMarginalization()) return false;
    // margin

    addImuStateMarginBlock(state);
    addImuResidualMarginBlocks(state);


    // Erase state
    it = states_.erase(it);
    // just handle one in one time
    bool ret = applyMarginalization();
    return ret;
  }

  return true;
}

bool RtkImuTcEstimator::gnssMarginalization()
{

  // sparsifyGnssStates();

  // Check if we need marginalization
  if (states_.size() < tc_options_.max_window_length) {
    return true;
  }

    // Marginalize the GNSS states that in front of the oldest keyframe
    // We do this at both here and the visual margin step to smooth computational load
    for (auto it = states_.begin(); it != states_.end();it++) {
      State& state = *it;
      // reached the oldest keyframe
      if (state.is_keyframe) break;
      // check if GNSS state
      if (state.id.type() != IdType::gPose) continue;

      // Erase old marginalization item
      if (!eraseOldMarginalization()) return false;

      // margin
      auto it_ambiguity = ambiguityStateAt(state.timestamp);
      if (it_ambiguity != ambiguity_states_.end()) {
        addAmbiguityMarginBlocksWithResiduals(*it_ambiguity);
      }
      else {
        addGnssLooseResidualMarginBlocks(state);
      }
      // IMU states and residuals
      // addImuStateMarginBlockWithResiduals(state);
      // addImuStateMarginBlock(state);
      // addImuResidualMarginBlocks(state);
      // // ambiguity
      // addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
      // // frequency
      // addFrequencyMarginBlocksWithResiduals(state);

      // Apply marginalization and add the item into graph
      // return applyMarginalization();

      addImuStateMarginBlock(state);
      addImuResidualMarginBlocks(state);
      addFrequencyMarginBlocksWithResiduals(state);
      addGnssResidualMarginBlocks(state);

      // erase ambiguity state and GNSS measurements
      if (it_ambiguity != ambiguity_states_.end()) {
        ambiguity_states_.erase(it_ambiguity);
      }
      if (gnssMeasurementPairAt(state.timestamp) != gnss_measurement_pairs_.end()) {
        gnss_measurement_pairs_.erase(gnssMeasurementPairAt(state.timestamp));
      }

      // Erase state
      it = states_.erase(it);

      // just handle one in one time
      bool ret = applyMarginalization();

      return ret;
    }
  // Erase old marginalization item
  // if (!eraseOldMarginalization()) return false;

  // // Add marginalization items
  // // IMU states and residuals
  // addImuStateMarginBlockWithResiduals(oldestState());
  // // ambiguity
  // addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
  // // frequency
  // addFrequencyMarginBlocksWithResiduals(oldestState());

  // // Apply marginalization and add the item into graph
  // return applyMarginalization();
  // return true;
}

// // Sparsify GNSS states to bound computational load
// void RtkImuTcEstimator::sparsifyGnssStates()
// {
//   // Check if we need to sparsify
//   std::vector<BackendId> gnss_ids;
//   std::vector<int> num_neighbors;
//   int max_num_neighbors = 0;
//   for (int i = 0; i < states_.size(); i++) {
//     if (states_[i].id.type() != IdType::gPose) continue;
//     gnss_ids.push_back(states_[i].id);
//     // find neighbors
//     num_neighbors.push_back(0);
//     int idx = num_neighbors.size() - 1;
//     if (i - 1 >= 0) for (int j = i - 1; j >= 0; j--) {
//       if (states_[j].id.type() != IdType::gPose) break;
//       num_neighbors[idx]++;
//     } 
//     if (i + 1 < states_.size()) for (int j = i; j < states_.size(); j++) {
//       if (states_[j].id.type() != IdType::gPose) break;
//       num_neighbors[idx]++;
//     } 
//     if (max_num_neighbors < num_neighbors[idx]) {
//       max_num_neighbors = num_neighbors[idx];
//     }
//   }
//   if (gnss_ids.size() <= rrr_options_.max_keyframes) return;

//   // Erase some GNSS states
//   // The states with most neighbors will be erased first
//   int num_to_erase = gnss_ids.size() - rrr_options_.max_keyframes;
//   std::vector<BackendId> ids_to_erase;
//   for (int m = max_num_neighbors; m >= 0; m--) {
//     for (size_t i = 0; i < num_neighbors.size(); i++) {
//       if (num_neighbors[i] != m) continue;
//       if (i == 0) continue;  // in case the first one connects to margin block
//       ids_to_erase.push_back(gnss_ids[i]);
//       if (ids_to_erase.size() >= num_to_erase) break;
//     }
//     if (ids_to_erase.size() >= num_to_erase) break;
//   }
//   CHECK(ids_to_erase.size() >= num_to_erase);
//   for (int i = 0; i < states_.size(); i++) {
//     if (std::find(ids_to_erase.begin(), ids_to_erase.end(), states_[i].id) 
//         == ids_to_erase.end()) {
//       continue;
//     }
//     State& state = states_[i];

//     auto it_ambiguity = ambiguityStateAt(state.timestamp);
//     // the first one may be connected with margin error
//     if (it_ambiguity == ambiguity_states_.begin()) continue;

//     eraseGnssMeasurementResidualBlocks(state);
//     eraseFrequencyParameterBlocks(state);

//     // we may failed to find corresponding ambiguity state because the GNSS state can be 
//     // loosely coupled state during initialization.
//     if (it_ambiguity != ambiguity_states_.end()) {
//       eraseAmbiguityParameterBlocks(*it_ambiguity);
//       ambiguity_states_.erase(it_ambiguity);
//     }
//     else {
//       eraseGnssLooseResidualBlocks(state);
//     }

//     eraseImuState(state);

//     if (gnssMeasurementPairAt(state.timestamp) != gnss_measurement_pairs_.end()) {
//       gnss_measurement_pairs_.erase(gnssMeasurementPairAt(state.timestamp));
//     }

//     i--;
//   }
// }

};