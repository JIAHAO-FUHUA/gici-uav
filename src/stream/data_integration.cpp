/**
* @Function: Integrate formator-decoded data to estimator data
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/data_integration.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

// Base class
DataIntegrationBase::DataIntegrationBase(
  const std::shared_ptr<EstimatingBase>& estimating, 
  const std::vector<std::shared_ptr<Streaming>>& streamings,
  const std::vector<std::string>& formator_tags,
  const std::vector<std::vector<std::string>>& roles)
{
  // Bind this->dataCallback to streamings
  for (const auto& streaming : streamings) {
    Streaming::DataCallback callback = std::bind(
      &DataIntegrationBase::dataCallback, 
      this, std::placeholders::_1, std::placeholders::_2);
    streaming->setDataCallback(callback);
  }

  // Bind estimator->estimatorDataCallback to this->estimator_callbacks_
  EstimatorDataCallback estimator_callback = 
    std::bind(&EstimatingBase::estimatorDataCallback, 
      estimating.get(), std::placeholders::_1);
  estimator_callbacks_.push_back(estimator_callback);

  // Declare behaviors for stream inputs
  CHECK(formator_tags.size() == roles.size());
  for (size_t i = 0; i < formator_tags.size(); i++) {
    behaviors_.insert(std::make_pair(formator_tags[i], roles[i]));
  }

  // Post Process --sbs
  is_post_process_    = estimating->is_post_process_;
  use_precise_product_= estimating->use_precise_product_;
  use_base_position_cfile_ = estimating->use_base_position_cfile_;
  base_position_cfile_      = estimating->base_position_cfile_;
}

// GNSS data integration
GnssDataIntegration::GnssDataIntegration(
  const std::shared_ptr<EstimatingBase>& estimating, 
  const std::vector<std::shared_ptr<Streaming>>& streamings,
  const std::vector<std::string>& formator_tags,
  const std::vector<std::vector<std::string>>& roles) :
  DataIntegrationBase(estimating, streamings, formator_tags, roles)
{
  // Initialize data handles
  gnss_local_ = std::make_shared<DataCluster::GNSS>();
  gnss_local_->init();
  bool found_precise_ephemeris = false;
  // code bias handle
  for (auto behavior : behaviors_) {
    for (auto role : behavior.second) {
      if (role == "ssr_ephemeris") found_precise_ephemeris = true;
    }
  }
  // precise ephemeris base
  if (found_precise_ephemeris) {
    CodeBias::BaseFrequencies bases;
    bases.insert(std::make_pair('G', std::make_pair(CODE_L1W, CODE_L2W)));
    bases.insert(std::make_pair('R', std::make_pair(CODE_L1P, CODE_L2P)));
    bases.insert(std::make_pair('E', std::make_pair(CODE_L1C, CODE_L5Q)));
    bases.insert(std::make_pair('C', std::make_pair(CODE_L2I, CODE_L6I)));
    code_bias_local_ = std::make_shared<CodeBias>(bases);
  }
  // broadcast ephemeris base
  else {
    CodeBias::BaseFrequencies bases;
    bases.insert(std::make_pair('G', std::make_pair(CODE_L1W, CODE_L2W)));
    bases.insert(std::make_pair('R', std::make_pair(CODE_L1P, CODE_L2P)));
    bases.insert(std::make_pair('E', std::make_pair(CODE_L1C, CODE_L5Q)));
    bases.insert(std::make_pair('C', std::make_pair(CODE_L6I, CODE_NONE)));
    code_bias_local_ = std::make_shared<CodeBias>(bases);
  }
  // phase bias handle
  phase_bias_local_ = std::make_shared<PhaseBias>();

  // set valid
  valid_ = true;
}

GnssDataIntegration::~GnssDataIntegration()
{
  // Free handles
  mutex_.lock();
  gnss_local_->free();
  mutex_.unlock();
}

// Data callback
void GnssDataIntegration::dataCallback(
  const std::string& input_tag, const std::shared_ptr<DataCluster>& data)
{
  if (data->gnss && valid_) {
    mutex_.lock();
    handleGNSS(input_tag, data->gnss);
    mutex_.unlock();
  }
}


void GnssDataIntegration::storeLocalGnss(const std::shared_ptr<DataCluster::GNSS>& gnss){

  DataCluster::GNSS gnss_local;

  for (auto it : gnss->types){
    if (it == GnssDataType::Ephemeris) {
      gnss_local.init();   // (TODO) memory leak? --sbs
     
      memcpy(gnss_local.ephemeris->eph, 
      gnss->ephemeris->eph, sizeof(eph_t) * 20 * MAXSAT);
      memcpy(gnss_local.ephemeris->geph, 
      gnss->ephemeris->geph, sizeof(geph_t) * 20 * NSATGLO);

      gnss_local_ephemerises_.insert(std::make_pair( gnss_common::gpsTimeToUtcTime(
    gnss_common::gtimeToDouble(gnss->ephemeris->eph[0].toe)), gnss_local));
    }
    else if(it == GnssDataType::PreciseEph){
      gnss_local_->ephemeris->peph = gnss->ephemeris->peph;
      gnss_local_->ephemeris->ne = gnss->ephemeris->ne;
    }
    else if(it == GnssDataType::PreciseClk){
      gnss_local_->ephemeris->pclk = gnss->ephemeris->pclk;
      gnss_local_->ephemeris->nc = gnss->ephemeris->nc;
    }
    else if(it == GnssDataType::ObservationBias){
      gnss_local_->obs_bias = gnss->obs_bias;
    }
    else if(it == GnssDataType::IonNetwork){
      gnss_local_->ion_network = gnss->ion_network;
    }  
    else if(it == GnssDataType::ZtdNetwork){
      gnss_local_->ztd_network = gnss->ztd_network;
    }  

  }
}

int GnssDataIntegration::dumpLocalGnss(const double epoch){

  int ret = 0;
  // combine the nearest valid ephemeris/ bias/ atmosphere to gnss_local_ --sbs

  //// 1. broadcast ephemeris
  if(gnss_local_ephemerises_.size()<=0 || gnss_local_ephemerises_.rbegin()->first < (epoch-7200.0)
    ) return -1;
  auto it_gnss = gnss_local_ephemerises_.upper_bound(epoch - 7200.0);
  if(it_gnss != gnss_local_ephemerises_.end()){
    if(epoch - it_gnss->first > 7200)  // test PPP --sbs
        return -1;    
    LOG(INFO)<<"epoch - ephemrises->first = "<< epoch - it_gnss->first <<" s";
    // if(epoch - it_gnss->first > MAXSECONDS){
    memcpy(gnss_local_->ephemeris->eph, 
        it_gnss->second.ephemeris->eph, sizeof(eph_t) * 20 * MAXSAT);
    memcpy(gnss_local_->ephemeris->geph, 
        it_gnss->second.ephemeris->geph, sizeof(geph_t) * 20 * NSATGLO);
    updateTgd();

    ret = 1;
  }
  else{  ret = -1; }

  if(ret == 1){
    //// 2. precise ephemeris
    if(gnss_local_->ephemeris->ne > 0)  {
      ret = 2;

      //// 3. precise clock
      if(gnss_local_->ephemeris->nc > 0) {
        ret = 3;

        //// 4. observation bias
        if(updateObservationBias(epoch)) {
          ret = 4;

        }
      }
    }
  }


  //// 5. ionosphere
  it_gnss = gnss_local_ionNetworks_.lower_bound(epoch);
  if(it_gnss != gnss_local_ionNetworks_.end()){
    LOG(INFO)<<"epoch - ionosphere->first = "<< epoch - it_gnss->first <<" s";
    // if(epoch - it_gnss->first > MAXSECONDS){
    gnss_local_->ion_network = it_gnss->second.ion_network;
  }
  else{
    // LOG(WARNING)<<"no ionosphere product found";
  }

  //// 6. troposphere
  it_gnss = gnss_local_ztdNetworks_.lower_bound(epoch);
  if(it_gnss != gnss_local_ztdNetworks_.end()){
    LOG(INFO)<<"epoch - troposphere->first = "<< epoch - it_gnss->first <<" s";
    // if(epoch - it_gnss->first > MAXSECONDS){
    gnss_local_->ztd_network = it_gnss->second.ztd_network;
  }  
  else{
    // LOG(WARNING)<<"no troposphere product found";
  }

  return ret;
}

// Handle GNSS data
void GnssDataIntegration::handleGNSS(const std::string& formator_tag, 
                const std::shared_ptr<DataCluster::GNSS>& gnss)
{
  // Check if initialized
  if (gnss_local_ == nullptr || gnss_local_->ephemeris == NULL || 
      gnss_local_->observation == NULL || gnss_local_->antenna == NULL ) return;

  // Get role
  if (behaviors_.find(formator_tag) == behaviors_.end()) return;
  std::vector<GnssRole> roles;
  GnssRole role_out;
  for (size_t i = 0; i < behaviors_.at(formator_tag).size(); i++) {
    roles.push_back(GnssRole());
    option_tools::convert(behaviors_.at(formator_tag)[i], roles[i]);
    if (roles[i] == GnssRole::Rover || roles[i] == GnssRole::Reference ||
        roles[i] == GnssRole::Heading) role_out = roles[i]; 
  }



  // Update ephemeris and corrections
  for (auto it : gnss->types) {
    if (it == GnssDataType::Ephemeris) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Ephemeris) 
        { found = true; break; }
      }
      if (!found) continue;
      memcpy(gnss_local_->ephemeris->eph, 
        gnss->ephemeris->eph, sizeof(eph_t) * 2 * MAXSAT);
      memcpy(gnss_local_->ephemeris->geph, 
        gnss->ephemeris->geph, sizeof(geph_t) * 2 * NSATGLO);
      updateTgd();
    }

    if (it == GnssDataType::SSR) {
      bool found = false;
      std::vector<gnss_common::UpdateSsrType> update_types;
      for (auto it_role : roles) {
        if (it_role == GnssRole::SsrEphemeris) {
          update_types.push_back(gnss_common::UpdateSsrType::Ephemeris);
        }
        if (it_role == GnssRole::CodeBias) {
          update_types.push_back(gnss_common::UpdateSsrType::CodeBias);
        }
        if (it_role == GnssRole::PhaseBias) {
          update_types.push_back(gnss_common::UpdateSsrType::PhaseBias);
        }

      }
      gnss_common::updateSsr(gnss->ephemeris->ssr, gnss_local_, update_types); // update code bias to gnss_local_
      for (auto it_role : roles) {
        if (it_role == GnssRole::CodeBias) updateCodeBias();   // update code bias from gnss_local_ to code_bias_local_
        if (it_role == GnssRole::PhaseBias) updatePhaseBias();
      }
    }

    if (it == GnssDataType::AntePos) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Reference) 
        { found = true; break; }
      }
      if (!found) continue;
      gnss_common::updateAntennaPosition(gnss->antenna, gnss_local_);
    }

    if (it == GnssDataType::IonAndUtcPara) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::IonAndUtc) 
        { found = true; break; }
      }
      if (!found) continue;
      gnss_common::updateIonAndUTC(gnss->ephemeris, gnss_local_);
    }

    if (it == GnssDataType::PhaseCenter) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::PhaseCenter) 
        { found = true; break; }
      }
      if (!found) continue;
      memcpy(gnss_local_->ephemeris->pcvs, 
        gnss->ephemeris->pcvs, sizeof(pcv_t) * MAXSAT);
      phase_center_local_.reset(new PhaseCenter(gnss_local_->ephemeris->pcvs));
    }
  }

  // store/dump data for postProcessing--sbs
  if(is_post_process_) {
    storeLocalGnss(gnss);
  }

  // Find observation message
  bool has_observation = false;
  for (auto it : gnss->types) 
    if (it == GnssDataType::Observation) has_observation = true;
  if (!has_observation) return;

  // Check role
  bool has_observation_role = false;
  for (auto it : roles) 
    if (it == GnssRole::Rover || it == GnssRole::Reference || 
        it == GnssRole::Heading) has_observation_role = true;
  if (!has_observation_role) return;

  // Set to epoch data
  GnssMeasurement epoch;
  epoch.timestamp = gnss_common::gpsTimeToUtcTime(
    gnss_common::gtimeToDouble(gnss->observation->data[0].time));


  // Check if the epoch is valid
  for (auto it : roles) {
    if (it == GnssRole::Rover ){
      if (epoch.timestamp - last_obs_epoch_ < 1.0){
        return;
      }
      last_obs_epoch_ = epoch.timestamp; // record current epoch --sbs

    }
  }


  




  if(is_post_process_) {
    int ret = dumpLocalGnss(gnss_common::gtimeToDouble(gnss->observation->data[0].time));
    if(use_precise_product_ && ret < 3) 
    {
      LOG(INFO)<<" No precise product for PPP. ret = "<<ret;
      return;
    }
    else if(!use_precise_product_ && ret < 1) 
    {
      LOG(INFO)<<" No broadcast ephemeris for SPP/RTK. ret = "<<ret;
      return;
    }
    else{ }
  }


  epoch.role = role_out;
  epoch.tag = formator_tag.substr(4, formator_tag.size() - 4);
  epoch.position.setZero();
  double *rs, *dts, *var;
  double *rs_ssr, *dts_ssr, *var_ssr;
  double *rs_sp3, *dts_sp3, *var_sp3;

  obs_t *obs = gnss->observation;
  nav_t *nav = gnss_local_->ephemeris;
  int svh[MAXOBS], svh_ssr[MAXOBS], svh_sp3[MAXOBS], n = obs->n;
  rs = mat(6, n); dts = mat(2, n); var = mat(1, n);
  rs_ssr = mat(6, n); dts_ssr = mat(2, n); var_ssr = mat(1, n);
  rs_sp3 = mat(6, n); dts_sp3 = mat(2, n); var_sp3 = mat(1, n);
  for (int i = 0; i < MAXOBS; i++) {
    svh[i] = -1; svh_ssr[i] = -1; svh_sp3[i] = -1;
  }
  satposs(obs->data[0].time, obs->data, n,
      nav, EPHOPT_BRDC, rs, dts, var, svh);
  satposs(obs->data[0].time, obs->data, n,
      nav, EPHOPT_SSRAPC, rs_ssr, dts_ssr, var_ssr, svh_ssr);
  satposs(obs->data[0].time, obs->data, n,
      nav, EPHOPT_PREC,   rs_sp3, dts_sp3, var_sp3, svh_sp3);  //-sbs

  int num_invalid_ephemeris = 0, num_valid_ephemeris = 0;
  for (int i = 0; i < n; i++) {
    Satellite satellite;

    // system and prn
    int prn = 0;
    char strprnnum[3];
    switch (satsys(obs->data[i].sat, &prn)) {
      case SYS_GPS: satellite.prn = 'G'; break;
      case SYS_GLO: satellite.prn = 'R'; break;
      case SYS_GAL: satellite.prn = 'E'; break;
      case SYS_CMP: satellite.prn = 'C'; break;
      default: continue;
    }
    sprintf(strprnnum, "%02d", prn);
    satellite.prn.append(strprnnum);

    // satellite position and clock
    if (svh_ssr[i] != -1 && rs_ssr[i * 6] != 0 && dts_ssr[i * 2] != 0) {
      satellite.sat_position = Eigen::Map<Eigen::Vector3d>(rs_ssr + i * 6);
      satellite.sat_velocity = Eigen::Map<Eigen::Vector3d>(rs_ssr + 3 + i * 6);
      satellite.sat_clock = dts_ssr[i * 2] * CLIGHT;
      satellite.sat_frequency = dts_ssr[1 + i * 2] * CLIGHT;
      satellite.sat_type = SatEphType::Precise;
      num_valid_ephemeris++;
    }
    else if (svh_sp3[i] != -1 && rs_sp3[i * 6] != 0 && dts_sp3[i * 2] != 0) {
      satellite.sat_position = Eigen::Map<Eigen::Vector3d>(rs_sp3 + i * 6);
      satellite.sat_velocity = Eigen::Map<Eigen::Vector3d>(rs_sp3 + 3 + i * 6);
      satellite.sat_clock = dts_sp3[i * 2] * CLIGHT;
      satellite.sat_frequency = dts_sp3[1 + i * 2] * CLIGHT;
      satellite.sat_type = SatEphType::Precise;
      num_valid_ephemeris++;
    }
    else if (svh[i] != -1 && rs[i * 6] != 0 && dts[i * 2] != 0) {
      satellite.sat_position = Eigen::Map<Eigen::Vector3d>(rs + i * 6);
      satellite.sat_velocity = Eigen::Map<Eigen::Vector3d>(rs + 3 + i * 6);
      satellite.sat_clock = dts[i * 2] * CLIGHT;
      satellite.sat_frequency = dts[1 + i * 2] * CLIGHT;
      satellite.sat_type = SatEphType::Broadcast;
      num_valid_ephemeris++;
    }
    else {
      num_invalid_ephemeris++; continue;
    }

    // ionosphere
    // (TODO:--sbs) set ionoshpere with IonFile 
    if(gnss_common::isPpprtkMode()){
      // satellite.ionosphere = obs->data[i].iono;
      satellite.ionosphere = 0.0;
      satellite.ionosphere_type = IonoType::Augmentation;
    }
    else{
      satellite.ionosphere = 0.0;
      satellite.ionosphere_type = IonoType::None;
    }
    

    // observations
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      if (obs->data[i].P[j] == 0.0) continue;
      Observation observation;
      int freq_index;
      int code_type = obs->data[i].code[j];
      double freq = sat2freq(obs->data[i].sat, code_type, nav);
      if (freq == 0.0) continue;
      observation.wavelength = CLIGHT / freq;
      observation.pseudorange = obs->data[i].P[j];
      observation.phaserange = obs->data[i].L[j] * observation.wavelength;
      observation.doppler = obs->data[i].D[j] * observation.wavelength ; 
      // test --sbs
      // if(is_post_process_){
        observation.doppler *= (-1) ; 
      // }
      observation.SNR = obs->data[i].SNR[j] * 1.0e-3;
      // (test):to limit the influence of absense of SNR  --sbs20240111
      if(fabs(observation.SNR)<1e-6){
        observation.SNR = 40.0;
      }
      observation.CEI = 0; // to indicate the gross error from pseudorange --sbs

      observation.LLI = obs->data[i].LLI[j];
      observation.slip = false;
      observation.raw_code = code_type;
      satellite.observations.insert(std::make_pair(code_type, observation));
    }

    epoch.satellites.insert(std::make_pair(satellite.prn, satellite));
  }
  free(rs); free(dts); free(var);
  free(rs_ssr); free(dts_ssr); free(var_ssr);
  free(rs_sp3); free(dts_sp3); free(var_sp3);

  // check number of satellites
  if (epoch.satellites.size() == 0) return;

  // check if we should wait for more ephemeris
  double invalid_ephemeris_ratio = 
    getDivide(num_invalid_ephemeris, num_invalid_ephemeris + num_valid_ephemeris);
  if (invalid_ephemeris_ratio > 0.2) {
    LOG(INFO) << "Waiting for ephemeris. We still have " << num_invalid_ephemeris
      << " satellites that do not have ephemeris. Total number of satellite is "
      << num_invalid_ephemeris + num_valid_ephemeris;
      
    // return;
  }

  // reference station position
  if (role_out == GnssRole::Reference) {
    if(!use_base_position_cfile_){
      if (gnss_local_->antenna->pos[0] == 0.0 ) {
        LOG(INFO) << "Unable to get antenna position of reference station!";
        return;
      }
      epoch.position = 
        Eigen::Map<Eigen::Vector3d>(gnss_local_->antenna->pos);
    }
    else{
      epoch.position = base_position_cfile_;
    }
  }

  // GPS ionosphere parameters
  epoch.ionosphere_parameters = 
    Eigen::Map<Eigen::VectorXd>(gnss_local_->ephemeris->ion_gps, 8);

  // Troposphere 
  epoch.troposphere_wet = 0.0;
  
  // set PPP-RTK augmentation network --sbs
  if(gnss_common::isPpprtkMode()){
    epoch.ionospheric_network = gnss_local_->ion_network;
    epoch.tropospheric_network = gnss_local_->ztd_network;
  }
  // Set code bias
  epoch.code_bias = code_bias_local_;

  // Set phase bias
  epoch.phase_bias = phase_bias_local_;

  // Set phase center
  epoch.phase_center = phase_center_local_;

  // Call GNSS observation processor
  for (auto it_gnss_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(epoch);
    it_gnss_callback(estimator_data);
  }
}

// Update GNSS code bias to local
void GnssDataIntegration::updateCodeBias()
{
  nav_t *nav = gnss_local_->ephemeris;
  for (int i = 0; i < MAXSAT; i++) {
    // TODO: segment fault here!
    if (nav->ssr[i].isdcb)
    {
      std::vector<std::pair<int, double>> code_vs_value;
      for (int j = 0; j < MAXCODE; j++) {
        double cbias = nav->ssr[i].cbias[j];
        if (cbias != 0.0) {
          code_vs_value.push_back(std::make_pair(j + 1, cbias));
        }
      }
      std::string prn = gnss_common::satToPrn(i + 1);
      if (prn == "") continue;
      for (size_t j = 1; j < code_vs_value.size(); j++) {
        int code = code_vs_value[j].first;
        int code_base = code_vs_value[0].first;
        double dcb = code_vs_value[j].second - code_vs_value[0].second;
        code_bias_local_->setDcb(prn, code, code_base, dcb);
      }
    }
    else
    {
      std::string prn = gnss_common::satToPrn(i + 1);
      if (prn == "") continue;
      for (int j = 0; j < MAXCODE; j++) {
        double cbias = nav->ssr[i].cbias[j];
        if (cbias == 0.0) continue;
        int code = j + 1;
        code_bias_local_->setZdcb(prn, code, cbias);
      }
    }
  }
  code_bias_local_->arrangeToBases();
}

// Update GNSS TGD to local
void GnssDataIntegration::updateTgd()
{
  nav_t *nav = gnss_local_->ephemeris;
  for (int i = 0; i < MAXSAT; i++) {
    eph_t *eph = nav->eph + i;
    if (eph->sat <= 0) continue;
    std::string prn = gnss_common::satToPrn(eph->sat);
    if (prn == "") continue;
    if (prn[0] == 'G') {
      if (eph->tgd[0] != 0.0) {
        code_bias_local_->setTgdIsc(prn, TgdIscType::GpsTgd, eph->tgd[0]);
      }
    }
    else if (prn[0] == 'E') {
      if (eph->tgd[0] != 0.0) {
        code_bias_local_->setTgdIsc(prn, TgdIscType::GalileoBgdE1E5a, eph->tgd[0]);
      }
      if (eph->tgd[1] != 0.0) {
        code_bias_local_->setTgdIsc(prn, TgdIscType::GalileoBgdE1E5b, eph->tgd[1]);
      }
    }
    else if (prn[0] == 'C') {
      if (eph->tgd[0] != 0.0) {
        code_bias_local_->setTgdIsc(prn, TgdIscType::BdsTgdB1B3, eph->tgd[0]);
      }
      if (eph->tgd[1] != 0.0) {
        code_bias_local_->setTgdIsc(prn, TgdIscType::BdsTgdB2B3, eph->tgd[1]);
      }
    }
  }
  for (int i = 0; i < NSATGLO; i++) {
    geph_t *geph = nav->geph;
    if (geph->sat <= 0) continue;
    std::string prn = gnss_common::satToPrn(geph->sat);
    if (prn == "") continue;
    CHECK(prn[0] == 'R');
    if (geph->dtaun != 0.0) {
      code_bias_local_->setTgdIsc(prn, TgdIscType::GlonassTgd, geph->dtaun);
    }
  }
  code_bias_local_->arrangeToBases();
}

// Update GNSS phase bias to local
void GnssDataIntegration::updatePhaseBias()
{
  nav_t *nav = gnss_local_->ephemeris;
  for (int i = 0; i < MAXSAT; i++) {
    CHECK(!nav->ssr[i].isdpb) << "Currently unsupported!";
    if (nav->ssr[i].isdpb)
    {

    }
    else
    {
      std::string prn = gnss_common::satToPrn(i + 1);
      if (prn == "") continue;
      for (int j = 0; j < MAXCODE; j++) {
        double pbias = nav->ssr[i].pbias[j];
        if (pbias == 0.0) continue;
        int phase_id = gnss_common::getPhaseID(prn[0], j + 1);
        phase_bias_local_->setPhaseBias(prn, phase_id, pbias);
      }
    }
  }
}

// Update observation bias to local --sbs
int GnssDataIntegration::updateObservationBias(const double epoch){

  // Code bias
  if(false == code_bias_updated_){
    if(gnss_local_->obs_bias.zcbs.size()<=0) return 0;
    for(auto& zcb : gnss_local_->obs_bias.zcbs){
      // osb
      std::string prn = zcb.first;
      int code_id = zcb.second.code;
      double cbias = zcb.second.value;
      code_bias_local_->setZdcb(prn, code_id, cbias);

    }
    code_bias_local_->arrangeToBases();
    code_bias_updated_ = true;
  }

  // Phase bias
  if(gnss_local_->obs_bias.zpbs_map.size()<=0) return 0;
  // std::map<std::vector<std::pair<int,double>>>::iterator
  auto it = gnss_local_->obs_bias.zpbs_map.lower_bound(epoch);
  if(it == gnss_local_->obs_bias.zpbs_map.end()) return 0;
  LOG(INFO)<<"find phase bias: epoch - obs_bias->first = "<< epoch - it->first <<" s";

  // Clear old phase bias
  phase_bias_local_->clear();
  std::vector<std::pair<std::string,zpb_t>> &zpbs = it->second;
  for(auto& zpb : zpbs){
    // osb
    std::string prn = zpb.first;
    int phase_id = zpb.second.phase;
    double pbias = zpb.second.value;
    phase_bias_local_->setPhaseBias(prn, phase_id, pbias);

  }

  return 1;
}

// Data callback
void ImuDataIntegration::dataCallback(
  const std::string& input_tag, const std::shared_ptr<DataCluster>& data)
{
  if (data->imu && valid_) {
    mutex_.lock();
    handleIMU(input_tag, data->imu);
    mutex_.unlock();
  }
}

// Handle IMU data
void ImuDataIntegration::handleIMU(const std::string& formator_tag, 
                const std::shared_ptr<DataCluster::IMU>& imu)
{
  // Get role
  if (behaviors_.find(formator_tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << formator_tag << " not registered!";
    return;
  }
  std::vector<ImuRole> roles;
  bool has_major = false;
  for (size_t i = 0; i < behaviors_.at(formator_tag).size(); i++) {
    roles.push_back(ImuRole());
    option_tools::convert(behaviors_.at(formator_tag)[i], roles[i]);
    if (roles[i] == ImuRole::Minor) {
      LOG(WARNING) << "We do not support multiple IMUs currently!";
      return;
    }
    if (roles[i] == ImuRole::Major) has_major = true;
  }
  if (roles.size() > 1) {
    LOG(ERROR) << "A IMU should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current IMU was specified!";
    return;
  }

  // Convert IMU data
  ImuMeasurement epoch(imu->time, 
    Eigen::Map<Eigen::Vector3d>(imu->angular_velocity), 
    Eigen::Map<Eigen::Vector3d>(imu->acceleration));

  // Call INS processor
  for (auto it_imu_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(
      epoch, roles[0], formator_tag.substr(4, formator_tag.size() - 4));
    it_imu_callback(estimator_data);
  }
}

void OdomDataIntegration::dataCallback(
  const std::string& input_tag, const std::shared_ptr<DataCluster>& data)
{
  if (data->odometer && valid_) {
    mutex_.lock();
    handleOdometer(input_tag, data->odometer);
    mutex_.unlock();
  }
}

void OdomDataIntegration::handleOdometer(const std::string& formator_tag,
                              const std::shared_ptr<DataCluster::Odometer>& odom)
{
  // Get role
  if (behaviors_.find(formator_tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << formator_tag << " not registered!";
    return;
  }
  std::vector<OdomRole> roles;
  bool has_major = false;
  for (size_t i = 0; i < behaviors_.at(formator_tag).size(); i++) {
    roles.push_back(OdomRole());
    option_tools::convert(behaviors_.at(formator_tag)[i], roles[i]);
    if (roles[i] != OdomRole::Skid) {
      LOG(WARNING) << "We do not support multiple Odometers currently!";
      return;
    }
    if (roles[i] == OdomRole::Skid) has_major = true;
  }
  if (roles.size() > 1) {
    LOG(ERROR) << "An Odometer should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current Odometer was specified!";
    return;
  }

  // Convert Odometer data
  OdomMeasurement epoch(odom->time, 
    Eigen::Vector3d{0.0, odom->forward_velocity, 0.0}, 
    Eigen::Vector3d{0.0, 0.0, odom->bearing_angular_velocity});

  // Call Odometer processor
  for (auto it_odom_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(
      epoch, roles[0], formator_tag.substr(4, formator_tag.size() - 4));
    it_odom_callback(estimator_data);
  }  
}


// Data callback
void ImageDataIntegration::dataCallback(
  const std::string& input_tag, const std::shared_ptr<DataCluster>& data)
{ 
  if (data->image && valid_) {
    mutex_.lock();
    handleImage(input_tag, data->image);
    mutex_.unlock();
  }
}

// Handle Image data
void ImageDataIntegration::handleImage(const std::string& formator_tag, 
                             const std::shared_ptr<DataCluster::Image>& image)
{
  // Get role
  if (behaviors_.find(formator_tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << formator_tag << " not registered!";
    return;
  }
  std::vector<CameraRole> roles;
  bool has_mono = false;
  for (size_t i = 0; i < behaviors_.at(formator_tag).size(); i++) {
    roles.push_back(CameraRole());
    option_tools::convert(behaviors_.at(formator_tag)[i], roles[i]);
    if (roles[i] != CameraRole::Mono) {
      LOG(WARNING) << "We do not support multiple Cameras currently!";
      return;
    }
    else has_mono = true;
  }
  if (!has_mono) return;
  if (roles.size() > 1) {
    LOG(ERROR) << "A camera should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current camera was specified!";
    return;
  }

  // Convert Image data
  CHECK(image->step == 1) << "We only support image input with step size of 1!";
  cv::Mat image_mat_raw(image->height, image->width, CV_8UC(image->step), image->image);
  // clone to allocate new memory
  std::shared_ptr<cv::Mat> image_mat = std::make_shared<cv::Mat>(image_mat_raw.clone());

  // Call Image processor
  for (auto it_image_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(image_mat, 
      roles[0], formator_tag.substr(4, formator_tag.size() - 4), image->time);
    it_image_callback(estimator_data);
  }
}

// Solution data integration
SolutionDataIntegration::SolutionDataIntegration(
  const std::shared_ptr<EstimatingBase>& estimating, 
  const std::shared_ptr<EstimatingBase>& input_estimating,
  const std::string& input_tag,
  const std::vector<std::string>& roles)
{
  // Bind this->dataCallback to input estimating handle
  Streaming::DataCallback callback = std::bind(
    &DataIntegrationBase::dataCallback, 
    this, std::placeholders::_1, std::placeholders::_2);
  input_estimating->setOutputDataCallback(callback);

  // Bind estimator->estimatorDataCallback to this->estimator_callbacks_
  EstimatorDataCallback estimator_callback = 
    std::bind(&EstimatingBase::estimatorDataCallback, 
      estimating.get(), std::placeholders::_1);
  estimator_callbacks_.push_back(estimator_callback);

  // Declare behaviors for stream input
  behaviors_.insert(std::make_pair(input_tag, roles));

  // set flags
  is_from_estimator_ = true;
  valid_ = true;
}

// Data callback
void SolutionDataIntegration::dataCallback(
  const std::string& input_tag, const std::shared_ptr<DataCluster>& data)
{
  if (data->solution && valid_) {
    mutex_.lock();
    handleSolution(input_tag, data->solution);
    mutex_.unlock();
  }
}

// Handle Solution data
void SolutionDataIntegration::handleSolution(const std::string& input_tag, 
                    const std::shared_ptr<Solution>& solution)
{
  // Get role
  if (behaviors_.find(input_tag) == behaviors_.end()) {
    LOG(ERROR) << "Input tag " << input_tag << " not registered!";
    return;
  }
  std::vector<SolutionRole> roles;
  for (size_t i = 0; i < behaviors_.at(input_tag).size(); i++) {
    roles.push_back(SolutionRole());
    option_tools::convert(behaviors_.at(input_tag)[i], roles[i]);
  }
  if (roles.size() > 1) {
    LOG(ERROR) << "A solution stream should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current solution stream was specified!";
    return;
  }

  // Check and adjust solution role
  roles[0] = adjustSolutionRole(*solution, roles[0]);

  // Call Solution processor (loosely coupled estimators)
  for (auto it_solution_callback : estimator_callbacks_) {
    EstimatorDataCluster estimator_data(
      *solution, roles[0], input_tag.substr(4, input_tag.size() - 4));
    it_solution_callback(estimator_data);
  }
}

// Check and adjust solution role
SolutionRole SolutionDataIntegration::adjustSolutionRole(
  const Solution& solution, const SolutionRole& role)
{
  if (role == SolutionRole::None) return role;
  if (role == SolutionRole::Position) {
    bool position_valid = checkPosition(solution);
    if (position_valid) return role;
    else return SolutionRole::None;
  }
  if (role == SolutionRole::Velocity) {
    bool velocity_valid = checkVelocity(solution);
    if (velocity_valid) return role;
    else return SolutionRole::None;
  }
  if (role == SolutionRole::Attitude) {
    bool attitude_valid = checkAttitude(solution);
    if (attitude_valid) return role;
    else return SolutionRole::None;
  }
  if (role == SolutionRole::Pose) {
    bool position_valid = checkPosition(solution);
    bool attitude_valid = checkAttitude(solution);
    if (position_valid && attitude_valid) return role;
    else if (position_valid) return SolutionRole::Position;
    else if (attitude_valid) return SolutionRole::Attitude;
    else return SolutionRole::None;
  }
  if (role == SolutionRole::PositionAndVelocity) {
    bool position_valid = checkPosition(solution);
    bool velocity_valid = checkVelocity(solution);
    if (position_valid && velocity_valid) return role;
    else if (position_valid) return SolutionRole::Position;
    else if (velocity_valid) return SolutionRole::Velocity;
    else return SolutionRole::None;
  }
  if (role == SolutionRole::PoseAndVelocity) {
    bool position_valid = checkPosition(solution);
    bool velocity_valid = checkVelocity(solution);
    bool attitude_valid = checkAttitude(solution);
    if (position_valid && velocity_valid && attitude_valid) return role;
    else if (position_valid && attitude_valid) return SolutionRole::Pose;
    else if (position_valid && velocity_valid) return SolutionRole::PositionAndVelocity;
    else if (position_valid) return SolutionRole::Position;
    else if (velocity_valid) return SolutionRole::Velocity;
    else if (attitude_valid) return SolutionRole::Attitude;
    else return SolutionRole::None;
  }
  return SolutionRole::None;
}

}
