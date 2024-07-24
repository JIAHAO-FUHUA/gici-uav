/**
 * @Function: Stream functions
 *
 * @Author  : Cheng Chi
 * @Email   : chichengcn@sjtu.edu.cn
 *
 * Copyright (C) 2023 by Cheng Chi, All rights reserved.
 **/
#include "gici/stream/formator.h"

#include <mutex>
#include <glog/logging.h>
#include <vikit/timer.h>

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"

#include <sstream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>


namespace gici
{

  DataCluster::DataCluster(FormatorType type)
  {
    if (type == FormatorType::RTCM2    || type == FormatorType::RTCM3 ||
        type == FormatorType::GnssRaw  || type == FormatorType::DcbFile ||
        type == FormatorType::AtxFile  || type == FormatorType::RnxOFile ||
        type == FormatorType::RnxNFile || type == FormatorType::OsbFile ||
        type == FormatorType::Sp3File  || type == FormatorType::ClkFile ||
        type == FormatorType::IonFile  || type == FormatorType::ZtdFile)
    {
      gnss = std::make_shared<GNSS>();
      gnss->init();
      return;
    }
    if (type == FormatorType::IMUPack || type == FormatorType::IMUImr)
    {
      imu = std::make_shared<IMU>();
      return;
    }
    if (type == FormatorType::OptionPack)
    {
      option = std::make_shared<Option>();
      return;
    }
    if (type == FormatorType::NMEA)
    {
      solution = std::make_shared<Solution>();
      return;
    }
    if (type == FormatorType::ImagePack || type == FormatorType::ImageV4L2 || type == FormatorType::ImageFile)
    {
      LOG(FATAL) << "Cannot initialize DataCluster::Image: "
                 << "Image length should be given!";
    }
    if (type == FormatorType::Odometer)  //--sbs
    {
      odometer = std::make_shared<Odometer>();
      return;
    }
    LOG(FATAL) << "Cannot initialize: Data format not recognized!";
  }

  DataCluster::DataCluster(FormatorType type, int _width, int _height, int _step)
  {
    if (type == FormatorType::ImagePack || type == FormatorType::ImageV4L2|| type == FormatorType::ImageFile)
    {
      image = std::make_shared<Image>();
      image->init(_width, _height, _step);
      return;
    }
    LOG(FATAL) << "Cannot initialize: Data format not recognized!";
  }

  DataCluster::~DataCluster()
  {
    if (gnss != nullptr)
      gnss->free();
    if (image != nullptr)
      image->free();
  }

  void DataCluster::GNSS::init()
  {
    if (!(observation = (obs_t *)malloc(sizeof(obs_t))) ||
        !(observation->data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS * MaxDataSize::MaxObsEpoch)) || //--sbs
        !(ephemeris = (nav_t *)malloc(sizeof(nav_t))) ||
        !(ephemeris->eph = (eph_t *)malloc(sizeof(eph_t) * MAXSAT * 20)) ||
        !(ephemeris->geph = (geph_t *)malloc(sizeof(geph_t) * NSATGLO * 20)) ||
        !(ephemeris->peph = (peph_t *)malloc(sizeof(peph_t))) ||
        !(ephemeris->pclk = (pclk_t *)malloc(sizeof(pclk_t))) ||
        !(antenna = (sta_t *)malloc(sizeof(sta_t))))
    {
      free();
    }
    eph_t eph0 = {0, -1, -1};
    geph_t geph0 = {0, -1};
    peph_t peph0 = {0}; //--sbs
    pclk_t pclk0 = {0};
    for (int i = 0; i < MAXSAT * 20; i++)
      ephemeris->eph[i] = eph0;
    for (int i = 0; i < NSATGLO * 20; i++)
      ephemeris->geph[i] = geph0;
    ephemeris->peph[0] = peph0;
    ephemeris->pclk[0] = pclk0;

    ephemeris->n = MAXSAT * 20;
    ephemeris->ng = NSATGLO * 20;
    ephemeris->ne = 0; //--sbs
    ephemeris->nc = 0;
    memset(ephemeris->ssr, 0, sizeof(ssr_t) * MAXSAT);
    memset(antenna, 0, sizeof(sta_t));
  }

  void DataCluster::GNSS::free()
  {
    ::free(observation->data);
    ::free(observation);
    observation = NULL;
    ::free(ephemeris->eph);
    ::free(ephemeris->geph);
    ::free(ephemeris->peph);
    ::free(ephemeris->pclk);
    ::free(ephemeris);
    ephemeris = NULL;
    ::free(antenna);
    antenna = NULL;
  }

  void DataCluster::Image::init(int _width, int _height, int _step)
  {
    width = _width;
    height = _height;
    step = _step;
    if (!(image = (uint8_t *)malloc(sizeof(uint8_t) * width * height * step)))
      free();
  }

  void DataCluster::Image::free()
  {
    ::free(image);
  }

  namespace gnss_common
  {

    // Update observation data
    extern void updateObservation(
        obs_t *obs, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      int n = 0;
      for (int i = 0; i < obs->n; i++)
      {
        gnss_data->observation[0].data[n++] = obs->data[i];
      }
      gnss_data->observation[0].n = n;
      sortobs(&gnss_data->observation[0]);
    }

    // Update ephemeris
    extern void updateEphemeris(
        nav_t *nav, int sat, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      eph_t *eph1, *eph2, *eph3;
      geph_t *geph1, *geph2, *geph3;
      int prn;
      if (satsys(sat, &prn) != SYS_GLO)
      {
        eph1 = nav->eph + sat - 1;
        eph2 = gnss_data->ephemeris->eph + sat - 1;
        eph3 = gnss_data->ephemeris->eph + sat - 1 + MAXSAT;
        if (eph2->ttr.time == 0 ||
            (eph1->iode != eph3->iode && eph1->iode != eph2->iode) ||
            (timediff(eph1->toe, eph3->toe) != 0.0 &&
             timediff(eph1->toe, eph2->toe) != 0.0) ||
            (timediff(eph1->toc, eph3->toc) != 0.0 &&
             timediff(eph1->toc, eph2->toc) != 0.0))
        {
          *eph3 = *eph2;
          *eph2 = *eph1;
        }
      }
      else
      {
        geph1 = nav->geph + prn - 1;
        geph2 = gnss_data->ephemeris->geph + prn - 1;
        geph3 = gnss_data->ephemeris->geph + prn - 1 + MAXPRNGLO;
        if (geph2->tof.time == 0 ||
            (geph1->iode != geph3->iode && geph1->iode != geph2->iode))
        {
          *geph3 = *geph2;
          *geph2 = *geph1;
        }
      }
    }

    // Update ion/utc parameters
    extern void updateIonAndUTC(
        nav_t *nav, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      matcpy(gnss_data->ephemeris->utc_gps, nav->utc_gps, 8, 1);
      matcpy(gnss_data->ephemeris->utc_glo, nav->utc_glo, 8, 1);
      matcpy(gnss_data->ephemeris->utc_gal, nav->utc_gal, 8, 1);
      matcpy(gnss_data->ephemeris->utc_qzs, nav->utc_qzs, 8, 1);
      matcpy(gnss_data->ephemeris->utc_cmp, nav->utc_cmp, 8, 1);
      matcpy(gnss_data->ephemeris->utc_irn, nav->utc_irn, 9, 1);
      matcpy(gnss_data->ephemeris->utc_sbs, nav->utc_sbs, 4, 1);
      matcpy(gnss_data->ephemeris->ion_gps, nav->ion_gps, 8, 1);
      matcpy(gnss_data->ephemeris->ion_gal, nav->ion_gal, 4, 1);
      matcpy(gnss_data->ephemeris->ion_qzs, nav->ion_qzs, 8, 1);
      matcpy(gnss_data->ephemeris->ion_cmp, nav->ion_cmp, 8, 1);
      matcpy(gnss_data->ephemeris->ion_irn, nav->ion_irn, 8, 1);
    }

    // Update antenna position
    extern void updateAntennaPosition(
        sta_t *sta, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      if (sta == NULL)
      {
        LOG(ERROR) << "Antenna position parameter has NULL pointer!";
        return;
      }
      for (int i = 0; i < 3; i++)
      {
        gnss_data->antenna->pos[i] = sta->pos[i];
      }
      double pos[3], del[3] = {0}, dr[3];
      ecef2pos(sta->pos, pos);
      if (sta->deltype == 1)
      { // ECEF
        del[2] = sta->hgt;
        enu2ecef(pos, del, dr);
        for (int i = 0; i < 3; i++)
        {
          gnss_data->antenna->pos[i] += sta->del[i] + dr[i];
        }
      }
      else
      { // ENU
        enu2ecef(pos, sta->del, dr);
        for (int i = 0; i < 3; i++)
        {
          gnss_data->antenna->pos[i] += dr[i];
        }
      }
    }

    // Update ssr corrections
    extern void updateSsr(
        ssr_t *ssr, std::shared_ptr<DataCluster::GNSS> &gnss_data,
        std::vector<UpdateSsrType> type)
    {
      if (ssr == NULL)
      {
        LOG(ERROR) << "SSR parameter has NULL pointer!";
        return;
      }
      for (int i = 0; i < MAXSAT; i++)
      {
        if (!ssr[i].update)
          continue;
        ssr[i].update = 0;

        // update ephemeris
        if (std::find(type.begin(), type.end(),
                      UpdateSsrType::Ephemeris) != type.end())
        {
          // check consistency between iods of orbit and clock
          if (ssr[i].iod[0] != ssr[i].iod[1])
            continue;

          // TODO: check corresponding ephemeris exists
          for (int j = 0; j < 4; j++)
          {
            gnss_data->ephemeris->ssr[i].t0[j] = ssr[i].t0[j];
            gnss_data->ephemeris->ssr[i].udi[j] = ssr[i].udi[j];
            gnss_data->ephemeris->ssr[i].iod[j] = ssr[i].iod[j];
          }
          gnss_data->ephemeris->ssr[i].iode = ssr[i].iode;
          gnss_data->ephemeris->ssr[i].iodcrc = ssr[i].iodcrc;
          gnss_data->ephemeris->ssr[i].ura = ssr[i].ura;
          gnss_data->ephemeris->ssr[i].refd = ssr[i].refd;
          for (int j = 0; j < 3; j++)
          {
            gnss_data->ephemeris->ssr[i].deph[j] = ssr[i].deph[j];
            gnss_data->ephemeris->ssr[i].ddeph[j] = ssr[i].ddeph[j];
            gnss_data->ephemeris->ssr[i].dclk[j] = ssr[i].dclk[j];
          }
          gnss_data->ephemeris->ssr[i].hrclk = ssr[i].hrclk;
        }
        // update code bias
        if (std::find(type.begin(), type.end(),
                      UpdateSsrType::CodeBias) != type.end())
        {
          gnss_data->ephemeris->ssr[i].t0[4] = ssr[i].t0[4];
          gnss_data->ephemeris->ssr[i].udi[4] = ssr[i].udi[4];
          memcpy(gnss_data->ephemeris->ssr[i].cbias,
                 ssr[i].cbias, sizeof(float) * MAXCODE);
          gnss_data->ephemeris->ssr[i].isdcb = ssr[i].isdcb;
        }
        // update phase bias
        if (std::find(type.begin(), type.end(),
                      UpdateSsrType::PhaseBias) != type.end())
        {
          gnss_data->ephemeris->ssr[i].t0[5] = ssr[i].t0[5];
          gnss_data->ephemeris->ssr[i].udi[5] = ssr[i].udi[5];
          memcpy(gnss_data->ephemeris->ssr[i].pbias,
                 ssr[i].pbias, sizeof(double) * MAXCODE);
          memcpy(gnss_data->ephemeris->ssr[i].stdpb,
                 ssr[i].stdpb, sizeof(float) * MAXCODE);
          gnss_data->ephemeris->ssr[i].yaw_ang = ssr[i].yaw_ang;
          gnss_data->ephemeris->ssr[i].yaw_rate = ssr[i].yaw_rate;
          gnss_data->ephemeris->ssr[i].isdpb = ssr[i].isdpb;
        }

        if (type.size() > 0)
        {
          gnss_data->ephemeris->ssr[i].update = 1;
        }
      }
    }

    extern void updatePreciseEph(
        nav_t *nav, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      gnss_data->ephemeris->peph = nav->peph;
      // memcpy(gnss_data->ephemeris->peph, nav->peph, sizeof(peph_t) * nav->ne );
      gnss_data->ephemeris->ne = nav->ne;
    }

    extern void updatePreciseClk(
        nav_t *nav, std::shared_ptr<DataCluster::GNSS> &gnss_data)
    {
      gnss_data->ephemeris->pclk = nav->pclk;
      // memcpy(gnss_data->ephemeris->pclk, nav->pclk, sizeof(pclk_t)* nav->nc );
      gnss_data->ephemeris->nc = nav->nc;
    }

    // Select data from GNSS stream
    extern void updateStreamData(int ret, obs_t *obs, nav_t *nav,
                                 sta_t *sta, ssr_t *ssr, int iobs, int sat,
                                 std::vector<std::shared_ptr<DataCluster::GNSS>> &gnss_data)
    {
      GnssDataType type = static_cast<GnssDataType>(ret);
      // Observation data
      if (type == GnssDataType::Observation)
      {
        updateObservation(obs, gnss_data[iobs]);
      }
      // Ephemeris data
      else if (type == GnssDataType::Ephemeris)
      {
        updateEphemeris(nav, sat, gnss_data[0]);
      }
      // Ionosphere parameters
      else if (type == GnssDataType::IonAndUtcPara)
      {
        updateIonAndUTC(nav, gnss_data[0]);
      }
      // Antenna position parameters
      else if (type == GnssDataType::AntePos)
      {
        updateAntennaPosition(sta, gnss_data[0]);
      }
      // SSR (precise ephemeris, DCBs, etc..)
      else if (type == GnssDataType::SSR)
      {
        updateSsr(ssr, gnss_data[0]);
      }
      // Precise Ephemeris data
      else if (type == GnssDataType::PreciseEph)
      {
        updatePreciseEph(nav, gnss_data[0]);
      }
      // Precise Clock data
      else if (type == GnssDataType::PreciseClk)
      {
        updatePreciseClk(nav, gnss_data[0]);
      }
    }
  }

// Load option with info
#define LOAD_COMMON(opt)                                     \
  if (!option_tools::safeGet(node, #opt, &option.opt))       \
  {                                                          \
    LOG(INFO) << __FUNCTION__ << ": Unable to load " << #opt \
              << ". Using default instead.";                 \
  }
// Load option with fatal error
#define LOAD_REQUIRED(opt)                                            \
  if (!option_tools::safeGet(node, #opt, &option.opt))                \
  {                                                                   \
    LOG(FATAL) << __FUNCTION__ << ": Unable to load " << #opt << "!"; \
  }

  // RTCM 2 ----------------------------------------------------
  // Load date
  inline bool loadStartTime(YAML::Node &node, double &start_time)
  {
    std::string str;
    if (!option_tools::safeGet(node, "start_time", &str))
    {
      LOG(INFO) << __FUNCTION__ << ": Unable to load "
                << "start_time"
                << ". Using default instead.";
      return false;
    }
    std::string strs[4];
    int index = 0;
    for (size_t i = 0; i < str.size(); i++)
    {
      if (str[i] == '.')
      {
        index++;
        continue;
      }
      strs[index] = strs[index] + str[i];
    }
    CHECK(index == 2) << "Start time format illegal!";
    int year = atoi(strs[0].data());
    int month = atoi(strs[1].data());
    int day = atoi(strs[2].data());
    int hour = 12;
    int min = 0, sec = 0;
    double ep[] = {(double)year, (double)month, (double)day,
                   (double)hour, (double)min, (double)sec};
    start_time = gnss_common::gtimeToDouble(epoch2time(ep));

    return true;
  }

  // path for file list in RNX/Ion/Ztd file   ---------------------------------
  // Load path
  inline bool loadPath(YAML::Node &node, std::string &path)
  {
    // std::string str;
    if (!option_tools::safeGet(node, "filepath", &path))
    {
      LOG(INFO) << __FUNCTION__ << ": Unable to load "
                << "path"
                << ". Using default instead: " << path;
      return false;
    }

    return true;
  }

  RTCM2Formator::RTCM2Formator(Option &option)
  {
    type_ = FormatorType::RTCM2;

    memset(&rtcm_, 0, sizeof(rtcm_t));
    init_rtcm(&rtcm_);
    sprintf(rtcm_.opt, "-EPHALL");
    rtcm_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::RTCM2; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  RTCM2Formator::RTCM2Formator(YAML::Node &node)
  {
    Option option;
    option.start_time = vk::Timer::getCurrentTime();
    loadStartTime(node, option.start_time);

    type_ = FormatorType::RTCM2;

    memset(&rtcm_, 0, sizeof(rtcm_t));
    init_rtcm(&rtcm_);
    sprintf(rtcm_.opt, "-EPHALL");
    rtcm_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::RTCM2; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  RTCM2Formator::~RTCM2Formator()
  {
    free_rtcm(&rtcm_);
  }

  // Decode stream to data
  int RTCM2Formator::decode(const uint8_t *buf, int size,
                            std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if (size == 0)
      return 0; //-sbs

    // Clear old informations and get GNSS data handle
    std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
    for (size_t i = 0; i < data_.size(); i++)
    {
      data_[i]->gnss->types.clear();
      gnss_data.push_back(data_[i]->gnss);
    }

    bool is_observation = false;
    bool has_others = false;
    int iobs = 0;
    for (int i = 0; i < size; i++)
    {
      int ret = input_rtcm2(&rtcm_, buf[i]);
      if (ret <= 0)
        continue;

      obs_t *obs = &rtcm_.obs;
      nav_t *nav = &rtcm_.nav;
      sta_t *sta = &rtcm_.sta;
      ssr_t *ssr = rtcm_.ssr;
      int sat = rtcm_.ephsat;
      gnss_common::updateStreamData(
          ret, obs, nav, sta, ssr, iobs, sat, gnss_data);
      GnssDataType type = static_cast<GnssDataType>(ret);
      std::shared_ptr<DataCluster::GNSS> &gnss =
          type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
      if (std::find(gnss->types.begin(), gnss->types.end(), type) == gnss->types.end())
      {
        gnss->types.push_back(type);
      }

      if (type == GnssDataType::Observation)
      {
        if (iobs < MaxDataSize::RTCM2)
          iobs++;
        if (iobs >= MaxDataSize::RTCM2)
        {
          LOG(WARNING) << "Max data length surpassed!";
          break;
        }
        is_observation = true;
      }
      else
      {
        has_others = true;
      }
    }

    data = data_;

    return is_observation ? iobs : has_others;
  }

  // Encode data to stream
  int RTCM2Formator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "RTCM2 Encoding not supported!";
    return 0;
  }

  // RTCM 3 -------------------------------------------------
  RTCM3Formator::RTCM3Formator(Option &option)
  {
    type_ = FormatorType::RTCM3;

    memset(&rtcm_, 0, sizeof(rtcm_t));
    init_rtcm(&rtcm_);
    sprintf(rtcm_.opt, "-EPHALL");
    rtcm_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::RTCM3; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  RTCM3Formator::RTCM3Formator(YAML::Node &node)
  {
    Option option;
    option.start_time = vk::Timer::getCurrentTime();
    loadStartTime(node, option.start_time);

    type_ = FormatorType::RTCM3;

    memset(&rtcm_, 0, sizeof(rtcm_t));
    init_rtcm(&rtcm_);
    sprintf(rtcm_.opt, "-EPHALL");
    rtcm_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::RTCM3; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  RTCM3Formator::~RTCM3Formator()
  {
    free_rtcm(&rtcm_);
  }

  // Decode stream to data
  int RTCM3Formator::decode(const uint8_t *buf, int size,
                            std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if (size == 0)
      return 0; //-sbs

    // Clear old informations and get GNSS data handle
    std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
    for (size_t i = 0; i < data_.size(); i++)
    {
      data_[i]->gnss->types.clear();
      gnss_data.push_back(data_[i]->gnss);
    }

    bool is_observation = false;
    bool has_others = false;
    int iobs = 0;
    for (int i = 0; i < size; i++)
    {
      int ret = input_rtcm3(&rtcm_, buf[i]);
      if (ret <= 0)
        continue;

      obs_t *obs = &rtcm_.obs;
      nav_t *nav = &rtcm_.nav;
      sta_t *sta = &rtcm_.sta;
      ssr_t *ssr = rtcm_.ssr;
      int sat = rtcm_.ephsat;
      gnss_common::updateStreamData(
          ret, obs, nav, sta, ssr, iobs, sat, gnss_data);
      GnssDataType type = static_cast<GnssDataType>(ret);
      std::shared_ptr<DataCluster::GNSS> &gnss =
          type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
      if (std::find(gnss->types.begin(), gnss->types.end(), type) == gnss->types.end())
      {
        gnss->types.push_back(type);
      }

      if (type == GnssDataType::Observation)
      {
        if (iobs < MaxDataSize::RTCM3)
          iobs++;
        if (iobs >= MaxDataSize::RTCM3)
        {
          LOG(WARNING) << "Max data length surpassed!";
          break;
        }
        is_observation = true;
      }
      else
      {
        has_others = true;
      }
    }

    data = data_;

    return is_observation ? iobs : has_others;
  }

  // Encode data to stream
  int RTCM3Formator::encode(
      const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
#if 0 // not finished yet
  // Check the control structure
  std::map<GnssDataType, bool> type_valid;
  type_valid.insert(std::make_pair(GnssDataType::Observation, false));
  type_valid.insert(std::make_pair(GnssDataType::Ephemeris, false));
  type_valid.insert(std::make_pair(GnssDataType::AntePos, false));
  type_valid.insert(std::make_pair(GnssDataType::IonAndUtcPara, false));
  type_valid.insert(std::make_pair(GnssDataType::SSR, false));
  type_valid.insert(std::make_pair(GnssDataType::PhaseCenter, false));
  for (auto it : data->gnss->types) {
    type_valid.at(it) = true;
  }

  // Encode data
  std::vector<int> msg_obs = {1077, 1087, 1097, 1127};
  std::vector<int> msg_eph = {1019, 1020, 1045, 1046, 1042};
  std::vector<int> msg_ant = {1005};
  int n = 0;
  rtcm_t rtcm = rtcm_;
  memcpy(&rtcm.obs, data->gnss->observation, sizeof(obs_t));
  memcpy(&rtcm.nav, data->gnss->ephemeris, sizeof(nav_t));
  memcpy(&rtcm.sta, data->gnss->antenna, sizeof(sta_t));
  memcpy(rtcm.ssr, data->gnss->ephemeris->ssr, sizeof(ssr_t) * MAXSAT);

  if (type_valid.at(GnssDataType::Observation)) {
    // Set time
    rtcm.time = rtcm.obs.data[0].time;
    if (fabs(timediff(rtcm.time, rtcm_.time)) > 30.0) rtcm_.time = rtcm.time;

    for (size_t i = 0; i < msg_obs.size(); i++) {
      gen_rtcm3(&rtcm, msg_obs[i], 0, i != 3);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }
  if (type_valid.at(GnssDataType::Ephemeris)) {
    for (size_t i = 0; i < msg_eph.size(); i++) {
      gen_rtcm3(&rtcm, msg_eph[i], 0, i != 4);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }
  if (type_valid.at(GnssDataType::AntePos)) {
    for (size_t i = 0; i < msg_ant.size(); i++) {
      gen_rtcm3(&rtcm, msg_ant[i], 0, i != 3);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }

  return n;
#else
    LOG(ERROR) << "RTCM3 Encoding not supported!";
    return 0;
#endif
  }

  // GNSS raw --------------------------------------------------------
  GnssRawFormator::GnssRawFormator(Option &option)
  {
    type_ = FormatorType::GnssRaw;
    option_tools::convert(option.sub_type, format_);

    init_raw(&raw_, static_cast<int>(format_));
    raw_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  GnssRawFormator::GnssRawFormator(YAML::Node &node)
  {
    Option option;
    option.start_time = vk::Timer::getCurrentTime();
    loadStartTime(node, option.start_time);
    LOAD_REQUIRED(sub_type);

    type_ = FormatorType::GnssRaw;
    option_tools::convert(option.sub_type, format_);

    init_raw(&raw_, static_cast<int>(format_));
    raw_.time = gnss_common::doubleToGtime(option.start_time);
    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }
  }

  GnssRawFormator::~GnssRawFormator()
  {
    free_raw(&raw_);
  }

  // Decode stream to data
  int GnssRawFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if (size == 0)
      return 0; //-sbs

    // Clear old informations and get GNSS data handle
    std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
    for (size_t i = 0; i < data_.size(); i++)
    {
      data_[i]->gnss->types.clear();
      gnss_data.push_back(data_[i]->gnss);
    }

    bool is_observation = false;
    bool has_others = false;
    int iobs = 0;
    for (int i = 0; i < size; i++)
    {
      int ret = input_raw(&raw_, static_cast<int>(format_), buf[i]);
      if (ret <= 0)
        continue;

      obs_t *obs = &raw_.obs;
      nav_t *nav = &raw_.nav;
      sta_t *sta = &raw_.sta;
      int sat = raw_.ephsat;
      gnss_common::updateStreamData(
          ret, obs, nav, sta, NULL, iobs, sat, gnss_data);
      GnssDataType type = static_cast<GnssDataType>(ret);
      std::shared_ptr<DataCluster::GNSS> &gnss =
          type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
      if (std::find(gnss->types.begin(), gnss->types.end(), type) == gnss->types.end())
      {
        gnss->types.push_back(type);
      }

      if (type == GnssDataType::Observation)
      {
        if (iobs < MaxDataSize::GnssRaw)
          iobs++;
        if (iobs >= MaxDataSize::GnssRaw)
        {
          LOG(WARNING) << "Max data length surpassed!";
          break;
        }
        is_observation = true;

        // modify some values
        if (format_ == GnssRawFormats::Tersus || format_ == GnssRawFormats::Novatel)
        {
          obs_t *obs = gnss->observation;
          for (int i = 0; i < obs->n; i++)
          {
            for (int j = 0; j < NFREQ + NEXOBS; j++)
            {
              obs->data[i].D[j] = -obs->data[i].D[j];
            }
          }
        }
      }
      else
      {
        has_others = true;
      }
    }

    data = data_;

    return is_observation ? iobs : has_others;
  }

  // Encode data to stream
  int GnssRawFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "GNSS-Raw Encoding not supported!";
    return 0;
  }

  // Image V4L2 ------------------------------------------------
  ImageV4L2Formator::ImageV4L2Formator(Option &option)
  {
    type_ = FormatorType::ImageV4L2;

    init_img(&image_, option.width, option.height, option.step);
    data_.push_back(std::make_shared<DataCluster>(
        FormatorType::ImageV4L2, option.width, option.height, option.step));
  }

  ImageV4L2Formator::ImageV4L2Formator(YAML::Node &node)
  {
    Option option;
    LOAD_REQUIRED(width);
    LOAD_REQUIRED(height);
    LOAD_COMMON(step);

    type_ = FormatorType::ImageV4L2;

    init_img(&image_, option.width, option.height, option.step);
    data_.push_back(std::make_shared<DataCluster>(
        FormatorType::ImageV4L2, option.width, option.height, option.step));
  }

  ImageV4L2Formator::~ImageV4L2Formator()
  {
    free_img(&image_);
  }

  // Decode stream to data
  int ImageV4L2Formator::decode(const uint8_t *buf, int size,
                                std::vector<std::shared_ptr<DataCluster>> &data)
  {
    int ret = input_image_v4l2(&image_, buf, size);
    if (ret <= 0)
      return 0;

    memcpy(data_[0]->image->image, image_.image,
           sizeof(uint8_t) * image_.width * image_.height * image_.step);
    data = data_;

    return 1;
  }

  // Encode data to stream
  int ImageV4L2Formator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Image-V4L2 Encoding not supported!";
    return 0;
  }

  // Image pack -------------------------------------------------
  ImagePackFormator::ImagePackFormator(Option &option)
  {
    type_ = FormatorType::ImagePack;

    init_img(&image_, option.width, option.height, option.step);
    for (int i = 0; i < MaxDataSize::ImagePack; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(
          FormatorType::ImagePack, option.width, option.height, option.step));
    }
  }

  ImagePackFormator::ImagePackFormator(YAML::Node &node)
  {
    Option option;
    LOAD_REQUIRED(width);
    LOAD_REQUIRED(height);
    LOAD_COMMON(step);

    type_ = FormatorType::ImagePack;

    init_img(&image_, option.width, option.height, option.step);
    for (int i = 0; i < MaxDataSize::ImagePack; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(
          FormatorType::ImagePack, option.width, option.height, option.step));
    }
  }

  ImagePackFormator::~ImagePackFormator()
  {
    free_img(&image_);
  }

  // Decode stream to data
  int ImagePackFormator::decode(const uint8_t *buf, int size,
                                std::vector<std::shared_ptr<DataCluster>> &data)
  {
    int iobs = 0;
    for (int i = 0; i < size; i++)
    {
      int ret = input_image(&image_, buf[i]);
      if (ret <= 0)
        continue;

      data_[iobs]->image->time = gnss_common::gtimeToDouble(image_.time);
      // TODO: This memcpy may increase memory occupation, according to
      // https://bbs.csdn.net/topics/390705325, maybe it is caused by the
      // convertion between physical and vitural memory, and this is not
      // a memory leak.
      memcpy(data_[iobs]->image->image, image_.image,
             sizeof(uint8_t) * image_.width * image_.height * image_.step);

      if (++iobs >= MaxDataSize::ImagePack)
      {
        LOG(WARNING) << "Max data length surpassed!";
        break;
      }
    }

    data = data_;

    return iobs;
  }

  // Encode data to stream
  int ImagePackFormator::encode(
      const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    img_t *image;
    init_img(image, data->image->width, data->image->height, data->image->step);
    image->time = gnss_common::doubleToGtime(data->image->time);
    memcpy(image->image, data->image->image,
           data->image->width * data->image->height * data->image->step);

    if (!gen_img(image))
      return 0;

    memcpy(buf, image->buff, image->nbyte);
    int nbyte = image->nbyte;
    free_img(image);

    return nbyte;
  }

  // IMU pack --------------------------------------------------
  IMUPackFormator::IMUPackFormator(Option &option)
  {
    type_ = FormatorType::IMUPack;

    init_imu(&imu_);
  }

  IMUPackFormator::IMUPackFormator(YAML::Node &node)
  {
    type_ = FormatorType::IMUPack;

    init_imu(&imu_);
  }

  IMUPackFormator::~IMUPackFormator()
  {
    free_imu(&imu_);
  }

  // Decode stream to data
  int IMUPackFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if (size == 0)
      return 0; //-sbs

    int n_data = 0;
    data.clear();
    for (int i = 0; i < size; i++)
    {
      int ret = input_imu(&imu_, buf[i]);
      if (ret <= 0)
        continue;

      std::shared_ptr<DataCluster> data_ptr;
      data_ptr = std::make_shared<DataCluster>(FormatorType::IMUPack);
      data_ptr->imu->time = gnss_common::gtimeToDouble(imu_.time);
      for (int k = 0; k < 3; k++)
      {
        data_ptr->imu->acceleration[k] = imu_.acc[k];
        data_ptr->imu->angular_velocity[k] = imu_.gyro[k];
      }
      data.push_back(data_ptr);

      if (++n_data >= MaxDataSize::IMUPack)
      {
        LOG(WARNING) << "Max data length surpassed!";
        break;
      }
    }

    return n_data;
  }

  // Encode data to stream
  int IMUPackFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    imu_t *imu;
    init_imu(imu);
    imu->time = gnss_common::doubleToGtime(data->imu->time);
    for (int i = 0; i < 3; i++)
    {
      imu->acc[i] = data->imu->acceleration[i];
      imu->gyro[i] = data->imu->angular_velocity[i];
    }

    if (!gen_imu(imu))
      return 0;

    memcpy(buf, imu->buff, imu->nbyte);
    int nbyte = imu->nbyte;
    free_imu(imu);
    return imu->nbyte;
  }

  // Option pack --------------------------------------------------
  OptionFormator::OptionFormator(Option &option)
  {
  }

  OptionFormator::OptionFormator(YAML::Node &node)
  {
  }

  OptionFormator::~OptionFormator()
  {
  }

  // Decode stream to data
  int OptionFormator::decode(const uint8_t *buf, int size,
                             std::vector<std::shared_ptr<DataCluster>> &data)
  {
    return 0;
  }

  // Encode data to stream
  int OptionFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    return 0;
  }

  // NMEA ----------------------------------------------------------
  NmeaFormator::NmeaFormator(Option &option)
  {
    type_ = FormatorType::NMEA;

    option_ = option;
  }

  NmeaFormator::NmeaFormator(YAML::Node &node)
  {
    type_ = FormatorType::NMEA;

    Option option;
    LOAD_COMMON(use_gga);
    LOAD_COMMON(use_rmc);
    LOAD_COMMON(use_esa);
    LOAD_COMMON(use_esd);
    LOAD_COMMON(talker_id);
    option_ = option;
  }

  NmeaFormator::~NmeaFormator()
  {
  }

  // Decode stream to data
  int NmeaFormator::decode(const uint8_t *buf, int size,
                           std::vector<std::shared_ptr<DataCluster>> &data)
  {
    LOG(ERROR) << "NMEA decoding not supported!";

    return 0;
  }

  // Encode data to stream
  int NmeaFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    if (data->solution == nullptr)
      return 0;

    uint8_t *p = buf;
    if (option_.use_rmc)
    {
      p += encodeRMC(*data->solution, p);
    }
    if (option_.use_gga)
    {
      p += encodeGGA(*data->solution, p);
    }
    if (option_.use_esa)
    {
      p += encodeESA(*data->solution, p);
    }
    if (option_.use_esd)
    {
      p += encodeESD(*data->solution, p);
    }

    return p - buf;
  }

#define MAXFIELD 64               /* max number of fields in a record */
#define MAXNMEA 256               /* max length of nmea sentence */
#define KNOT2M 0.514444444        /* m/knot */
  static const int nmea_solq[] = {/* NMEA GPS quality indicator */
                                  /* 0=Fix not available or invalid */
                                  /* 1=GPS SPS Mode, fix valid */
                                  /* 2=Differential GPS, SPS Mode, fix valid */
                                  /* 3=GPS PPS Mode, fix valid */
                                  /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
                                  /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
                                  /* 6=Estimated (dead reckoning) Mode */
                                  /* 7=Manual Input Mode */
                                  /* 8=Simulation Mode */
                                  SOLQ_NONE, SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP, SOLQ_FIX,
                                  SOLQ_FLOAT, SOLQ_DR, SOLQ_NONE, SOLQ_NONE, SOLQ_NONE};

  // Encode GNGGA message
  int NmeaFormator::encodeGGA(const Solution &solution, uint8_t *buf)
  {
    sol_t sol;
    convertSolution(solution, sol);

    gtime_t time;
    double h, ep[6], pos[3], dms1[3], dms2[3], dop = 1.0;
    int solq, refid = 0;
    char *p = (char *)buf, *q, sum;

    if (sol.stat <= SOLQ_NONE)
    {
      p += sprintf(p, "$%sGGA,,,,,,,,,,,,,,", option_.talker_id.data());
      for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q;
      p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
      return p - (char *)buf;
    }
    for (solq = 0; solq < 8; solq++)
      if (nmea_solq[solq] == sol.stat)
        break;
    if (solq >= 8)
      solq = 0;
    time = gpst2utc(sol.time);
    time2epoch(time, ep);
    ecef2pos(sol.rr, pos);
    h = geoidh(pos);
    deg2dms(fabs(pos[0]) * R2D, dms1, 7);
    deg2dms(fabs(pos[1]) * R2D, dms2, 7);
    p += sprintf(p, "$%sGGA,%02.0f%02.0f%06.3f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,"
                    "%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
                 option_.talker_id.data(), ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0,
                 pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",
                 solq, sol.ns, dop, pos[2] - h, h, sol.age, refid);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
      sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return p - (char *)buf;
  }

  // Encode GNRMC message
  int NmeaFormator::encodeRMC(const Solution &solution, uint8_t *buf)
  {
    sol_t sol;
    convertSolution(solution, sol);

    static double dirp = 0.0;
    gtime_t time;
    double ep[6], pos[3], enuv[3], dms1[3], dms2[3], vel, dir, amag = 0.0;
    char *p = (char *)buf, *q, sum;
    const char *emag = "E", *mode = "A", *status = "V";

    trace(3, "outnmea_rmc:\n");

    if (sol.stat <= SOLQ_NONE)
    {
      p += sprintf(p, "$%sRMC,,,,,,,,,,,,,", option_.talker_id.data());
      for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q;
      p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
      return p - (char *)buf;
    }
    time = gpst2utc(sol.time);
    time2epoch(time, ep);
    ecef2pos(sol.rr, pos);
    ecef2enu(pos, sol.rr + 3, enuv);
    vel = norm(enuv, 3);
    if (vel >= 1.0)
    {
      dir = atan2(enuv[0], enuv[1]) * R2D;
      if (dir < 0.0)
        dir += 360.0;
      dirp = dir;
    }
    else
    {
      dir = dirp;
    }
    if (sol.stat == SOLQ_DGPS || sol.stat == SOLQ_SBAS)
      mode = "D";
    else if (sol.stat == SOLQ_FLOAT || sol.stat == SOLQ_FIX)
      mode = "R";
    else if (sol.stat == SOLQ_PPP)
      mode = "P";
    deg2dms(fabs(pos[0]) * R2D, dms1, 7);
    deg2dms(fabs(pos[1]) * R2D, dms2, 7);
    p += sprintf(p, "$%sRMC,%02.0f%02.0f%06.3f,A,%02.0f%010.7f,%s,%03.0f%010.7f,"
                    "%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s,%s",
                 option_.talker_id.data(), ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0,
                 pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",
                 vel / KNOT2M, dir, ep[2], ep[1], (int)ep[0] % 100, amag, emag, mode, status);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
      sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return p - (char *)buf;
  }

  // Encode GNESA (self-defined Extended Speed and Attitude) message
  // Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
  int NmeaFormator::encodeESA(const Solution &solution, uint8_t *buf)
  {
    sol_t sol;
    convertSolution(solution, sol);
    Eigen::Vector3d rpy = quaternionToEulerAngle(solution.pose.getEigenQuaternion());
    rpy *= R2D;

    gtime_t time;
    double ep[6];
    char *p = (char *)buf, *q, sum;

    if (sol.stat <= SOLQ_NONE)
    {
      p += sprintf(p, "$%sESA,,,,,,,", option_.talker_id.data());
      for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q;
      p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
      return p - (char *)buf;
    }
    time = gpst2utc(sol.time);
    time2epoch(time, ep);

    // debug --sbs
    LOG(INFO)<< "IMU Bias: "<<solution.speed_and_bias(3) * 3600 * R2D<< "  "
    <<solution.speed_and_bias(4) * 3600 * R2D  << "  "
    <<solution.speed_and_bias(5) * 3600 * R2D  << " deg/h;  " 
    << solution.speed_and_bias(6) * 1000 / 9.84 << "  " 
    << solution.speed_and_bias(7) * 1000 / 9.84 << "  "
    << solution.speed_and_bias(8) * 1000 / 9.84 << " mg" ;

    p += sprintf(p, "$%sESA,%02.0f%02.0f%06.3f,%+.3f,%+.3f,%+.3f,"
                    "%+.3f,%+.3f,%+.3f",
                 option_.talker_id.data(), ep[3], ep[4], ep[5], sol.rr[3], sol.rr[4], sol.rr[5],
                 rpy[0], rpy[1], rpy[2]);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
      sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return p - (char *)buf;
  }

  // Encode GNESD (self-defined Extended STD) message
  // Format: $GNESD,tod,STD_Pe,STD_Pn,STD_Pu,STD_Ve,STD_Vn,STD_Vu,
  //         STD_Ar,STD_Ap,STD_Py*checksum
  int NmeaFormator::encodeESD(const Solution &solution, uint8_t *buf)
  {
    sol_t sol;
    convertSolution(solution, sol);

    gtime_t time;
    double ep[6], std_p[3], std_v[3], std_a[3];
    char *p = (char *)buf, *q, sum;

    for (size_t i = 0; i < 3; i++)
    {
      std_p[i] = sqrt(solution.covariance(i, i));
      std_v[i] = sqrt(solution.covariance(i + 6, i + 6));
      std_a[i] = sqrt(solution.covariance(i + 3, i + 3)) * R2D;
    }

    if (sol.stat <= SOLQ_NONE)
    {
      p += sprintf(p, "$%sESD,,,,,,,", option_.talker_id.data());
      for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q;
      p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
      return p - (char *)buf;
    }
    time = gpst2utc(sol.time);
    time2epoch(time, ep);
    p += sprintf(p, "$%sESD,%02.0f%02.0f%06.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                 option_.talker_id.data(), ep[3], ep[4], ep[5], std_p[0], std_p[1], std_p[2],
                 std_v[0], std_v[1], std_v[2], std_a[0], std_a[1], std_a[2]);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
      sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return p - (char *)buf;
  }

  // Convert Solution to sol_t
  void NmeaFormator::convertSolution(const Solution &solution, sol_t &sol)
  {
    sol.type = 0;
    sol.time = gnss_common::doubleToGtime(solution.timestamp);
    sol.time = utc2gpst(sol.time);
    sol.age = solution.differential_age;
    sol.ns = solution.num_satellites;
    Eigen::Map<Eigen::Vector3d> rr(sol.rr);
    rr = solution.coordinate->convert(
        solution.pose.getPosition(), GeoType::ENU, GeoType::ECEF);
    Eigen::Map<Eigen::Vector3d> vv(sol.rr + 3);
    vv = solution.speed_and_bias.head<3>();
    if (solution.status == GnssSolutionStatus::Fixed)
      sol.stat = SOLQ_FIX;
    else if (solution.status == GnssSolutionStatus::Float)
      sol.stat = SOLQ_FLOAT;
    else if (solution.status == GnssSolutionStatus::DGNSS)
      sol.stat = SOLQ_DGPS;
    else if (solution.status == GnssSolutionStatus::Single)
      sol.stat = SOLQ_SINGLE;
    else if (solution.status == GnssSolutionStatus::DeadReckoning)
      sol.stat = SOLQ_DR;
    else
      sol.stat = SOLQ_NONE;
  }

  // DCB file pack --------------------------------------------------
  DcbFileFormator::DcbFileFormator(Option &option)
  {
    type_ = FormatorType::DcbFile;
    line_.reserve(max_line_length_);
  }

  DcbFileFormator::DcbFileFormator(YAML::Node &node)
  {
    type_ = FormatorType::DcbFile;
    line_.reserve(max_line_length_);
  }

  DcbFileFormator::~DcbFileFormator()
  {
  }

  // Decode stream to data
  int DcbFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if (finished_reading_)
      return 0;

    for (int i = 0; i < size; i++)
    {
      if (buf[i] != '\n')
        line_ = line_ + (char)buf[i];
      // decode line
      else
      {
        if (passed_header_ && line_.substr(1, 3) == "DSB")
        {
          // reached the station DCB part
          if (line_.substr(15, 4) != "    ")
          {
            finished_reading_ = true;
            break;
          }

          std::string str_prn = line_.substr(11, 3);
          std::string str_obs1 = line_.substr(25, 3);
          std::string str_obs2 = line_.substr(30, 3);
          std::string str_value = line_.substr(72, 19);

          Dcb dcb;
          char system = str_prn[0];
          if (std::find(getGnssSystemList().begin(), getGnssSystemList().end(),
                        system) != getGnssSystemList().end())
          {
            dcb.code1 = gnss_common::rinexTypeToCodeType(system, str_obs1.substr(1, 2));
            dcb.code2 = gnss_common::rinexTypeToCodeType(system, str_obs2.substr(1, 2));
            dcb.value = atof(str_value.data()) * 1e-9 * CLIGHT; // ns to m
            dcbs_.insert(std::make_pair(str_prn, dcb));
          }
        }
        if (line_.substr(0, 5) == "*BIAS")
        {
          passed_header_ = true;
        }
        line_.clear();
      }
    }

    // convert to DataCluster
    if (finished_reading_)
    {
      std::shared_ptr<DataCluster> data_cluster =
          std::make_shared<DataCluster>(FormatorType::DcbFile);
      nav_t *ephemeris = data_cluster->gnss->ephemeris;

      // get all PRNs
      std::vector<std::string> prns;
      for (auto dcb : dcbs_)
      {
        if (prns.size() == 0 || prns.back() != dcb.first)
        {
          prns.push_back(dcb.first);
        }
      }
      // fill DCBs of every satellites
      for (auto prn : prns)
      {
        std::vector<int> codes;
        std::vector<Dcb> dcbs;
        for (auto dcb = dcbs_.lower_bound(prn);
             dcb != dcbs_.upper_bound(prn); dcb++)
        {
          if (std::find(codes.begin(), codes.end(), dcb->second.code1) == codes.end())
          {
            codes.push_back(dcb->second.code1);
          }
          if (std::find(codes.begin(), codes.end(), dcb->second.code2) == codes.end())
          {
            codes.push_back(dcb->second.code2);
          }
          dcbs.push_back(dcb->second);
        }

        // apply a least-square to convert DCB to code biases
        Eigen::VectorXd x = Eigen::VectorXd::Zero(codes.size());
        Eigen::VectorXd z = Eigen::VectorXd::Zero(dcbs.size() + 1);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dcbs.size() + 1, codes.size());
        for (size_t i = 0; i < dcbs.size(); i++)
        {
          z(i) = dcbs[i].value;
          for (size_t j = 0; j < codes.size(); j++)
          {
            if (codes[j] == dcbs[i].code1)
              H(i, j) = -1.0;
            if (codes[j] == dcbs[i].code2)
              H(i, j) = 1.0;
          }
        }
        // set the first code of each satellite as zero
        z(dcbs.size()) = 0.0;
        H(dcbs.size(), 0) = 1.0;
        // add a further contraint for Galileo: C1C = C1X
        if (prn[0] == 'E')
        {
          z.conservativeResize(dcbs.size() + 2);
          H.conservativeResize(dcbs.size() + 2, Eigen::NoChange);
          size_t i = dcbs.size() + 1;
          z(i) = 0.0;
          for (size_t j = 0; j < codes.size(); j++)
          {
            if (codes[j] == CODE_L1C)
              H(i, j) = 1.0;
            else if (codes[j] == CODE_L1X)
              H(i, j) = -1.0;
            else
              H(i, j) = 0.0;
          }
        }

        // Check rank
        if (checkZero((H.transpose() * H).determinant()))
        {
          LOG(INFO) << "Input DCBs are not closed for " << prn;
          continue;
        }

        // solve
        x = (H.transpose() * H).inverse() * H.transpose() * z;

        // fill ephemeris
        int sat = gnss_common::prnToSat(prn);
        for (size_t i = 0; i < codes.size(); i++)
        {
          if (sat <= 0 || codes[i] <= 0)
            continue;
          if (x(i) == 0.0)
            x(i) = 1e-4;
          ephemeris->ssr[sat - 1].cbias[codes[i] - 1] = x(i);
          ephemeris->ssr[sat - 1].update = 1;
          ephemeris->ssr[sat - 1].isdcb = 1;
        }
      }

      data_cluster->gnss->types.push_back(GnssDataType::SSR);
      data.clear();
      data.push_back(data_cluster);
      return 1;
    }

    return 0;
  }

  // Encode data to stream
  int DcbFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "DCB file encoding not supported!";

    return 0;
  }

  // OSB file pack --------------------------------------------------
  OsbFileFormator::OsbFileFormator(Option &option)
  {
    type_ = FormatorType::OsbFile;
    line_.reserve(max_line_length_);
  }

  OsbFileFormator::OsbFileFormator(YAML::Node &node)
  {
    type_ = FormatorType::OsbFile;
    line_.reserve(max_line_length_);
  }

  OsbFileFormator::~OsbFileFormator()
  {
  }

  // Decode stream to data
  int OsbFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if (finished_reading_) return 0;

    for (int i = 0; i < size; i++) {
      if (buf[i] != '\n') line_ = line_ + (char)buf[i];
      // decode line
      else {
        // reached the end of the PRIDE osbfile
        if ( line_.substr(0, 8) == "%=ENDBIA") {
            finished_reading_ = true;
            break;
        }        
        if (passed_header_ && line_.substr(1, 3) == "OSB") {
          //  OSB  SVNX C12           L7        2021:357:59520 2021:357:59550 ns                 -2.0958      0.0000
          if(line_.substr(6,4)=="SVNX"){
            std::string str_prn = line_.substr(11, 3);
            std::string str_phase = line_.substr(25, 2);
            std::string str_value = line_.substr(72, 19);

            std::string str_time = line_.substr(35, 14);
            int year = atoi(str_time.substr(0, 4).data());
            int doy = atoi(str_time.substr(5, 3).data());
            int sod = atoi(str_time.substr(9, 5).data());
            gtime_t t_begin;
            if(!doy2time(year,  doy, sod, &t_begin)){
              LOG(ERROR) << "Failed to convert day of year (start) to gtime_t!";
              continue;
            }
            str_time = line_.substr(50, 14);
            year = atoi(str_time.substr(0, 4).data());
            doy = atoi(str_time.substr(5, 3).data());
            sod = atoi(str_time.substr(9, 5).data());
            gtime_t t_end;
            if(!doy2time(year,  doy, sod, &t_end)){
              LOG(ERROR) << "Failed to convert day of year (end) to gtime_t!";
              continue;
            }
            double timestamp  = gnss_common::gtimeToDouble(t_begin);

            zpb_t zpb;
            char system = str_prn[0];
            if (std::find(getGnssSystemList().begin(), getGnssSystemList().end(),
                system) != getGnssSystemList().end()) {
              zpb.phase = gnss_common::phaseStringToPhaseType(system, str_phase); 
              zpb.value = atof(str_value.data()) * 1e-9 * CLIGHT; // ns to m
              if(osbs_.zpbs_map.find(timestamp) == osbs_.zpbs_map.end()){
                osbs_.zpbs_map.insert(std::make_pair(timestamp, zpb_epoch_t()));
              }
              osbs_.zpbs_map[timestamp].push_back(std::make_pair(str_prn, zpb));
            }
          }
          else{
            std::string str_prn = line_.substr(11, 3);
            std::string str_code = line_.substr(25, 3);
            std::string str_value = line_.substr(72, 19);

            zcb_t zcb;
            char system = str_prn[0];
            if (std::find(getGnssSystemList().begin(), getGnssSystemList().end(),
                system) != getGnssSystemList().end()) {
              zcb.code = gnss_common::rinexTypeToCodeType(system, str_code.substr(1, 2));
              zcb.value = atof(str_value.data()) * 1e-9 * CLIGHT;  // ns to m
              osbs_.zcbs.push_back(std::make_pair(str_prn, zcb));
            }
          }
        }
        if (line_.substr(0, 5) == "*BIAS") {
          passed_header_ = true;
        }
        line_.clear();
      }
    }
    // convert to DataCluster
    if (finished_reading_) {
      std::shared_ptr<DataCluster> data_cluster =
        std::make_shared<DataCluster>(FormatorType::OsbFile);
      data_cluster->gnss->obs_bias = osbs_;
      data_cluster->gnss->types.push_back(GnssDataType::ObservationBias);
      data.clear();
      data.push_back(data_cluster);
      return 1;
    }
    
    return 0;
  }

  // Encode data to stream
  int OsbFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Osb file encoding not supported!";

    return 0;
  }

  // RINEX-O File --------------------------------------------------
  RnxOFileFormator::RnxOFileFormator(Option &option)
  {
    type_ = FormatorType::RnxOFile;

    option_ = option;
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of obs per file
    obs = new obs_t;
    obsd_t data0 = {{0}};
    obs->data = new obsd_t[MaxDataSize::MaxObsEpoch];
    obs->n = 0;
    obs->nmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      obs->data[i] = data0;
  }

  RnxOFileFormator::RnxOFileFormator(YAML::Node &node)
  {
    type_ = FormatorType::RnxOFile;

    // default path
    option_.filepath = "./obs/";
    option_.start_time = {0};

    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of obs per file
    obs = new obs_t;
    obsd_t data0 = {{0}};
    obs->data = new obsd_t[MaxDataSize::MaxObsEpoch];
    obs->n = 0;
    obs->nmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      obs->data[i] = data0;
  }

  RnxOFileFormator::~RnxOFileFormator()
  {
    delete[] obs->data;
    delete obs;
  }

  // Decode stream to data
  int RnxOFileFormator::decode(const uint8_t *buf, int size,
                               std::vector<std::shared_ptr<DataCluster>> &data)
  {


    if(obs->n > 0 && index_ >= obs->n ){
      // std::cout<<"obs->n: "<<obs->n<<"; index_: "<<index_<<std::endl;
      return -3;
    }

    int iobs = 0; // return epoch cnt

    if (!finished_filelist_)
    {
      std::stringstream ss;
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of rnx-o file
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        char type;
        gtime_t t = {0};
        // read each rinex file
        for (auto file : files_)
        {
          //\ function params: path,          t_start,    t_end,    t_int,
          //\                 option,        "1" for clk/ "0" for others,
          //\                 station_index, rnx_type,   obs_t,    nav_t,    sta_t
          int ret = readrnxfile((option_.filepath + file).c_str(), t, t, 0,
                                "", 0,
                                0, &type, obs, NULL, NULL);
          if (ret <= 0)
          {
            LOG(WARNING) << "Failed to read Rnx-o-file: " << file;
            continue;
          }
        }
        sortobs(obs);

        // for(int i=0;i<obs->n;i++)
        //   LOG(INFO)<<"obs time: "<<obs->data[i].time.time <<"; sat:"<<obs->data[i].sat;
        // index = find_start(obs);
        finished_reading_ = true;
      }
      else
      {
        // initialization of DataCluster
        std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
        for (size_t i = 0; i < data_.size(); i++)
        {
          data_[i]->gnss->types.clear();
          gnss_data.push_back(data_[i]->gnss);
        }

        for (; iobs < dump_epoch_; iobs++)
        {

          obs_t *obs_epoch = new obs_t;
          obs_epoch->n = 0;
          obs_epoch->data = new obsd_t[MAXSAT];
          option_.start_time = obs->data[index_].time;

          double start = time2gpst(option_.start_time, NULL);
          double epoch = time2gpst(obs->data[index_].time, NULL);

          while (index_ < obs->n && epoch <= start)
          {
            memcpy(&obs_epoch->data[obs_epoch->n++], &obs->data[index_++], sizeof(obsd_t) * 1);
            epoch = time2gpst(obs->data[index_].time, NULL);
          }
          // memcpy(obs_epoch->data,&obs->data[index_++],sizeof(obsd_t)*1);

          // LOG(INFO)<<obs->n<<"; time:"<<obs->data[0].time.time;
          // function params: int ret,      obs_t *obs,     nav_t *nav,     sta_t *sta,    ssr_t *ssr,
          //                  int iobs,     int input satellite number,
          //                  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data
          gnss_common::updateStreamData(1, obs_epoch, NULL, NULL, NULL,
                                        iobs, 0, gnss_data);
          // store type to DataCluster
          GnssDataType type = GnssDataType::Observation;
          std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[iobs];
          gnss->types.push_back(type);

          if (iobs >= MaxDataSize::GnssRaw)
          {
            LOG(WARNING) << "Max GnssRaw number surpassed!";
            break;
          }

          delete[] obs_epoch->data;
          delete obs_epoch;
        }
      }
    }

    data = data_;

    return iobs;
  }

  // Encode data to stream
  int RnxOFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Rinex O file encoding not supported!";

    return 0;
  }




  // RINEX-N File --------------------------------------------------
  RnxNFileFormator::RnxNFileFormator(Option &option)
  {

    type_ = FormatorType::RnxNFile;

    option_ = option;
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    index_glonass_ = 0;

    dump_epoch_ = 1;

    // initilization of brocast ephemeris local storage
    nav = new nav_t;
    eph_t data0 = {0};
    geph_t data0_g = {0};
    nav->eph = new eph_t[MaxDataSize::MaxObsEpoch];
    nav->geph = new geph_t[MaxDataSize::MaxObsEpoch];
    nav->n = 0; nav->ng = 0;
    nav->nmax = MaxDataSize::MaxObsEpoch; nav->ngmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++){
      nav->eph[i] = data0;
      nav->geph[i] = data0_g;
    }


  }

  RnxNFileFormator::RnxNFileFormator(YAML::Node &node)
  {

    type_ = FormatorType::RnxNFile;

    // default path
    option_.filepath = "./product/";
    option_.start_time = {0};

    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    index_glonass_ = 0;
    dump_epoch_ = 1;

   // initilization of brocast ephemeris local storage
    nav = new nav_t;
    eph_t data0 = {0};
    geph_t data0_g = {0};
    nav->eph = new eph_t[MaxDataSize::MaxObsEpoch];
    nav->geph = new geph_t[MaxDataSize::MaxObsEpoch];
    nav->n = 0; nav->ng = 0;
    nav->nmax = MaxDataSize::MaxObsEpoch; nav->ngmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++){
      nav->eph[i] = data0;
      nav->geph[i] = data0_g;
    }
  }

  RnxNFileFormator::~RnxNFileFormator()
  {
    delete[] nav->eph;
    delete[] nav->geph;
    delete nav;
  }

  // Decode stream to data
  int RnxNFileFormator::decode(const uint8_t *buf, int size,
                               std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if(nav->n > 0)
    { 
      if(index_ < nav->n  && index_glonass_ < nav->ng){}
      else
      {
        return -3;
      }
    }

    int iobs = 0; // return epoch cnt

    if (!finished_filelist_)
    {
      std::stringstream ss;
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of rnx-o file
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        char type;
        gtime_t t = {0};
        // read each rinex file
        for (auto file : files_)
        {
          //\ function params: path,          t_start,    t_end,    t_int,
          //\                 option,        "1" for clk/ "0" for others,
          //\                 station_index, rnx_type,   obs_t,    nav_t,    sta_t
          int ret = readrnxfile((option_.filepath + file).c_str(), t, t, 0,
                                "", 0,
                                0, &type, NULL, nav, NULL);
          if (ret <= 0)
          {
            LOG(WARNING) << "Failed to read Rnx-N-File: " << file;
            continue;
          }
          finished_reading_ = true;
          uniqnav(nav);
        }

      }
      else
      {
        // LOG(INFO)<<"nav->n: "<<nav->n<<"; nav->ng: "<<nav->ng;
        // for(int i=0;i<nav->n;i++){
        //   char prn[4];
        //   satno2id(nav->eph[i].sat,prn);
        //   LOG(INFO)<<"nav time: "<<nav->eph[i].toe.time <<"; sat:"<<prn;

        // }

        // initialization of DataCluster
        std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
        for (size_t i = 0; i < data_.size(); i++)
        {
          data_[i]->gnss->types.clear();
          gnss_data.push_back(data_[i]->gnss);
        }

        for (; iobs < dump_epoch_; iobs++)
        {

          nav_t *nav_epoch = new nav_t;
          nav_epoch->n = 0;
          nav_epoch->eph = new eph_t[MAXSAT*20];
          nav_epoch->ng = 0;
          nav_epoch->geph = new geph_t[MAXSAT*20];


          // for G/E/C/J systems
          option_.start_time = nav->eph[index_].toc;
          double start = time2gpst(option_.start_time, NULL);
          double epoch = time2gpst(nav->eph[index_].toc, NULL);

          while (index_ < nav->n && epoch <= start + 7200.0)
          {
            memcpy(&nav_epoch->eph[nav_epoch->n++], &nav->eph[index_++], sizeof(eph_t) * 1);
            if(index_ < nav->n)
              epoch = time2gpst(nav->eph[index_].toc, NULL);
          }

          // for R system
          // option_.start_time = nav->geph[index_glonass_].tof;
          // start = time2gpst(option_.start_time, NULL);
          epoch = time2gpst(nav->geph[index_glonass_].toe, NULL);

          while (index_glonass_ < nav->ng && epoch <= start + 7200.0 )
          {
            memcpy(&nav_epoch->geph[nav_epoch->ng++], &nav->geph[index_glonass_++], sizeof(geph_t) * 1);
            if (index_glonass_ < nav->ng)
              epoch = time2gpst(nav->geph[index_glonass_].toe, NULL);
          }

          // function params: int ret,      obs_t *obs,     nav_t *nav,     sta_t *sta,    ssr_t *ssr,
          //                  int iobs,     int input satellite number,
          //                  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data
          // gnss_common::updateStreamData(2, NULL, nav_epoch, NULL, NULL,
          //                               0, 0, gnss_data);



          memcpy(gnss_data[0]->ephemeris->eph, 
              nav_epoch->eph, sizeof(eph_t) * 20 * MAXSAT);
          memcpy(gnss_data[0]->ephemeris->geph, 
              nav_epoch->geph, sizeof(geph_t) * 20 * NSATGLO);


          // store type to DataCluster
          GnssDataType type = GnssDataType::Ephemeris;
          std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[iobs];
          gnss->types.push_back(type);

          if (iobs >= MaxDataSize::GnssRaw)
          {
            LOG(WARNING) << "Max Ephemeris number surpassed!";
            break;
          }

          delete[] nav_epoch->eph;
          delete[] nav_epoch->geph;
          delete nav_epoch;
        }
      }
    }

    data = data_;

    return iobs;
  }

  // Encode data to stream
  int RnxNFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Rinex N file encoding not supported!";

    return 0;
  }



  // Sp3 pack --------------------------------------------------
  Sp3FileFormator::Sp3FileFormator(Option &option)
  {
    type_ = FormatorType::Sp3File;

    option_ = option;
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of sp3 local storage
    nav = new nav_t;
    peph_t data0 = {{0}};
    nav->peph = new peph_t[MaxDataSize::MaxObsEpoch];
    nav->ne = 0;
    nav->nemax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      nav->peph[i] = data0;
  }

  Sp3FileFormator::Sp3FileFormator(YAML::Node &node)
  {
    type_ = FormatorType::Sp3File;

    // default path
    option_.filepath = "./product/";
    option_.start_time = {0};

    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of sp3 local storage
    nav = new nav_t;
    peph_t data0 = {{0}};
    nav->peph = new peph_t[MaxDataSize::MaxObsEpoch];
    nav->ne = 0;
    nav->nemax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      nav->peph[i] = data0;
  }

  Sp3FileFormator::~Sp3FileFormator()
  {
    delete[] nav->peph;
    delete nav;
  }

  // Decode stream to data
  int Sp3FileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if(finished_reading_){
      return 0;
    }

    int iobs = 0; // return epoch cnt

    if (!finished_filelist_)
    {
      std::stringstream ss;
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of sp3 filelist
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        // read each rinex file
        for (auto file : files_)
        {
          //\ function params: path, nav_t, opt
          int old_ne = nav->ne;
          readsp3((option_.filepath + file).c_str(), nav, 0);
          if (nav->ne <= old_ne)
          {
            LOG(INFO) << "Sp3 file read error: " << file;
            continue;
          }
          finished_reading_ = true; //(TODO:--sbs) change the logic for more files
        }


        std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
        for (size_t i = 0; i < data_.size(); i++)
        {
          data_[i]->gnss->types.clear();
          gnss_data.push_back(data_[i]->gnss);
        }
        gnss_common::updateStreamData(12, NULL, nav, NULL, NULL,
                                        0, 0, gnss_data);
        GnssDataType type = GnssDataType::PreciseEph;
        std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[0];
        gnss->types.push_back(type);

      }
      // else
      // {
      //   // initialization of DataCluster
      //   std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
      //   for (size_t i = 0; i < data_.size(); i++)
      //   {
      //     data_[i]->gnss->types.clear();
      //     gnss_data.push_back(data_[i]->gnss);
      //   }

      //   for (; iobs < dump_epoch_; iobs++)
      //   {

      //     nav_t *nav_epoch = new nav_t;
      //     // nav_epoch->n = 0;
      //     nav_epoch->peph = new peph_t[MAXSAT];
      //     nav_epoch->ne = 0;
      //     option_.start_time = nav->peph[index_].time;

      //     double start = time2gpst(option_.start_time, NULL);
      //     double epoch = time2gpst(nav->peph[index_].time, NULL);

      //     while (index_ < nav->ne && epoch <= start)
      //     {
      //       memcpy(&nav_epoch->peph[nav_epoch->ne++], &nav->peph[index_++], sizeof(peph_t) * 1);
      //       epoch = time2gpst(nav->peph[index_].time, NULL);
      //     }

      //     // function params: int ret,      obs_t *obs,     nav_t *nav,     sta_t *sta,    ssr_t *ssr,
      //     //                  int iobs,     int input satellite number,
      //     //                  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data
      //     gnss_common::updateStreamData(12, NULL, nav_epoch, NULL, NULL,
      //                                   0, 0, gnss_data);
      //     // store type to DataCluster
      //     GnssDataType type = GnssDataType::PreciseEph;
      //     std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[iobs];
      //     gnss->types.push_back(type);

      //     if (iobs >= MaxDataSize::GnssRaw)
      //     {
      //       LOG(WARNING) << "Max GnssRaw number surpassed!";
      //       break;
      //     }

      //     delete[] nav_epoch->peph;
      //     delete nav_epoch;
      //   }
      // }
    }

    data = data_;

    return 1;
  }

  // Encode data to stream
  int Sp3FileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Sp3 file encoding not supported!";

    return 0;
  }

  // Clk pack --------------------------------------------------
  ClkFileFormator::ClkFileFormator(Option &option)
  {
    type_ = FormatorType::ClkFile;

    option_ = option;
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of sp3 local storage
    nav = new nav_t;
    pclk_t data0 = {{0}};
    nav->pclk = new pclk_t[MaxDataSize::MaxObsEpoch];
    nav->nc = 0;
    nav->ncmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      nav->pclk[i] = data0;
  }

  ClkFileFormator::ClkFileFormator(YAML::Node &node)
  {
    type_ = FormatorType::ClkFile;

    // default path
    option_.filepath = "./product/";
    option_.start_time = {0};

    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

    for (int i = 0; i < MaxDataSize::GnssRaw; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(type_));
    }

    index_ = 0;
    dump_epoch_ = 1;

    // initilization of sp3 local storage
    nav = new nav_t;
    pclk_t data0 = {{0}};
    nav->pclk = new pclk_t[MaxDataSize::MaxObsEpoch];
    nav->nc = 0;
    nav->ncmax = MaxDataSize::MaxObsEpoch;
    for (int i = 0; i < MaxDataSize::MaxObsEpoch; i++)
      nav->pclk[i] = data0;
  }

  ClkFileFormator::~ClkFileFormator()
  {
    delete[] nav->pclk;
    delete nav;
  }

  // Decode stream to data
  int ClkFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if(finished_reading_){
      return 0;
    }

    int iobs = 0; // return epoch cnt

    if (!finished_filelist_)
    {
      std::stringstream ss;
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of sp3 filelist
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        char typ;
        gtime_t t = {0};
        // read each rinex file
        for (auto file : files_)
        {
          //\ function params: path,          t_start,    t_end,    t_int,
          //\                 option,        "1" for clk/ "0" for others,
          //\                 station_index, rnx_type,   obs_t,    nav_t,    sta_t
          int ret = readrnxfile((option_.filepath + file).c_str(), t, t, 0,
                                "", 1,
                                0, &typ, NULL, nav, NULL);
          if (ret <= 0)
          {
            LOG(WARNING) << "Failed to read Precise Clock File: " << file;
            continue;
          }
          finished_reading_ = true;

        }

        std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
        for (size_t i = 0; i < data_.size(); i++)
        {
          data_[i]->gnss->types.clear();
          gnss_data.push_back(data_[i]->gnss);
        }
        gnss_common::updateStreamData(13, NULL, nav, NULL, NULL,
                                        0, 0, gnss_data);
        GnssDataType type = GnssDataType::PreciseClk;
        std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[0];
        gnss->types.push_back(type);


      }
      // else
      // {
      //   // initialization of DataCluster
      //   std::vector<std::shared_ptr<DataCluster::GNSS>> gnss_data;
      //   for (size_t i = 0; i < data_.size(); i++)
      //   {
      //     data_[i]->gnss->types.clear();
      //     gnss_data.push_back(data_[i]->gnss);
      //   }

      //   for (; iobs < dump_epoch_; iobs++)
      //   {

      //     nav_t *nav_epoch = new nav_t;
      //     nav_epoch->nc = 0;
      //     nav_epoch->pclk = new pclk_t[MAXSAT];
      //     option_.start_time = nav->pclk[index_].time;

      //     double start = time2gpst(option_.start_time, NULL);
      //     double epoch = time2gpst(nav->pclk[index_].time, NULL);

      //     while (index_ < nav->nc && epoch <= start)
      //     {
      //       memcpy(&nav_epoch->pclk[nav_epoch->nc++], &nav->pclk[index_++], sizeof(pclk_t) * 1);
      //       epoch = time2gpst(nav->pclk[index_].time, NULL);
      //     }

      //     // function params: int ret,      obs_t *obs,     nav_t *nav,     sta_t *sta,    ssr_t *ssr,
      //     //                  int iobs,     int input satellite number,
      //     //                  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data
      //     gnss_common::updateStreamData(13, NULL, nav_epoch, NULL, NULL,
      //                                   0, 0, gnss_data);
      //     // store type to DataCluster
      //     GnssDataType type = GnssDataType::PreciseClk;
      //     std::shared_ptr<DataCluster::GNSS> &gnss = gnss_data[iobs];
      //     gnss->types.push_back(type);

      //     if (iobs >= MaxDataSize::GnssRaw)
      //     {
      //       LOG(WARNING) << "Max GnssRaw number surpassed!";
      //       break;
      //     }

      //     delete[] nav_epoch->pclk;
      //     delete nav_epoch;
      //   }
      // }
    }

    data = data_;

    return 1;
  }

  // Encode data to stream
  int ClkFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "Satellite precise Clk file encoding not supported!";

    return 0;
  }

  // ATX file pack --------------------------------------------------
  AtxFileFormator::AtxFileFormator(Option &option)
  {
    type_ = FormatorType::AtxFile;
    line_.reserve(max_line_length_);
    if (!(pcvs_ = (pcvs_t *)malloc(sizeof(pcvs_t))))
    {
      free(pcvs_);
      return;
    }
    memset(pcvs_, 0, sizeof(pcvs_t));
  }

  AtxFileFormator::AtxFileFormator(YAML::Node &node)
  {
    type_ = FormatorType::AtxFile;
    line_.reserve(max_line_length_);
    if (!(pcvs_ = (pcvs_t *)malloc(sizeof(pcvs_t))))
    {
      free(pcvs_);
      return;
    }
    memset(pcvs_, 0, sizeof(pcvs_t));
  }

  AtxFileFormator::~AtxFileFormator()
  {
    free(pcvs_);
  }

  // Decode stream to data
  int AtxFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if (size == 0)
      return 0; //-sbs

    const pcv_t pcv0 = {0};
    double neu[3];
    int i, f, freq = 0, freqs[] = {1, 2, 5, 0};

    for (int k = 0; k < size; k++)
    {
      if (buf[k] != '\n')
        line_ = line_ + (char)buf[k];
      // decode line
      else
      {
        const char *buff = line_.data();
        if (strlen(buff) < 60 || strstr(buff + 60, "COMMENT"))
        {
          line_.clear();
          continue;
        }

        if (strstr(buff + 60, "START OF ANTENNA"))
        {
          pcv_ = pcv0;
          state_ = 1;
        }
        if (strstr(buff + 60, "END OF ANTENNA"))
        {
          addpcv(&pcv_, pcvs_);
          state_ = 0;
        }
        if (!state_)
        {
          line_.clear();
          continue;
        }

        if (strstr(buff + 60, "TYPE / SERIAL NO"))
        {
          strncpy(pcv_.type, buff, 20);
          pcv_.type[20] = '\0';
          strncpy(pcv_.code, buff + 20, 20);
          pcv_.code[20] = '\0';
          if (!strncmp(pcv_.code + 3, "        ", 8))
          {
            pcv_.sat = satid2no(pcv_.code);
          }
        }
        else if (strstr(buff + 60, "VALID FROM"))
        {
          if (!str2time(buff, 0, 43, &pcv_.ts))
          {
            line_.clear();
            continue;
          }
        }
        else if (strstr(buff + 60, "VALID UNTIL"))
        {
          if (!str2time(buff, 0, 43, &pcv_.te))
          {
            line_.clear();
            continue;
          }
        }
        else if (strstr(buff + 60, "START OF FREQUENCY"))
        {
          if (!pcv_.sat && buff[3] != 'G')
          {
            line_.clear();
            continue;
          } /* only read rec ant for GPS */
          if (sscanf(buff + 4, "%d", &f) < 1)
          {
            line_.clear();
            continue;
          }
          for (i = 0; freqs[i]; i++)
            if (freqs[i] == f)
              break;
          if (freqs[i])
            freq = i + 1;
        }
        else if (strstr(buff + 60, "END OF FREQUENCY"))
        {
          freq = 0;
        }
        else if (strstr(buff + 60, "NORTH / EAST / UP"))
        {
          if (freq < 1 || NFREQ < freq)
          {
            line_.clear();
            continue;
          }
          if (decodef((char *)buff, 3, neu) < 3)
          {
            line_.clear();
            continue;
          }
          pcv_.off[freq - 1][0] = neu[pcv_.sat ? 0 : 1]; /* x or e */
          pcv_.off[freq - 1][1] = neu[pcv_.sat ? 1 : 0]; /* y or n */
          pcv_.off[freq - 1][2] = neu[2];                /* z or u */
        }
        else if (strstr(buff, "NOAZI"))
        {
          if (freq < 1 || NFREQ < freq)
          {
            line_.clear();
            continue;
          }
          if ((i = decodef((char *)(buff + 8), 19, pcv_.var[freq - 1])) <= 0)
          {
            line_.clear();
            continue;
          }
          for (; i < 19; i++)
            pcv_.var[freq - 1][i] = pcv_.var[freq - 1][i - 1];
        }
        line_.clear();
      }
    }

    // convert to DataCluster
    if (last_size_ != 0 && last_size_ > size)
    {
      std::shared_ptr<DataCluster> data_cluster =
          std::make_shared<DataCluster>(FormatorType::AtxFile);
      nav_t *ephemeris = data_cluster->gnss->ephemeris;

      pcv_t *pcv;
      for (i = 0; i < MAXSAT; i++)
      {
        pcv = searchpcv(i + 1, "", timeget(), pcvs_);
        ephemeris->pcvs[i] = pcv ? *pcv : pcv0;
      }
      free(pcvs_->pcv);

      data_cluster->gnss->types.push_back(GnssDataType::PhaseCenter);
      data.clear();
      data.push_back(data_cluster);
      return 1;
    }
    last_size_ = size;

    return 0;
  }

  // Encode data to stream
  int AtxFileFormator::encode(const std::shared_ptr<DataCluster> &data, uint8_t *buf)
  {
    LOG(ERROR) << "ATX file encoding not supported!";

    return 0;
  }

  // Add antenna parameter
  void AtxFileFormator::addpcv(const pcv_t *pcv, pcvs_t *pcvs)
  {
    pcv_t *pcvs_pcv;

    if (pcvs->nmax <= pcvs->n)
    {
      pcvs->nmax += 256;
      if (!(pcvs_pcv = (pcv_t *)realloc(pcvs->pcv, sizeof(pcv_t) * pcvs->nmax)))
      {
        free(pcvs->pcv);
        pcvs->pcv = NULL;
        pcvs->n = pcvs->nmax = 0;
        return;
      }
      pcvs->pcv = pcvs_pcv;
    }
    pcvs->pcv[pcvs->n++] = *pcv;
  }

  // Decode antenna parameter field
  int AtxFileFormator::decodef(char *p, int n, double *v)
  {
    int i;

    for (i = 0; i < n; i++)
      v[i] = 0.0;
    for (i = 0, p = strtok(p, " "); p && i < n; p = strtok(NULL, " "))
    {
      v[i++] = atof(p) * 1E-3;
    }
    return i;
  }


  // Ionosphere file pack --------------------------------------------------
  IonFileFormator::IonFileFormator(Option &option)
  {
    type_ = FormatorType::IonFile;
    line_.reserve(max_line_length_);

    option_ = option;
  }
  IonFileFormator::IonFileFormator(YAML::Node &node)
  {

    // default path
    option_.filepath = "./ion/";
    // load path from yaml
    loadPath(node, option_.filepath);

    type_ = FormatorType::IonFile;
    line_.reserve(max_line_length_);
  }
  IonFileFormator::~IonFileFormator()
  {
  }

  int IonFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
  {
    if(finished_reading_) return 0;

    for(int i=0;i<size;i++){
      if(buf[i]!='\n') line_=line_ + (char)buf[i];
      else{
        // end of ion_list file
        if(line_.substr(0,5) == "*AUGS"){
          finished_reading_ = true;
          break;
        }

        // hkks -2429526.6039   5377816.3872   2412152.4809
        std::string str_station = line_.substr(0,4);
        std::string str_position_x = line_.substr(5, 13);
        std::string str_position_y = line_.substr(20, 13);
        std::string str_position_z = line_.substr(35, 13);
        Eigen::Vector3d position={
          atof(str_position_x.data()),
          atof(str_position_y.data()),
          atof(str_position_z.data())
        };
        station_t station;
        station.name = str_station;
        station.position = position;

        stations_.push_back(station);
        line_.clear();
      }
    }

    if(finished_reading_){

      std::shared_ptr<DataCluster> data_cluster =
        std::make_shared<DataCluster>(FormatorType::IonFile);
      //read ionosphere file
      for(auto station : stations_){
        input_ionosphere_file(station);
      }
      data_cluster->gnss->ion_network = ionospheric_network_;

      data_cluster->gnss->types.push_back(GnssDataType::IonNetwork);
      data.clear();
      data.push_back(data_cluster);

      return 1;  // 1 observation in total (since we save all ionosphere delay in one ionnetworks)
    }

    return 0;
  }

  int IonFileFormator::input_ionosphere_file(const station_t station)
  {
    // station name and position
    ions_t ions;
    ions.clear();

    // open ionosphere file
    std::string line;
    std::stringstream ss;
    std::ifstream ion_file;
    ion_file.open( option_.filepath +"iondat_"+station.name, std::ios::in);
    if(!ion_file.is_open()){
      LOG(ERROR) << "ionosphere file open failed!";
      return 0;
    }

    // read ionosphere file from one station
    //julian_day   sod  PRN     ion_delay amb_fixed_flag delay_std   no_use
    //  59571      0.00 G05       1.069155     0       0.094873   29.308
    while(!ion_file.eof())
    {
      int modified_julian_day,flag;  
      double second_of_day, ion_delay, ion_delay_std, no_use;
      std::string PRN;
      
      // read one line
      getline(ion_file, line);
      ss.clear();
      ss.str(line);
      ss>>modified_julian_day>>second_of_day>>PRN>>ion_delay>>flag>>ion_delay_std>>no_use;
      // convert time from modified julian day to gpstime
      // double time = MJD2GPST(modified_julian_day, second_of_day);
      double timestamp = second_of_day;  //test --sbs

      ion_t ion;
      ion.prn = PRN;
      ion.value = ion_delay;
      ion.std = ion_delay_std;

      if(ions.find(timestamp) == ions.end()){
        ions.insert(std::make_pair(timestamp, std::vector<ion_t>()));
      }
      ions[timestamp].push_back(ion);
    }

    // add ionosphere from one station to ionnetwork
    ionospheric_network_.push_back(std::make_pair(station, ions));

    return 1;
  }

  int IonFileFormator::encode(const std::shared_ptr<DataCluster> &data,
                              uint8_t *buf)
  {
  }


  // Ztd file pack --------------------------------------------------
  ZtdFileFormator::ZtdFileFormator(Option &option)
  {
    type_ = FormatorType::ZtdFile;
    line_.reserve(max_line_length_);
    option_ = option;
  }
  ZtdFileFormator::ZtdFileFormator(YAML::Node &node)
  {
    // default path
    option_.filepath = "./ztd/";
    // load path from yaml
    loadPath(node, option_.filepath);

    type_ = FormatorType::ZtdFile;
    line_.reserve(max_line_length_);
  }
  ZtdFileFormator::~ZtdFileFormator()
  {
  }

  int ZtdFileFormator::decode(const uint8_t *buf, int size,
                              std::vector<std::shared_ptr<DataCluster>> &data)
 {
    if(finished_reading_) return 0;

    for(int i=0;i<size;i++){
      if(buf[i]!='\n') line_=line_ + (char)buf[i];
      else{
        // end of ion_list file
        if(line_.substr(0,5) == "*AUGS"){
          finished_reading_ = true;
          break;
        }

        // hkks -2429526.6039   5377816.3872   2412152.4809
        std::string str_station = line_.substr(0,4);
        std::string str_position_x = line_.substr(5, 13);
        std::string str_position_y = line_.substr(20, 13);
        std::string str_position_z = line_.substr(35, 13);
        Eigen::Vector3d position={
          atof(str_position_x.data()),
          atof(str_position_y.data()),
          atof(str_position_z.data())
        };
        station_t station;
        station.name = str_station;
        station.position = position;
        stations_.push_back(station);
        line_.clear();
      }
    }

    if(finished_reading_){

      std::shared_ptr<DataCluster> data_cluster =
        std::make_shared<DataCluster>(FormatorType::ZtdFile);
      //read ionosphere file
      for(auto station : stations_){
        input_tropospheric_file(station);
      }
      data_cluster->gnss->ztd_network = tropospheric_network_;

      data_cluster->gnss->types.push_back(GnssDataType::ZtdNetwork);
      data.clear();
      data.push_back(data_cluster);

      return 1;  // 1 observation in total (since we save all ionosphere delay in one ionnetworks)
    }

    return 0;
  }

  int ZtdFileFormator::input_tropospheric_file(const station_t station)
  {
    // station name and position
    ztds_t ztds;
    ztds.clear();

    // open tropospheric file
    std::string line;
    std::stringstream ss;
    std::ifstream ion_file;
    ion_file.open( option_.filepath +"ztddat_"+station.name, std::ios::in);
    if(!ion_file.is_open()){
      LOG(ERROR) << "ionosphere file open failed!";
      return 0;
    }

    // read tropospheric file from one station
    //julian_day   sod    delay_part1 delay_part2 delay_std   
    //  59571    30.000   2.308173   0.109190  -0.000344
    while(!ion_file.eof())
    {
      int modified_julian_day;  
      double second_of_day, delay_part1,delay_part2 , delay_std;
      
      // read one line
      getline(ion_file, line);
      ss.clear();
      ss.str(line);
      ss>>modified_julian_day>>second_of_day>>delay_part1>>delay_part2>>delay_std;
      // convert time from modified julian day to gpstime
      // double time = MJD2GPST(modified_julian_day, second_of_day);
      double timestamp = second_of_day;  //test --sbs

      ztd_t ztd;
      ztd.value = delay_part1 + delay_part2;
      ztd.std = delay_std;

      if(ztds.find(timestamp) == ztds.end()){
        ztds.insert(std::make_pair(timestamp, std::vector<ztd_t>()));
      }
      ztds[timestamp].push_back(ztd);
    }

    // add ionosphere from one station to ionnetwork
    tropospheric_network_.push_back(std::make_pair(station, ztds));

    return 1;
  }

  
  int ZtdFileFormator::encode(const std::shared_ptr<DataCluster> &data,
                              uint8_t *buf)
  {
  }

  IMUImrFormator::IMUImrFormator(Option &option)
  {
    type_ = FormatorType::IMUImr;
    index_ = 0;
    option_ = option;
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

  }
  IMUImrFormator::IMUImrFormator(YAML::Node &node)
  {

    type_ = FormatorType::IMUImr;
    index_ = 0;
    // default path
    option_.filepath = "./imu/";
    option_.start_time = {0};
    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);

    line_.reserve(max_line_length_);

  }
  IMUImrFormator::~IMUImrFormator()
  {
  }

  int IMUImrFormator::decode(const uint8_t *buf, int size,
                             std::vector<std::shared_ptr<DataCluster>> &data)
  {
    // return -3;
    // if(finished_reading_)
    if(imus_.size() > 0 && index_ >= imus_.size() )
    {
      return -3;
    }

    

    int n_data = 0;
    std::stringstream ss;
    data.clear();


    if (!finished_filelist_)
    {
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of rnx-o file
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        char type;
        gtime_t t = {0};

        // read each rinex file
        for (auto file : files_)
        {

          std::ifstream imu_file(option_.filepath + file);
          std::string line;
          while (std::getline(imu_file, line))
          {
            if(line[0]=="#" || line[0]=="%") continue;

            ss.clear();
            ss.str(line);

            // read 6-axis imu data
            imu_t imu;
            ss >>  imu.time.sec  
               >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2] 
               >> imu.acc[0]  >> imu.acc[1]  >> imu.acc[2] ;

            imu.gyro[0]*=D2R;
            imu.gyro[1]*=D2R;
            imu.gyro[2]*=D2R;

            imu.time.time = int(imu.time.sec);
            imu.time.sec = imu.time.sec - imu.time.time;

            // add imu data to imu_
            imus_.push_back(imu);

          }
        }

        finished_reading_ = true;
            // LOG(WARNING)<< "IMU file read finished! "<<" Total "<<imus_.size()<<" IMU data.";


      }
      else
      {

        imu_ = imus_[index_++];
        // imus_.pop_front();
        // initialization of DataCluster
        // LOG(INFO)<<index_<<" IMU data read!";
        std::shared_ptr<DataCluster> data_ptr;
        data_ptr = std::make_shared<DataCluster>(FormatorType::IMUImr);
        data_ptr->imu->time = gnss_common::gtimeToDouble(imu_.time);
        for (int k = 0; k < 3; k++)
        {
          data_ptr->imu->acceleration[k] = imu_.acc[k];
          data_ptr->imu->angular_velocity[k] = imu_.gyro[k];
        }
        data.push_back(data_ptr);

        if (++n_data >= MaxDataSize::IMUPack)
        {
          LOG(WARNING) << "Max data length surpassed!";
          // return -3;
        }
      }
    }

    return n_data;
  }

  int IMUImrFormator::encode(const std::shared_ptr<DataCluster> &data,
                             uint8_t *buf)
  {
  }





  ImageFileFormator::ImageFileFormator(Option &option)
  {

    type_ = FormatorType::ImageFile;
    line_.reserve(max_line_length_);

    init_img(&image_, option.width, option.height, option.step);
    for (int i = 0; i < MaxDataSize::ImagePack; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(
          FormatorType::ImageFile, option.width, option.height, option.step));
    }

  }
  ImageFileFormator::ImageFileFormator(YAML::Node &node)
  {

    type_ = FormatorType::ImageFile;
    // default path
    option_.filepath = "./image/";
    option_.start_time = {0};
    // load path from yaml
    loadPath(node, option_.filepath);
    // loadStartTime(node,option_.start_time);
    line_.reserve(max_line_length_);

    Option option;
    LOAD_REQUIRED(width);
    LOAD_REQUIRED(height);
    LOAD_COMMON(step);

    init_img(&image_, option.width, option.height, option.step);
    for (int i = 0; i < MaxDataSize::ImagePack; i++)
    {
      data_.push_back(std::make_shared<DataCluster>(
          FormatorType::ImageFile, option.width, option.height, option.step));
    }

  }
  ImageFileFormator::~ImageFileFormator()
  {
    free_img(&image_);
  }

  int ImageFileFormator::decode(const uint8_t *buf, int size,
                             std::vector<std::shared_ptr<DataCluster>> &data)
  {

    if(finished_reading_ && image_names_.size()<=0 ){
      return -3;
    }

    int iobs = 0;
    std::stringstream ss;
    data.clear();


    if (!finished_filelist_)
    {
      // read file paths
      for (int i = 0; i < size; i++)
      {
        if (buf[i] != '\n')
          line_ = line_ + (char)buf[i];
        else
        {
          // end of rnx-o file
          if (line_.substr(0, 5) == "*RNXS")
          {
            finished_filelist_ = true;
            break;
          }

          std::string  filename;
          ss.clear();
          ss.str(line_);
          ss >> filename;
          files_.push_back(filename);
          line_.clear();
        }
      }
    }
    else
    {
      // input observation to DataCluster
      if (!finished_reading_)
      {
        // read each rinex file
        for (auto file : files_)
        {

          std::ifstream imu_file(option_.filepath + file);
          std::string line;
          while (std::getline(imu_file, line))
          {
            if(line[0]=="#" || line[0]=="%") continue;

            ss.clear();
            ss.str(line);

            // read 6-axis imu data
            std::string time, image_name;
            ss >> time>> image_name ;
            // add imu data to imu_
            image_names_.push_back(std::make_pair(std::stod(time),image_name));
          }
        }

        finished_reading_ = true;
      }
      else
      {
        auto image_name = image_names_.front();
        image_names_.pop_front();

        cv::Mat image= cv::imread(option_.filepath +image_name.second, 0); 
        if(image.empty()){
          LOG(ERROR) << "Image file open failed: "<< image_name.second;
          return 0;
        }

        // dump data to image_
        image_.time.sec =  image_name.first/(1.0*1E6);  // test --sbs
        image_.time.time = int(image_.time.sec);
        image_.time.sec = image_.time.sec - image_.time.time;


        // (TODO:--sbs) only for Mono8 image, need to be modified for RGB32 type
        for(int i=0;i<image_.height;i++){
          for(int j=0;j<image_.width;j++){
            image_.image[i*image_.width+j] = image.at<uchar>(i,j);
          }
        }

        // initialization of DataCluster
        data_[iobs]->image->time = gnss_common::gtimeToDouble(image_.time);
        memcpy(data_[iobs]->image->image, image_.image,
             sizeof(uint8_t) * image_.width * image_.height * image_.step);

        data.push_back(data_[iobs]);

        if (++iobs >= MaxDataSize::ImagePack)
        {
          LOG(WARNING) << "Max image data length surpassed!";
          return iobs;
        }
      }
    }

    return iobs;
  }

  int ImageFileFormator::encode(const std::shared_ptr<DataCluster> &data,
                             uint8_t *buf)
  {
  }




// -------------------------------------------------------------
// Get formator handle from yaml
#define MAP_FORMATOR(Type, Formator)         \
  if (type == Type)                          \
  {                                          \
    return std::make_shared<Formator>(node); \
  }
#define LOG_UNSUPPORT LOG(FATAL) << "Formator type not supported!";
  inline static FormatorType loadType(YAML::Node &node)
  {
    if (!node["type"].IsDefined())
    {
      LOG(FATAL) << "Unable to load formator type!";
    }
    std::string type_str = node["type"].as<std::string>();
    FormatorType type;
    option_tools::convert(type_str, type);
    return type;
  }
  std::shared_ptr<FormatorBase> makeFormator(YAML::Node &node)
  {
    FormatorType type = loadType(node);
    MAP_FORMATOR(FormatorType::RTCM2, RTCM2Formator);
    MAP_FORMATOR(FormatorType::RTCM3, RTCM3Formator);
    MAP_FORMATOR(FormatorType::GnssRaw, GnssRawFormator);
    MAP_FORMATOR(FormatorType::ImagePack, ImagePackFormator);
    MAP_FORMATOR(FormatorType::ImageV4L2, ImageV4L2Formator);
    MAP_FORMATOR(FormatorType::IMUPack, IMUPackFormator);
    MAP_FORMATOR(FormatorType::OptionPack, OptionFormator);
    MAP_FORMATOR(FormatorType::NMEA, NmeaFormator);
    MAP_FORMATOR(FormatorType::DcbFile, DcbFileFormator);
    MAP_FORMATOR(FormatorType::AtxFile, AtxFileFormator);
    MAP_FORMATOR(FormatorType::RnxOFile, RnxOFileFormator); //--sbs
    MAP_FORMATOR(FormatorType::RnxNFile, RnxNFileFormator); //--sbs
    MAP_FORMATOR(FormatorType::OsbFile, OsbFileFormator);
    MAP_FORMATOR(FormatorType::Sp3File, Sp3FileFormator);
    MAP_FORMATOR(FormatorType::ClkFile, ClkFileFormator);
    MAP_FORMATOR(FormatorType::IonFile, IonFileFormator);
    MAP_FORMATOR(FormatorType::ZtdFile, ZtdFileFormator);
    MAP_FORMATOR(FormatorType::IMUImr, IMUImrFormator);
    MAP_FORMATOR(FormatorType::ImageFile, ImageFileFormator);
    LOG_UNSUPPORT;
    return nullptr;
  }

} // namespace gici