/**
* @Function: Decoding and encoding stream
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <memory>
#include <glog/logging.h>

#include "gici/stream/format_image.h"
#include "gici/stream/format_imu.h"
#include "gici/utility/option.h"
#include "gici/utility/rtklib_safe.h"
#include "gici/estimate/estimator_types.h"
#include "gici/gnss/code_bias.h"

#include "gici/gnss/observation_bias.h" //--sbs

namespace gici {

// Formator types
enum class FormatorType {
  RTCM2, 
  RTCM3,
  GnssRaw, 
  ImageV4L2,
  ImagePack,  
  IMUPack,
  OptionPack, 
  NMEA,
  DcbFile,
  AtxFile,

  // for PPP-RTK --sbs
  RnxOFile,
  RnxNFile,
  OsbFile,
  Sp3File,
  ClkFile,
  IonFile,
  ZtdFile,
  ImageFile,

  IMUImr,
  Odometer,
};

// GNSS data types
enum class GnssDataType {
  None = 0,
  Ephemeris = 2,
  Observation = 1,
  AntePos = 5,  // Antenna position
  IonAndUtcPara = 9,  // Ionosphere and UTC parameters
  SSR = 10,
  PhaseCenter=11,   // PCVs and PCOs

  PreciseEph = 12,  // Post-time Precise ephemeris  --sbs
  PreciseClk = 13,  // Post-time Precise clock  --sbs
  ObservationBias = 14,  //Post-time  Observation bias  --sbs
  IonNetwork= 15,  //Post-time  Ionosphere from PRIDE  --sbs
  ZtdNetwork= 16  //Post-time  ZTD from PRIDE

};

// Data 
class DataCluster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataCluster() {}

  DataCluster(FormatorType type);

  DataCluster(FormatorType type, int _width, int _height, int _step);

  DataCluster(const Solution& data) : 
    solution(std::make_shared<Solution>(data)) {}

  DataCluster(const FramePtr& data) : frame(data) {}

  DataCluster(const MapPtr& data) : map(data) {}

  ~DataCluster();

  // GNSS data format
  struct GNSS {
    void init();
    void free();

    std::vector<GnssDataType> types;
    obs_t *observation;
    nav_t *ephemeris;
    sta_t *antenna;

    osb_t obs_bias; //--sbs
    ionnetwork ion_network; //--sbs  
    ztdnetwork ztd_network; //--sbs
  };

  // Image data format
  struct Image {
    void init(int _width, int _height, int _step);
    void free();

    double time;
    int width;
    int height;
    int step;
    uint8_t *image;
  };

  // IMU data format
  struct IMU {
    double time;
    double acceleration[3];
    double angular_velocity[3];
  };

  // Odometry data format --sbs
  struct Odometer {
    double time;
    double forward_velocity;
    double bearing_angular_velocity;
  };

  // Option data format
  struct Option {

  };

  // Input data types
  std::shared_ptr<GNSS> gnss;
  std::shared_ptr<Image> image;
  std::shared_ptr<IMU> imu;
  std::shared_ptr<Odometer> odometer; //--sbs

  std::shared_ptr<Option> option;

  // Output data types
  std::shared_ptr<Solution> solution;
  std::shared_ptr<Frame> frame;
  std::shared_ptr<Map> map;
};

// Formats of FormatorType::GNSS_Raw
enum class GnssRawFormats {
  Ublox = STRFMT_UBX,
  Septentrio = STRFMT_SEPT,
  Novatel = STRFMT_OEM4,
  Tersus = STRFMT_OEM4
};

// Max number of output data buffers for decoders
struct MaxDataSize {
  static const int RTCM2 = 30;
  static const int RTCM3 = RTCM2;
  static const int GnssRaw = RTCM2;
  static const int ImagePack = 2;
  static const int IMUPack = 500;
  static const int RnxOFile = 200;  // for batch processing <= 200 stations --sbs
  static const int MaxObsEpoch = 200;  //  --sbs
};

// Tools for RTKLIB types
namespace gnss_common {

// Update observation data
extern void updateObservation(
  obs_t *obs, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ssr corrections
enum class UpdateSsrType {
  Ephemeris,
  CodeBias,
  PhaseBias,
  Ionosphere,
  Troposphere  //--sbs
};
extern void updateSsr(
  ssr_t *ssr, std::shared_ptr<DataCluster::GNSS>& gnss_data,
  std::vector<UpdateSsrType> type = {UpdateSsrType::Ephemeris, 
  UpdateSsrType::CodeBias, UpdateSsrType::PhaseBias});



extern void updatePreciseEph(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data);

extern void updatePreciseClk(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Select data from GNSS stream
// Note that data except for observation are 
// putted in the first place of the vector
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data);

}

// Base class
class FormatorBase {
public:
  FormatorBase() { }
  ~FormatorBase() { }

  // Decode stream to data
  virtual int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) = 0;

  // Encode data to stream
  virtual int encode(
    const std::shared_ptr<DataCluster>& data, uint8_t *buf) = 0;

  // Get formator type
  FormatorType getType() { return type_; }

  // Get data handle
  std::vector<std::shared_ptr<DataCluster>>& getDataHandle() { return data_; }

protected:
  std::vector<std::shared_ptr<DataCluster>> data_;
  FormatorType type_;
};

// RTCM 2
class RTCM2Formator : public FormatorBase {
public:
  struct Option {
    double start_time; 
  };

  RTCM2Formator(Option& option);
  RTCM2Formator(YAML::Node& node);
  ~RTCM2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// RTCM 3
class RTCM3Formator : public FormatorBase {
public:
  struct Option {
    double start_time; 
  };

  RTCM3Formator(Option& option);
  RTCM3Formator(YAML::Node& node);
  ~RTCM3Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// GNSS raw
class GnssRawFormator : public FormatorBase {
public:
  struct Option {
    double start_time; 
    std::string sub_type;
  };

  GnssRawFormator(Option& option);
  GnssRawFormator(YAML::Node& node);
  ~GnssRawFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  raw_t raw_;
  GnssRawFormats format_;
};

// Image V4L2
class ImageV4L2Formator : public FormatorBase {
public:
  struct Option {
    int width;
    int height;
    int step = 1;
  };

  ImageV4L2Formator(Option& option);
  ImageV4L2Formator(YAML::Node& node);
  ~ImageV4L2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// Image pack
class ImagePackFormator : public FormatorBase {
public:
  struct Option {
    int width;
    int height;
    int step = 1;
  };

  ImagePackFormator(Option& option);
  ImagePackFormator(YAML::Node& node);
  ~ImagePackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// IMU pack
class IMUPackFormator : public FormatorBase {
public:
  struct Option {
    
  };

  IMUPackFormator(Option& option);
  IMUPackFormator(YAML::Node& node);
  ~IMUPackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  imu_t imu_;
};

// Option
class OptionFormator : public FormatorBase {
public:
  struct Option {
    
  };

  OptionFormator(Option& option);
  OptionFormator(YAML::Node& node);
  ~OptionFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:

};

// NMEA (for solution)
class NmeaFormator : public FormatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Option {
    bool use_gga = true;  // Use GxGGA message
    bool use_rmc = true;  // Use GxRMC message
    bool use_esa = false; // Use GxESA message (see encodeESA)
    bool use_esd = false; // Use GxESD message (see encodeESD)
    std::string talker_id = "GN";
  };

  NmeaFormator(Option& option);
  NmeaFormator(YAML::Node& node);
  ~NmeaFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  // Encode GNGGA message
  int encodeGGA(const Solution& solution, uint8_t* buf);

  // Encode GNRMC message
  int encodeRMC(const Solution& solution, uint8_t* buf);

  // Encode GNESA (self-defined Extended Speed and Attitude) message
  // Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
  int encodeESA(const Solution& solution, uint8_t* buf);

  // Encode GNESD (self-defined Extended STD) message
  // Format: $GNESD,tod,STD_Pe,STD_Pn,STD_Pu,STD_Ve,STD_Vn,STD_Vu,
  //         STD_Ar,STD_Ap,STD_Py*checksum
  int encodeESD(const Solution& solution, uint8_t* buf);

  // Convert Solution to sol_t
  void convertSolution(const Solution& solution, sol_t& sol);

  // Configure
  Option option_;
};

// Read DCB file in CAS format (https://cddis.nasa.gov/archive/gnss/products/bias/)
class DcbFileFormator : public FormatorBase {
public:
  struct Option {
    
  };

  DcbFileFormator(Option& option);
  DcbFileFormator(YAML::Node& node);
  ~DcbFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  const int max_line_length_ = 128;
  std::string line_;
  bool passed_header_ = false;
  bool finished_reading_ = false;

  // map from PRN string to DCB storage
  using Dcb = CodeBias::Dcb;
  std::multimap<std::string, Dcb> dcbs_;
};


// Read OSB(Observation-Specific Biases) file  
// (including code/phase bias; generated by Pride's PPP-RTK server) in CAS format (https://cddis.nasa.gov/archive/gnss/products/bias/)
class OsbFileFormator : public FormatorBase {
public:
  struct Option {
    
  };

  OsbFileFormator(Option& option);
  OsbFileFormator(YAML::Node& node);
  ~OsbFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  const int max_line_length_ = 128;
  std::string line_;
  bool passed_header_ = false;
  bool finished_reading_ = false;

  // map from PRN string to DCB storage
  // using OsbCode = ObsBias::OsbCode;
  // using OsbPhase = ObsBias::OsbPhase;

  // std::multimap<std::string, OsbCode> osbs_code_;
  // std::multimap<std::string, OsbPhase> osbs_phase_;
  osb_t osbs_;
};


// Read IGS SP3 file
class RnxOFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;  
    gtime_t start_time;  
  };

  RnxOFileFormator(Option& option);
  RnxOFileFormator(YAML::Node& node);
  ~RnxOFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:
  // Add antenna parameter

protected:
  const int max_line_length_ = 256;
  std::string line_;
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;

  int index_;
  int dump_epoch_;
  obs_t* obs;
  // Configure
  Option option_;

};


// Read broadcast ephemeris file
class RnxNFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;  
    gtime_t start_time;  
  };

  RnxNFileFormator(Option& option);
  RnxNFileFormator(YAML::Node& node);
  ~RnxNFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:

protected:
  const int max_line_length_ = 256;
  std::string line_;
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;

  int index_;
  int index_glonass_;
  int dump_epoch_;
  nav_t* nav;
  // Configure
  Option option_;

};


// Read IGS SP3 file
class Sp3FileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;  
    gtime_t start_time;  
  };

  Sp3FileFormator(Option& option);
  Sp3FileFormator(YAML::Node& node);
  ~Sp3FileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:
  // Add antenna parameter

protected:
  const int max_line_length_ = 256;
  std::string line_;
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;

  int index_;
  int dump_epoch_;
  nav_t* nav;
  // Configure
  Option option_;

};


// Read IGS CLK file
class ClkFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;  
    gtime_t start_time;  
  };

  ClkFileFormator(Option& option);
  ClkFileFormator(YAML::Node& node);
  ~ClkFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:

protected:
  const int max_line_length_ = 256;
  std::string line_;
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;

  int index_;
  int dump_epoch_;
  nav_t* nav;
  // Configure
  Option option_;

};

class IonFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;
  };

  IonFileFormator(Option& option);
  IonFileFormator(YAML::Node& node);
  ~IonFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;
  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;


private:
  // Read ionosphere file
  int input_ionosphere_file(const station_t ion);

protected:
  const int max_line_length_ = 256;
  std::string line_;
  std::vector<station_t> stations_;
  bool finished_reading_ = false;
  
  ionnetwork ionospheric_network_;

  // Configure
  Option option_;

};


class ZtdFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;
  };

  ZtdFileFormator(Option& option);
  ZtdFileFormator(YAML::Node& node);
  ~ZtdFileFormator();


  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;
  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;


private:
  // Read tropospheric file
  int input_tropospheric_file(const station_t station);


protected:
  std::vector<station_t> stations_;

  const int max_line_length_ = 256;
  
  std::string line_;
  bool finished_reading_ = false;
  ztdnetwork tropospheric_network_;

  // Configure
  Option option_;

};



class IMUImrFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;
    gtime_t start_time;
  };

  IMUImrFormator(Option& option);
  IMUImrFormator(YAML::Node& node);
  ~IMUImrFormator();


  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;
  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  imu_t imu_;
  std::vector<imu_t> imus_;  // cache all imu data

  int index_ ;
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;
  const int max_line_length_ = 256;
  std::string line_;

  // Configure
  Option option_;
};



class ImageFileFormator : public FormatorBase {
public:
  struct Option {
    std::string filepath;
    gtime_t start_time;

    int width;
    int height;
    int step = 1;
  };

  ImageFileFormator(Option& option);
  ImageFileFormator(YAML::Node& node);
  ~ImageFileFormator();


  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;
  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  std::vector<std::string> files_;
  bool finished_filelist_ = false;
  bool finished_reading_ = false;
  const int max_line_length_ = 256;
  std::string line_;

  img_t image_;
  std::deque<std::pair<double, std::string>> image_names_;
  // Configure
  Option option_;

};


// Read IGS ATX file
class AtxFileFormator : public FormatorBase {
public:
  struct Option {
    
  };

  AtxFileFormator(Option& option);
  AtxFileFormator(YAML::Node& node);
  ~AtxFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:
  // Add antenna parameter
  void addpcv(const pcv_t *pcv, pcvs_t *pcvs);

  // Decode antenna parameter field
  int decodef(char *p, int n, double *v);

protected:
  const int max_line_length_ = 256;
  std::string line_;
  pcvs_t *pcvs_;
  pcv_t pcv_;
  int state_ = 0;
  int last_size_ = 0;
};

// Get formator handle from configure
#define MAKE_FORMATOR(Formator) \
inline std::shared_ptr<FormatorBase> makeFormator( \
  Formator::Option& option) { \
   return std::make_shared<Formator>(option); \
}
MAKE_FORMATOR(RTCM2Formator);
MAKE_FORMATOR(RTCM3Formator);
MAKE_FORMATOR(GnssRawFormator);
MAKE_FORMATOR(ImageV4L2Formator);
MAKE_FORMATOR(ImagePackFormator);
MAKE_FORMATOR(IMUPackFormator);
MAKE_FORMATOR(OptionFormator);
MAKE_FORMATOR(NmeaFormator);
MAKE_FORMATOR(DcbFileFormator);
MAKE_FORMATOR(AtxFileFormator);
MAKE_FORMATOR(RnxOFileFormator); //--sbs
MAKE_FORMATOR(RnxNFileFormator);
MAKE_FORMATOR(Sp3FileFormator);
MAKE_FORMATOR(ClkFileFormator);
MAKE_FORMATOR(OsbFileFormator);
MAKE_FORMATOR(IonFileFormator);
MAKE_FORMATOR(ZtdFileFormator);
MAKE_FORMATOR(IMUImrFormator);
MAKE_FORMATOR(ImageFileFormator);


// Get formator handle from yaml
std::shared_ptr<FormatorBase> makeFormator(YAML::Node& node);


}
