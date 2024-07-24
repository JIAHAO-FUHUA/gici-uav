/**********************************************************************
 * SOFA PPP-RTK (Supporting Optimization-/Filter- based advanced PPP-RTK)
 *
 * Copyright (C) : Shaoming Xin,
 *
 * Author : Shaoming Xin  (sxin@whu.edu.cn)
 **********************************************************************/

#ifndef ATMNETWORK_H_
#define ATMNETWORK_H_

#include <iostream>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>

#define POLYPERIOD 120

struct station_t{
  std::string name;
  Eigen::Vector3d position;
};

struct ion_t{  
  std::string prn;
  double value;
  double std;
};

typedef std::map<double, std::vector<ion_t>> ions_t;  // time, ion_t
typedef std::vector<std::pair<station_t, ions_t>> ionnetwork;  // station, ions_t

struct ztd_t{  
  double value;
  double std;
};

typedef std::map<double, std::vector<ztd_t>> ztds_t;  // time, ztd_t
typedef std::vector<std::pair<station_t, ztds_t>> ztdnetwork;  // station, ztds_t


// typedef struct {
//   std::string name;
//   Eigen::Vector3d pos;
//   std::map<std::string, double> delay;
//   std::map<std::string, double> ddelay;
//   std::map<std::string, int> flag;
//   std::map<std::string,double> delay_std;
//   // this is predicted by the past (include self) data (within 120 s is ok)
//   // default interval is 1 s
//   std::map<std::string, double> pre_delay_next;
//   std::map<std::string, std::pair<double, double>> pre_coef;
// }ionsit;


// typedef struct {
//   std::string name;
//   Eigen::Vector3d pos;
//   double delay;
//   // this is predicted by the past (include self) data (within 120 s is ok)
//   double pre_delay_next;
//   std::pair<double, double> pre_coef;
// }ztdsit;


// class ionnetwork {
// public:
//   //GREJC2C3 base satellite
//   std::string g_ref = "", r_ref = "", c2_ref = "", c3_ref = "", e_ref = "",
//               j_ref = "";
//   double tim;
//   // prn, reference station number, amb fixed station number
//   std::map<std::string, std::pair<int, int>> existprns;
//   std::vector<ionsit> ions;

//   ionnetwork(){}
//   //double
// //   ionnetwork(std::vector<ionsit0> ions0, double tim0, std::vector<std::string> prns)
// //       :tim(tim0) {
// //     for(size_t i = 0; i < ions0.size(); ++i) {
// //       ionsit iontmp;
// //       iontmp.name = ions0[i].name;
// //       iontmp.pos = ions0[i].pos;
// //       //
// //       double coef1 = 0, coef2 =0;
// //       if((ions0[i].tim1 - tim0.mjds()) * 86400.0 > 30.01) continue;
// //       else if(ions0[i].tim1 - tim0.mjds() >= 0) {
// //         coef1 = 1;
// //         coef2 = 0;
// //       } else if(ions0[i].tim1 - tim0.mjds() <= 0 && ions0[i].tim2 - tim0.mjds() > 0) {
// //         coef1 = (ions0[i].tim2 - tim0.mjds()) / (ions0[i].tim2 - ions0[i].tim1);
// //         coef2 = (tim0.mjds() - ions0[i].tim1) / (ions0[i].tim2 - ions0[i].tim1);
// //       }
// //       if(fabs(coef1) < DBL_EPSILON && fabs(coef2) < DBL_EPSILON) continue;
// //       for(size_t j = 0; j < prns.size(); ++j) {
// // //        if(ions0[i].flag1[j] <= 0 && ions0[i].flag2[j] <=0) continue;
// //         if(fabs(ions0[i].delay1[j]) < DBL_EPSILON ||
// //            fabs(ions0[i].delay2[j]) < DBL_EPSILON) continue;
// //         std::string prn = prns[j];
// //         double delay = 0, ddelay = 0;
// //         delay = coef1 * ions0[i].delay1[j] + coef2 * ions0[i].delay2[j];
// //         iontmp.delay[prn] = delay;
// //         iontmp.ddelay[prn] = ddelay;
// //         if(ions0[i].flag1[j] || ions0[i].flag2[j]) iontmp.flag[prn] = 1;//fixed
// //         else iontmp.flag[prn] = 0;
// //         //
// //         if(existprns.find(prn) == existprns.end())
// //           existprns[prn] = std::make_pair(0, 0);
// //         // if the flag is 1 (amb fixed) the prn fixed station num will increase
// //         existprns[prn].first = existprns[prn].first + 1;
// //         if(iontmp.flag[prn])
// //           existprns[prn].second = existprns[prn].second + 1;
// //       }
// //       //
// //       ions.push_back(iontmp);
// //     }
// //   }
//   ~ionnetwork() {}
// // double
//   // for current epoch
//   // double idw_sdiondelay(oposition pos, std::string sat, std::string sat_ref,
//   //                       int &actn, double refs[][3], bool &isfix) const;

//   // double var_CrossValid(oposition pos,
//   //                       int num, double refs[][3]) const;

//   // double var_VarOfSits(int num, double refs[][3]) const;

//   // double var_Default() const;

//   // // for between epochs
//   // double idw_ionvariation(oposition pos, std::string sat, double delta_t,
//   //                         int &actn, double refs[][3], bool &isfix) const;
//   // //
//   // double var_var_CrossValid(oposition pos, int num, double refs[][3]) const;

//   // double var_var_VarOfSits(int num, double refs[][3]) const;

//   // double var_var_Default() const;

//   // //
//   // void findreference(std::vector<std::pair<std::string,double>> prnelevs);

//   // void calsdion();

//   // void clear();

// };

// class ztdnetwork {
// public:
//   double tim;
//   std::vector<ztdsit> ztds;

//   ztdnetwork() {}
//   // ztdnetwork(std::vector<ztdsit0> ztds0, otime tim0):tim(tim0) {
//   //   for(size_t i = 0; i < ztds0.size(); ++i) {
//   //     ztdsit ztdtmp;
//   //     ztdtmp.name = ztds0[i].name;
//   //     ztdtmp.pos = ztds0[i].pos;
//   //     //
//   //     double coef1 = 0, coef2 =0;
//   //     if((ztds0[i].tim1 - tim0.mjds()) * 86400.0 > 30.01) continue;
//   //     else if(ztds0[i].tim1 - tim0.mjds() >= 0) {
//   //       coef1 = 1;
//   //       coef2 = 0;
//   //     } else if(ztds0[i].tim1 - tim0.mjds() <= 0 && ztds0[i].tim2 - tim0.mjds() > 0) {
//   //       coef1 = (ztds0[i].tim2 - tim0.mjds()) / (ztds0[i].tim2 - ztds0[i].tim1);
//   //       coef2 = (tim0.mjds() - ztds0[i].tim1) / (ztds0[i].tim2 - ztds0[i].tim1);
//   //     }
//   //     if(coef1 == 0 && coef2 ==0) continue;
//   //     double delay = 0;
//   //     delay = coef1 * ztds0[i].delay1 + coef2 * ztds0[i].delay2;
//   //     ztdtmp.delay = delay;
//   //     ztdtmp.pre_delay_next = 0;
//   //     //
//   //     ztds.push_back(ztdtmp);
//   //   }
//   // }
//   ~ztdnetwork() {}

//   // // for current epoch
//   // double idw_ztddelay(oposition pos, int &actn, double refs[][3]) const;

//   // double var_CrossValid(oposition pos, int num, double refs[][3]) const;

//   // double var_VarOfSits(int num, double refs[][3]) const;

//   // double var_Default() const;

//   // // for between epoch
//   // double idw_ztdvariation(oposition pos, double delta_t,
//   //                         int &actn, double refs[][3]) const;

//   // double var_var_CrossValid(oposition pos, int num, double refs[][3]) const;

//   // double var_var_VarOfSits(int num, double refs[][3]) const;

//   // double var_var_Default() const;

//   // void clear();
// };

// // class atm_predict {
// // public:
// //   static void ionpolypredict(bool lpoly, ionnetwork &ion1,
// //                              std::map<double, ionnetwork> ssr_ion_map);

// //   static void ztdpolypredict(bool lpoly, ztdnetwork &ztd1,
// //                              std::map<double, ztdnetwork> ssr_ztd_map);
// // };

#endif
