/**********************************************************************
 * SOFA PPP-RTK (Supporting Optimization-/Filter- based advanced PPP-RTK)
 *
 * Copyright (C) : Shaoming Xin,
 *
 * Author : Shaoming Xin  (sxin@whu.edu.cn)
 **********************************************************************/

#include "atmnetwork.h"

// for current epoch
double ionnetwork::idw_sdiondelay(oposition pos, std::string sat, std::string sat_ref,
                      int &actn, double refs[][3], bool &isfix) const {
  actn = 0;
  double pnt[3] = {pos.B, pos.L, 0};
  //
  isfix = true;
  for(size_t i = 0; i < ions.size(); ++i) {
    if(ions[i].ddelay.find(sat) == ions[i].ddelay.end()) continue;
    //if(!ions[i].flag[sat] || !ions[i].flag[sat_ref]) continue;
    if(!ions[i].flag.find(sat)->second || !ions[i].flag.find(sat_ref)->second) continue;
    refs[actn][0] = ions[i].pos.B;
    refs[actn][1] = ions[i].pos.L;
    refs[actn][2] = ions[i].ddelay.find(sat)->second - ions[i].ddelay.find(sat_ref)->second;
    actn++;
  }
  if(actn == 0) {
    for(size_t i = 0; i < ions.size(); ++i) {
      if(ions[i].ddelay.find(sat) == ions[i].ddelay.end()) continue;
      refs[actn][0] = ions[i].pos.B;
      refs[actn][1] = ions[i].pos.L;
      refs[actn][2] = ions[i].ddelay.find(sat)->second - ions[i].ddelay.find(sat_ref)->second;
      actn++;
    }
    isfix = false;
  }
  double value = 0; //!
  if(actn > 0) value = gnss_utils::idw(pnt, actn, &refs[0][0]);
  return value;
}

double ionnetwork::var_CrossValid(oposition pos,
                      int num, double refs[][3]) const {
  if(num <= 2) return var_VarOfSits(num, refs);
  //
  double mean_dist = 0.0;
  double sum_mean_dist = 0.0;
  double sum_pre_var = 0.0;
  //
  for(int i = 0; i < num; ++i) {
    double pnt[3] = {refs[i][0], refs[i][1], 0};
    double refstmp[MAXSIT_ATM][3]={0};
    int actn = 0;
    double mean_dist_tmp = 0.0;
    for(int j = 0; j < num; ++j) {
      if(i == j) continue;
      refstmp[actn][0] = refs[j][0];
      refstmp[actn][1] = refs[j][1];
      refstmp[actn][2] = refs[j][2];
      mean_dist_tmp += sqrt(pow((refstmp[actn][0] - pnt[0]), 2) +
                            pow((refstmp[actn][1] - pnt[1]), 2));
      actn++;
    }
    mean_dist_tmp /= actn;
    double pre_var =
      pow((refs[i][2] - gnss_utils::idw(pnt, actn, &refstmp[0][0])),2);
    sum_pre_var += pre_var;
    sum_mean_dist += mean_dist_tmp;
    //
    mean_dist += sqrt(pow((pos.B - pnt[0]), 2) +
                      pow((pos.L - pnt[1]), 2));
  }
  mean_dist /= num;
  return sqrt((mean_dist / sum_mean_dist) * sum_pre_var);
}

double ionnetwork::var_VarOfSits(int num, double refs[][3]) const {
  if(num <= 1) return var_Default();
  std::vector<double> array;
  for(int i = 0; i < num; ++i) array.push_back(refs[i][2]);
  double mean = 0;
  return omath::calstd(mean, array);
}

double ionnetwork::var_Default() const {
  return 0.05;
}

// for between epochs
double ionnetwork::idw_ionvariation(oposition pos, std::string sat, double delta_t,
                        int &actn, double refs[][3], bool &isfix) const {
  actn = 0;
  double pnt[3] = {pos.B, pos.L, 0};
  //
  isfix = true;
  for(size_t i = 0; i < ions.size(); ++i) {
    if(ions[i].ddelay.find(sat) == ions[i].ddelay.end()) continue;
    if(!ions[i].flag.find(sat)->second) continue;
    refs[actn][0] = ions[i].pos.B;
    refs[actn][1] = ions[i].pos.L;
    double nextvalue = 0;
    if(delta_t <= 1.001 ||
       ions[i].pre_coef.find(sat) == ions[i].pre_coef.end())
      nextvalue = ions[i].pre_delay_next.find(sat)->second;
    else {
      nextvalue = ions[i].pre_coef.find(sat)->second.first +
                  ions[i].pre_coef.find(sat)->second.second * (delta_t + POLYPERIOD);
    }
    if(fabs(nextvalue) < 0.001) nextvalue = ions[i].pre_delay_next.find(sat)->second;
    refs[actn][2] = nextvalue - ions[i].delay.find(sat)->second;
    actn++;
  }
  if(actn == 0) {
    for(size_t i = 0; i < ions.size(); ++i) {
      if(ions[i].ddelay.find(sat) == ions[i].ddelay.end()) continue;
      refs[actn][0] = ions[i].pos.B;
      refs[actn][1] = ions[i].pos.L;
      double nextvalue = 0;
      if(delta_t <= 1.001 ||
         ions[i].pre_coef.find(sat) == ions[i].pre_coef.end())
        nextvalue = ions[i].pre_delay_next.find(sat)->second;
      else {
        nextvalue = ions[i].pre_coef.find(sat)->second.first +
                    ions[i].pre_coef.find(sat)->second.second  * (delta_t + POLYPERIOD);
      }
      if(fabs(nextvalue) < 0.001) nextvalue = ions[i].pre_delay_next.find(sat)->second;
      refs[actn][2] = nextvalue - ions[i].delay.find(sat)->second;
      actn++;
    }
    isfix = false;
  }
  double value = 0; //!
  if(actn > 0) value = gnss_utils::idw(pnt, actn, &refs[0][0]);
  return value;
}
//
double ionnetwork::var_var_CrossValid(oposition pos, int num, double refs[][3]) const {
  if(num <= 2) return var_var_VarOfSits(num, refs);
  //
  double mean_dist = 0.0;
  double sum_mean_dist = 0.0;
  double sum_pre_var = 0.0;
  //
  for(int i = 0; i < num; ++i) {
    double pnt[3] = {refs[i][0], refs[i][1], 0};
    double refstmp[MAXSIT_ATM][3]={0};
    int actn = 0;
    double mean_dist_tmp = 0.0;
    for(int j = 0; j < num; ++j) {
      if(i == j) continue;
      refstmp[actn][0] = refs[j][0];
      refstmp[actn][1] = refs[j][1];
      refstmp[actn][2] = refs[j][2];
      mean_dist_tmp += sqrt(pow((refstmp[actn][0] - pnt[0]), 2) +
                            pow((refstmp[actn][1] - pnt[1]), 2));
      actn++;
    }
    mean_dist_tmp /= actn;
    double pre_var =
      pow((refs[i][2] - gnss_utils::idw(pnt, actn, &refstmp[0][0])),2);
    sum_pre_var += pre_var;
    sum_mean_dist += mean_dist_tmp;
    //
    mean_dist += sqrt(pow((pos.B - pnt[0]), 2) +
                      pow((pos.L - pnt[1]), 2));
  }
  mean_dist /= num;
  double retvar = sqrt((mean_dist / sum_mean_dist) * sum_pre_var);
  return retvar < 0.02 ? 0.02 : retvar;
}

double ionnetwork::var_var_VarOfSits(int num, double refs[][3]) const {
  if(num <= 1) return var_var_Default();
  std::vector<double> array;
  for(int i = 0; i < num; ++i) array.push_back(refs[i][2]);
  double mean = 0;
  double retvar = omath::calstd(mean, array);
  return retvar < 0.02 ? 0.02 : retvar;
}

double ionnetwork::var_var_Default() const {
  return 0.04;
}

//
void ionnetwork::findreference(std::vector<std::pair<std::string,double>> prnelevs) {
  // we should find the satellite in the server end, the ambiguity is fixed
  // and in the client end, the satellite has the biggest elevation
  bool cfmgps = false, cfmgal = false, cfmgls = false;
  bool cfmqzs = false, cfmbd2 = false, cfmbd3 = false;
  for(size_t i = 0; i < prnelevs.size(); ++i) {
    oprn prn(prnelevs[i].first);
    if(existprns.find(prn.prn()) == existprns.end()) continue;
    // at least three station
    if(existprns.at(prn.prn()).first < 3) continue;
    if(prn.sys() == 'G' && !cfmgps) {
      if(g_ref == "") g_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        g_ref = prn.prn();
        cfmgps = true;
      }
    } else if(prn.sys() == 'R' && !cfmgls) {
      if(r_ref == "") r_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        r_ref = prn.prn();
        cfmgls = true;
      }
    } else if(prn.sys() == 'E' && !cfmgal) {
      if(e_ref == "") e_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        e_ref = prn.prn();
        cfmgal = true;
      }
    } else if(prn.sys() == 'J' && !cfmqzs) {
      if(j_ref == "") j_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        j_ref = prn.prn();
        cfmqzs = true;
      }
    } else if(prn.sys() == 'C' && !cfmbd2 && prn.bds_ver() == 2) {
      if(c2_ref == "") c2_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        c2_ref = prn.prn();
        cfmbd2 = true;
      }
    } else if(prn.sys() == 'C' && !cfmbd3 && prn.bds_ver() == 3) {
      if(c3_ref == "") c3_ref = prn.prn();
      // have at least 3 stations, update and fixed
      if(existprns.at(prn.prn()).second >= 3) {
        c3_ref = prn.prn();
        cfmbd3 = true;
      }
    }
  }
}

void ionnetwork::calsdion() {
  for(size_t i = 0; i < ions.size(); ++i) {
    for(auto it = ions[i].delay.begin();
        it != ions[i].delay.end(); ++it) {
      oprn prn(it->first);
      std::string refprn = "";
      if(prn.sys() == 'G') {
        refprn = g_ref;
      } else if(prn.sys() == 'R') {
        refprn = r_ref;
      } else if(prn.sys() == 'E') {
        refprn = e_ref;
      } else if(prn.sys() == 'J') {
        refprn = j_ref;
      } else if(prn.sys() == 'C' && prn.bds_ver() == 2) {
        refprn = c2_ref;
      } else if(prn.sys() == 'C' && prn.bds_ver() == 3) {
        refprn = c3_ref;
      }
      if(ions[i].delay.find(refprn) == ions[i].delay.end()) continue;
      if(it->first == refprn || refprn == "") continue;
      ions[i].ddelay[it->first] = ions[i].delay.at(it->first) -
                                  ions[i].delay.at(refprn);
    }
  }
}

void ionnetwork::clear() {
  for(size_t i = 0; i < ions.size(); ++i) {
    ions[i].delay.clear();
    ions[i].ddelay.clear();
    ions[i].flag.clear();
  }
  ions.clear();
}

//--------------------
// for current epoch
double ztdnetwork::idw_ztddelay(oposition pos, int &actn, double refs[][3]) const {
  actn = 0;
  double pnt[3] = {pos.B, pos.L, 0};
  //
  for(size_t i = 0; i < ztds.size(); ++i) {
    refs[actn][0] = ztds[i].pos.B;
    refs[actn][1] = ztds[i].pos.L;
    refs[actn][2] = ztds[i].delay;
    actn++;
  }
  double value = 0; //!
  if(actn > 0) value = gnss_utils::idw(pnt, actn, &refs[0][0]);
  return value;
}

double ztdnetwork::var_CrossValid(oposition pos, int num, double refs[][3]) const {
  if(num <= 2) return var_VarOfSits(num, refs);
  //
  double mean_dist = 0.0;
  double sum_mean_dist = 0.0;
  double sum_pre_var = 0.0;
  //
  for(int i = 0; i < num; ++i) {
    double pnt[3] = {refs[i][0], refs[i][1], 0};
    double refstmp[MAXSIT_ATM][3]={0};
    int actn = 0;
    double mean_dist_tmp = 0.0;
    for(int j = 0; j < num; ++j) {
      if(i == j) continue;
      refstmp[actn][0] = refs[j][0];
      refstmp[actn][1] = refs[j][1];
      refstmp[actn][2] = refs[j][2];
      mean_dist_tmp += sqrt(pow((refstmp[actn][0] - pnt[0]), 2) +
                            pow((refstmp[actn][1] - pnt[1]), 2));
      actn++;
    }
    mean_dist_tmp /= actn;
    double pre_var =
      pow((refs[i][2] - gnss_utils::idw(pnt, actn, &refstmp[0][0])),2);
    sum_pre_var += pre_var;
    sum_mean_dist += mean_dist_tmp;
    //
    mean_dist += sqrt(pow((pos.B - pnt[0]), 2) +
                      pow((pos.L - pnt[1]), 2));
  }
  mean_dist /= num;
  return sqrt((mean_dist / sum_mean_dist) * sum_pre_var);
}

double ztdnetwork::var_VarOfSits(int num, double refs[][3]) const {
  if(num <= 1) return var_Default();
  std::vector<double> array;
  for(int i = 0; i < num; ++i) array.push_back(refs[i][2]);
  double mean = 0;
  return omath::calstd(mean, array);
}

double ztdnetwork::var_Default() const {
  return 0.05;
}

// for between epoch
double ztdnetwork::idw_ztdvariation(oposition pos, double delta_t,
                        int &actn, double refs[][3]) const {
  actn = 0;
  double pnt[3] = {pos.B, pos.L, 0};
  //
  for(size_t i = 0; i < ztds.size(); ++i) {
    refs[actn][0] = ztds[i].pos.B;
    refs[actn][1] = ztds[i].pos.L;
    double nextvalue = 0;
    if(delta_t <= 1.001)
      nextvalue = ztds[i].pre_delay_next;
    else {
      nextvalue = ztds[i].pre_coef.first +
                  ztds[i].pre_coef.second * (delta_t + POLYPERIOD);
    }
    if(fabs(nextvalue) < 0.001) nextvalue = ztds[i].pre_delay_next;
    refs[actn][2] = nextvalue - ztds[i].delay;
    actn++;
  }
  double value = 0; //!
  if(actn > 0) value = gnss_utils::idw(pnt, actn, &refs[0][0]);
  return value;
}

double ztdnetwork::var_var_CrossValid(oposition pos, int num, double refs[][3]) const {
  if(num <= 2) return var_var_VarOfSits(num, refs);
  //
  double mean_dist = 0.0;
  double sum_mean_dist = 0.0;
  double sum_pre_var = 0.0;
  //
  for(int i = 0; i < num; ++i) {
    double pnt[3] = {refs[i][0], refs[i][1], 0};
    double refstmp[MAXSIT_ATM][3]={0};
    int actn = 0;
    double mean_dist_tmp = 0.0;
    for(int j = 0; j < num; ++j) {
      if(i == j) continue;
      refstmp[actn][0] = refs[j][0];
      refstmp[actn][1] = refs[j][1];
      refstmp[actn][2] = refs[j][2];
      mean_dist_tmp += sqrt(pow((refstmp[actn][0] - pnt[0]), 2) +
                            pow((refstmp[actn][1] - pnt[1]), 2));
      actn++;
    }
    mean_dist_tmp /= actn;
    double pre_var =
      pow((refs[i][2] - gnss_utils::idw(pnt, actn, &refstmp[0][0])),2);
    sum_pre_var += pre_var;
    sum_mean_dist += mean_dist_tmp;
    //
    mean_dist += sqrt(pow((pos.B - pnt[0]), 2) +
                      pow((pos.L - pnt[1]), 2));
  }
  mean_dist /= num;
  double retvar = sqrt((mean_dist / sum_mean_dist) * sum_pre_var);
  return retvar < 0.0005 ? 0.0005: retvar;
}

double ztdnetwork::var_var_VarOfSits(int num, double refs[][3]) const {
  if(num <= 1) return var_var_Default();
  std::vector<double> array;
  for(int i = 0; i < num; ++i) array.push_back(refs[i][2]);
  double mean = 0;
  double retvar = omath::calstd(mean, array);
  return retvar < 0.0005 ? 0.0005: retvar;
}

double ztdnetwork::var_var_Default() const {
  return 0.0005;
}

void ztdnetwork::clear() {
  ztds.clear();
}

//---------------------
void atm_predict::ionpolypredict(bool lpoly, ionnetwork &ion1, std::map<double, ionnetwork> ssr_ion_map) {
  double defaultvar = 0.04;
  if(ion1.tim.sod() < POLYPERIOD) lpoly = false;
  if(!lpoly) {
    for(size_t i = 0; i < ion1.ions.size(); ++i) {
      for(auto it = ion1.ions[i].delay.begin();
          it != ion1.ions[i].delay.end(); ++it) {
        ion1.ions[i].pre_delay_next[it->first] = it->second + defaultvar;
      }
    }
  } else {
    //
    double timstart = ion1.tim.sod() - POLYPERIOD + 1;
    for(size_t i = 0; i < ion1.ions.size(); ++i) {
      for(auto it = ion1.ions[i].delay.begin();
          it != ion1.ions[i].delay.end(); ++it) {
        std::string prn = it->first;
        //<tim, ion>
        std::vector<std::pair<double, double>> info;
        //-- use a reverse iteroator , may be more effective than count way
        for(auto rit = ssr_ion_map.rbegin();
            rit != ssr_ion_map.rend(); ++rit) {
          double tt = rit->first;
          if(tt < timstart) break;
          if(rit->second.ions[i].delay.find(prn) ==
             rit->second.ions[i].delay.end()) continue;
          if(rit->second.ions[i].name != ion1.ions[i].name) {
            LOGE << MSGE("atmnetwork:ion:polypredict",
                         "sit name not same, shouldn't be\n");
            continue;
          }
          std::pair<double, double>
            tmp(tt - timstart + 1,rit->second.ions[i].delay.at(prn));
          info.push_back(tmp);
        }
        //--
        //
        std::pair<double, double> tmpnow(POLYPERIOD,it->second);
        info.push_back(tmpnow);
        //
        if(info.size() < 2) {
          ion1.ions[i].pre_delay_next[it->first] = it->second + defaultvar;
          continue;
        }
        //
        double coef_a0, coef_a1;
        omath::linearfit(info, coef_a0, coef_a1);
        ion1.ions[i].pre_delay_next[it->first] =
          coef_a0 + coef_a1 * (POLYPERIOD + 1);
        ion1.ions[i].pre_coef[it->first] = std::make_pair(coef_a0, coef_a1);
      }
    }
  }
}


void atm_predict::ztdpolypredict(bool lpoly, ztdnetwork &ztd1, std::map<double, ztdnetwork> ssr_ztd_map) {
  double defaultvar = 0.0001;
  if(ztd1.tim.sod() < POLYPERIOD) lpoly = false;
  if(!lpoly) {
    for(size_t i = 0; i < ztd1.ztds.size(); ++i) {
      ztd1.ztds[i].pre_delay_next = ztd1.ztds[i].delay + defaultvar;
    }
  } else {
    //
    double timstart = ztd1.tim.sod() - POLYPERIOD + 1;
    for(size_t i = 0; i < ztd1.ztds.size(); ++i) {
      std::vector<std::pair<double, double>> info;
      for(auto rit = ssr_ztd_map.rbegin();
          rit != ssr_ztd_map.rend(); ++rit) {
        double tt = rit->first;
        if(tt < timstart) break;
        std::pair<double, double>
          tmp(tt - timstart + 1,rit->second.ztds[i].delay);
        info.push_back(tmp);
      }
      //
      std::pair<double, double> tmpnow(POLYPERIOD,ztd1.ztds[i].delay);
      info.push_back(tmpnow);
      //
      if(info.size() < 2) {
        ztd1.ztds[i].pre_delay_next = ztd1.ztds[i].delay + defaultvar;
        continue;
      }
      //
      double coef_a0, coef_a1;
      omath::linearfit(info, coef_a0, coef_a1);
      ztd1.ztds[i].pre_delay_next = coef_a0 + coef_a1 * (POLYPERIOD + 1);
      ztd1.ztds[i].pre_coef = std::make_pair(coef_a0, coef_a1);
    }
  }
}

