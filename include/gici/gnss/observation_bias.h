

/**
* @Function: GNSS types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include "gici/utility/rtklib_safe.h"

namespace gici{

struct zcb_t{
    int code;
    double value;
};

struct zpb_t{
    int phase;
    double value;
};
typedef std::vector<std::pair<std::string, zcb_t>> zcbs_t;

typedef std::vector<std::pair<std::string, zpb_t>> zpb_epoch_t;
typedef std::map<double, zpb_epoch_t> zpbs_t;

struct osb_t{
    // Code bias
    zcbs_t zcbs;
    // Phase bias
    zpbs_t zpbs_map;

};

}