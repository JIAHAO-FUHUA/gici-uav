/**
* @Function: GNSS parameter blocks for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/common_parameter_block.h"

#include "gici/estimate/relative_const_error.h"
namespace gici {

// All the parameters used by Odometer are common blocks
using ScalerParameterBlock = CommonParameterBlock<2, CommonParameterBlockType::Scaler>;
using RelativeScalerError = RelativeConstError<2, ErrorType::kRelativeScalerError>;


} 
