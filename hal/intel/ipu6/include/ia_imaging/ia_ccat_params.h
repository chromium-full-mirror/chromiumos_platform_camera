/*
 * Copyright (C) 2017-2020 Intel Corporation.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*!
 * \file ia_ccat_params.h
 * \brief Definitions of CCAT constants.
*/

#ifndef CCAT_PARAMS_H_
#define CCAT_PARAMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MANUAL_CONVERGENCE_TIME_GRANULARITY 0.01f
#define TIMED_TRIMMED_FILTER_SIZE 34
#define CCAT_DEFAULT_MODULE_ISO 100U
#define MAX_IR_WEIGHT_GRID_SIZE 480u /* Max IR weight grid size = ir_width x ir_height*/
#define MAX_NUM_SECTORS 36u
#define MAX_NUM_IR_WEIGHT_GRIDS 30u
#define PI_2 1.57079f
#define IA_AEC_FLASHES_NUM 2
#define MAX_NUM_PERCENTILES 20

#define MIN_LUMINANCE_SATURATED_TARGET 92
#define MIN_LUMINANCE_BRIGHT_TARGET 50
#define LUMINANCE_BASED_MOTION_THRESHOLD 8
#define MAX_STATISTICS_WIDTH 200

#define USE_STABLE_FACE

#ifdef __cplusplus
}
#endif

#endif /* CCAT_PARAMS_H_ */