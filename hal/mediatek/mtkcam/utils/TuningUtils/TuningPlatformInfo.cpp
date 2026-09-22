/*
 * Copyright (C) 2019 MediaTek Inc.
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

#define LOG_TAG "mtkcam-tuning-util"

#include <base/files/file_path.h>
#include <base/files/file_util.h>
#include "cros-camera/utils/camera_config.h"

#include <memory>
#include <mtkcam/def/common.h>
#include <mtkcam/utils/std/Log.h>
#include <mtkcam/utils/TuningUtils/TuningPlatformInfo.h>
#include <property_service/property.h>
#include <property_service/property_lib.h>

namespace NSCam {
namespace TuningUtils {

TuningPlatformInfo::TuningPlatformInfo() {}

int GetTuningIndex(int sensor_id) {
  TuningPlatformInfo tuning_info;
  PlatformInfo sensor_info;
  tuning_info.getTuningInfo(&sensor_info);
  if (sensor_id == 0) {
    return sensor_info.wf_sensor.tuning_module_id;
  } else if (sensor_id == 1) {
    return sensor_info.uf_sensor.tuning_module_id;
  } else {
    CAM_LOGE("%s: sensor id out of range", __func__);
    return 0;
  }
}

void TuningPlatformInfo::getTuningInfo(PlatformInfo* info) {
  getTuningInfo(info, "");
}

void TuningPlatformInfo::getTuningInfo(PlatformInfo* info,
                                       const std::string& config_path) {
  if (info == nullptr) {
    return;
  }

  std::unique_ptr<cros::CameraConfig> camera_config;
  if (!config_path.empty()) {
    base::FilePath explicit_path(config_path);
    if (explicit_path.IsAbsolute() && !explicit_path.ReferencesParent() &&
        base::PathExists(explicit_path)) {
      camera_config = cros::CameraConfig::Create(config_path);
    }
    if (!camera_config) {
      camera_config =
          cros::CameraConfig::Create("/etc/camera/camera_info.json");
    }
  } else {
    base::FilePath cros_config_path("/run/camera/camera_config_path");
    if (base::PathExists(cros_config_path)) {
      camera_config =
          cros::CameraConfig::Create("/run/camera/camera_config_path");
    }
    if (!camera_config) {
      camera_config =
          cros::CameraConfig::Create("/etc/camera/camera_info.json");
    }
  }

  if (!camera_config) {
    CAM_LOGE("%s: failed to load camera config, using defaults", __func__);
    info->wf_sensor.tuning_module_id = 0;
    info->wf_sensor.mirror = 0;
    info->wf_sensor.flip = 0;
    info->wf_sensor.eeprom_path = "/sys/bus/nvmem/devices/2-00500/nvmem";
    info->wf_sensor.orientation = 0;
    info->wf_sensor.minFocusDistance = 0;
    info->uf_sensor.tuning_module_id = 0;
    info->uf_sensor.mirror = 0;
    info->uf_sensor.flip = 0;
    info->uf_sensor.eeprom_path = "/sys/bus/nvmem/devices/4-00500/nvmem";
    info->uf_sensor.orientation = 0;
    info->uf_sensor.minFocusDistance = 0;
    return;
  }

  info->wf_sensor.tuning_module_id =
      camera_config->GetInteger("wf_sensor_tuning_module_id", 0);
  info->wf_sensor.mirror = camera_config->GetInteger("wf_sensor_mirror", 0);
  info->wf_sensor.flip = camera_config->GetInteger("wf_sensor_flip", 0);
  info->wf_sensor.eeprom_path = camera_config->GetString(
      "wf_sensor_eeprom", "/sys/bus/nvmem/devices/2-00500/nvmem");
  info->wf_sensor.orientation =
      camera_config->GetInteger("wf_sensor_orientation", 0);
  info->wf_sensor.minFocusDistance =
      camera_config->GetInteger("wf_sensor_minFocusDistance", 0);

  info->uf_sensor.tuning_module_id =
      camera_config->GetInteger("uf_sensor_tuning_module_id", 0);
  info->uf_sensor.mirror = camera_config->GetInteger("uf_sensor_mirror", 0);
  info->uf_sensor.flip = camera_config->GetInteger("uf_sensor_flip", 0);
  info->uf_sensor.eeprom_path = camera_config->GetString(
      "uf_sensor_eeprom", "/sys/bus/nvmem/devices/4-00500/nvmem");
  info->uf_sensor.orientation =
      camera_config->GetInteger("uf_sensor_orientation", 0);
  info->uf_sensor.minFocusDistance =
      camera_config->GetInteger("uf_sensor_minFocusDistance", 0);
}

}  // namespace TuningUtils
}  // namespace NSCam
