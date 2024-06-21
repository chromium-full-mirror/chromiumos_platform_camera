/*
 * Copyright (C) 2019-2024 Intel Corporation
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

#define LOG_TAG HalV3Utils
#include <linux/videodev2.h>
#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include <base/strings/stringprintf.h>
#include <chromeos-config/libcros_config/cros_config.h>

#include "Errors.h"
#include "HALv3Utils.h"
#include "MetadataConvert.h"
#include "PlatformData.h"
#include "Utils.h"

namespace camera3 {
namespace HalV3Utils {

static const char* Camera3StreamTypes[] = {"OUTPUT",         // CAMERA3_STREAM_OUTPUT
                                           "INPUT",          // CAMERA3_STREAM_INPUT
                                           "BIDIRECTIONAL",  // CAMERA3_STREAM_BIDIRECTIONAL
                                           "INVALID"};

const char* getCamera3StreamType(int type) {
    int num = sizeof(Camera3StreamTypes) / sizeof(Camera3StreamTypes[0]);
    return (type >= 0 && type < num) ? Camera3StreamTypes[type] : Camera3StreamTypes[num - 1];
}

int HALFormatToV4l2Format(int cameraId, int halFormat, int usage) {
    int format = -1;
    switch (halFormat) {
        case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
            if (IS_ZSL_USAGE(usage)) {
                format = icamera::PlatformData::getISysRawFormat(cameraId);
            } else {
                format = V4L2_PIX_FMT_NV12;
            }
            break;
        case HAL_PIXEL_FORMAT_YCBCR_P010:
            format = V4L2_PIX_FMT_P010;
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_888:
        case HAL_PIXEL_FORMAT_BLOB:
            format = V4L2_PIX_FMT_NV12;
            break;
        case HAL_PIXEL_FORMAT_RAW_OPAQUE:
            format = icamera::PlatformData::getISysRawFormat(cameraId);
            break;
        default:
            LOGW("unsupport format %d", halFormat);
            break;
    }

    return format;
}

int V4l2FormatToHALFormat(int v4l2Format) {
    int format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    switch (v4l2Format) {
        case V4L2_PIX_FMT_P010:
            format = HAL_PIXEL_FORMAT_YCBCR_P010;
            break;
        default:
            break;
    }

    return format;
}

int getRotationDegrees(const camera3_stream_t& stream) {
    if (stream.stream_type != CAMERA3_STREAM_OUTPUT) {
        LOG2("%s, no need rotation for stream type %d", __func__, stream.stream_type);
        return 0;
    }
    switch (stream.crop_rotate_scale_degrees) {
        case CAMERA3_STREAM_ROTATION_0:
            return 0;
        case CAMERA3_STREAM_ROTATION_90:
            return 90;
        case CAMERA3_STREAM_ROTATION_270:
            return 270;
        default:
            LOGE("unsupport rotate degree: %d, the value must be (0,1,3)",
                 stream.crop_rotate_scale_degrees);
            return -1;
    }
}

bool isSameRatioWithSensor(const icamera::stream_t* stream, int cameraId) {
    const icamera::CameraMetadata* meta = StaticCapability::getInstance().getCapability(cameraId);

    float sensorRatio = 0.0f;
    icamera_metadata_ro_entry entry = meta->find(CAMERA_SENSOR_INFO_PIXEL_ARRAY_SIZE);
    if (entry.count == 2) {
        sensorRatio = static_cast<float>(entry.data.i32[0]) / entry.data.i32[1];
    }

    LOG2("%s, the sensor output sensorRatio: %f", __func__, sensorRatio);
    // invalid sensor output ratio, ignore this condition
    if (sensorRatio == 0.0) return true;

    // The pixel array size may be larger than biggest output size, set the
    // default tolerance to 0.1
    const float RATIO_TOLERANCE = 0.1f;
    const float streamRatio = static_cast<float>(stream->width) / stream->height;
    if (fabs(sensorRatio - streamRatio) < RATIO_TOLERANCE) return true;

    return false;
}

int fillHALStreams(int cameraId, const camera3_stream_t& camera3Stream, icamera::stream_t* stream) {
    LOG1("<id%d>@%s", cameraId, __func__);

    stream->format = HALFormatToV4l2Format(cameraId, camera3Stream.format, camera3Stream.usage);
    CheckAndLogError(stream->format == -1, icamera::BAD_VALUE, "unsupported format %x",
                     camera3Stream.format);

    // For rotation cases, aal needs to get the psl output mapping to user requirement.
    if (getRotationDegrees(camera3Stream) > 0) {
        icamera::camera_resolution_t* psl = icamera::PlatformData::getPslOutputForRotation(
            camera3Stream.width, camera3Stream.height, cameraId);

        stream->width = psl ? psl->width : camera3Stream.height;
        stream->height = psl ? psl->height : camera3Stream.width;
        LOG1("%s, Use the psl output %dx%d to map user requirement: %dx%d", __func__, stream->width,
             stream->height, camera3Stream.width, camera3Stream.height);
    } else {
        stream->width = camera3Stream.width;
        stream->height = camera3Stream.height;
    }

    stream->field = 0;
    stream->stride = icamera::CameraUtils::getStride(stream->format, stream->width);
    stream->size = icamera::CameraUtils::getFrameSize(stream->format, stream->width, stream->height,
                                                      false, false, false);
    stream->memType = V4L2_MEMORY_USERPTR;
    stream->streamType = icamera::CAMERA_STREAM_OUTPUT;
    // CAMERA_STREAM_PREVIEW is for user preview stream
    // CAMERA_STREAM_VIDEO_CAPTURE is for other yuv stream
    if (camera3Stream.format == HAL_PIXEL_FORMAT_YCbCr_420_888 &&
        IS_STILL_USAGE(camera3Stream.usage)) {
        stream->usage = icamera::CAMERA_STREAM_STILL_CAPTURE;
    } else if (camera3Stream.usage & (GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_COMPOSER)) {
        stream->usage = icamera::CAMERA_STREAM_PREVIEW;
    } else if (IS_ZSL_USAGE(camera3Stream.usage)) {
        stream->usage = icamera::CAMERA_STREAM_OPAQUE_RAW;
    } else {
        if (camera3Stream.format == HAL_PIXEL_FORMAT_BLOB) {
            stream->usage = icamera::CAMERA_STREAM_STILL_CAPTURE;
#ifdef IPU_SYSVER_ipu6v3
            // When enable GPU tnr, JSL use video pipe to output BLOB stream for small resolutions
            if (stream->width * stream->height <=
                    RESOLUTION_1080P_WIDTH * RESOLUTION_1080P_HEIGHT &&
                icamera::PlatformData::isGpuTnrEnabled(cameraId)) {
                stream->usage = icamera::CAMERA_STREAM_VIDEO_CAPTURE;
            }
#endif
        } else {
            stream->usage = icamera::CAMERA_STREAM_VIDEO_CAPTURE;
        }
    }

    if (getRotationDegrees(camera3Stream) <= 0) {
        const icamera::camera_resolution_t* psl =
            icamera::PlatformData::getPreferOutput(stream->width, stream->height, cameraId);

        if (psl) {
            stream->width = psl->width;
            stream->height = psl->height;
            LOG1("%s, Use the psl output %dx%d to map user's stream: %dx%d",
                 __func__, stream->width, stream->height, camera3Stream.width,
                 camera3Stream.height);
        }
    }

    LOG2("@%s, stream: width:%d, height:%d, usage %d", __func__, stream->width, stream->height,
         stream->usage);
    return icamera::OK;
}

int getCrosConfigCameraNumber(const std::vector<CrosCameraInfo>& infos) {
    return infos.size();
}

// Only when /camera/devices exists in CrOS config on the board, it returns the
// cros camera info vector, otherwise it returns empty vector.
std::vector<CrosCameraInfo> getCrosCameraInfo() {
    brillo::CrosConfig crosConfigReader;
    std::vector<CrosCameraInfo> infos;
    int cameraIdx = 0;

    std::string modelName;
    if (crosConfigReader.GetString("/", "name", &modelName)) {
        LOGI("%s, model name %s", __func__, modelName.c_str());
        icamera::PlatformData::setBoardName(modelName);
    }

    // Get MIPI camera info from "devices" array in Chrome OS config. The structure looks like:
    //     camera - devices + 0 + interface (mipi, usb)
    //                      |   + facing (front, back)
    //                      |   + orientation (0, 90, 180, 270)
    //                      |   ...
    //                      + 1 + interface
    //                          ...
    while (true) {
        std::string interfaceType;
        if (!crosConfigReader.GetString(base::StringPrintf("/camera/devices/%i", cameraIdx),
                                        "interface", &interfaceType)) {
            break;
        }
        if (interfaceType == "mipi") {
            std::string orient;
            std::string facing;
            CrosCameraInfo info = {0, 0};

            if (crosConfigReader.GetString(base::StringPrintf("/camera/devices/%i", cameraIdx),
                                           "orientation", &orient)) {
                info.orientation = atoi(orient.c_str());
            }
            if (crosConfigReader.GetString(base::StringPrintf("/camera/devices/%i", cameraIdx),
                                           "facing", &facing)) {
                if (facing == "front") {
                    info.facing = icamera::FACING_FRONT;
                } else {
                    info.facing = icamera::FACING_BACK;
                }
            }
            LOG2("%s, cameraIdx: %d, facing: %d, str: %s, orientation: %d", __func__,
                 cameraIdx, info.facing, facing.c_str(), info.orientation);
            infos.push_back(info);
        }
        ++cameraIdx;
    }
    return infos;
}

void updateOrientation(const std::vector<CrosCameraInfo>& infos) {
    for (size_t i = 0; i < infos.size(); ++i) {
        for (size_t j = 0; j < infos.size(); ++j) {
            icamera::camera_info_t info;
            int ret = icamera::get_camera_info(j, info);
            if (ret != icamera::OK) continue;

            if (info.facing == infos[i].facing) {
                LOG1("%s@, cameraID %zu's orient = %d ", __func__, j, infos[i].orientation);
                icamera::PlatformData::setSensorOrientation(j, infos[i].orientation);
            }
        }
    }
}

// whitelevel is equal to 2^(sensor bits per pixel)-1.
int32_t getDefaultRawWhiteLevel(int cameraId) {
    int format = icamera::PlatformData::getISysRawFormat(cameraId);
    // Use 10 bits as default.
    int32_t whiteLevel = 1023;

    switch (format) {
        case V4L2_PIX_FMT_SBGGR8:
        case V4L2_PIX_FMT_SGBRG8:
        case V4L2_PIX_FMT_SGRBG8:
        case V4L2_PIX_FMT_SRGGB8:
            whiteLevel = 255;
            break;
        case V4L2_PIX_FMT_SBGGR10:
        case V4L2_PIX_FMT_SGBRG10:
        case V4L2_PIX_FMT_SGRBG10:
        case V4L2_PIX_FMT_SRGGB10:
            whiteLevel = 1023;
            break;
        case V4L2_PIX_FMT_SBGGR12:
        case V4L2_PIX_FMT_SGBRG12:
        case V4L2_PIX_FMT_SGRBG12:
        case V4L2_PIX_FMT_SRGGB12:
            whiteLevel = 4095;
            break;
        default:
            LOGW("%s, Not suppport format %x", __func__, format);
            break;
    }

    return whiteLevel;
}

}  // namespace HalV3Utils
}  // namespace camera3
