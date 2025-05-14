/*
 * Copyright (C) 2017-2024 Intel Corporation
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

#define LOG_TAG RequestManager

#include "RequestManager.h"

#include <hardware/gralloc.h>
#include <linux/videodev2.h>
#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <list>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Errors.h"
#include "FaceDetectionResultCallbackManager.h"
#include "HALv3Utils.h"
#include "ICamera.h"
#include "MetadataConvert.h"
#include "ParameterResult.h"
#include "Parameters.h"
#include "PlatformData.h"
#include "Utils.h"

namespace camera3 {
#define VGA_360P 360

RequestManager::RequestManager(int cameraId)
        : mCameraId(cameraId),
          mPrivacyControl(nullptr),
          mSwPrivacyMode(false),
          mPrivacyStarted(false),
          mCallbackOps(nullptr),
          mCameraDeviceStarted(false),
          mResultProcessor(nullptr),
          mParameterResult(nullptr),
          mInputStreamConfigured(false),
          mRequestInProgress(0),
          mFDStream(nullptr),
          mFDStreamIndex(-1) {
    LOG1("@%s", __func__);

    mParameterResult = new ParameterResult(cameraId);

    CLEAR(mCameraBufferInfo);
    CLEAR(mFDStreamsInfo);
}

RequestManager::~RequestManager() {
    LOG1("@%s", __func__);

    deleteStreams();

    delete mResultProcessor;
    delete mParameterResult;
}

int RequestManager::init(const camera3_callback_ops_t* callback_ops) {
    LOG1("@%s", __func__);

    /* If it supports one more video output to process face detection */
    if (icamera::PlatformData::isIPUSupportFD(mCameraId)) {
        mFDStreamsInfo.stream_type = CAMERA3_STREAM_OUTPUT;
        mFDStreamsInfo.width = RESOLUTION_VGA_WIDTH;
        mFDStreamsInfo.height = RESOLUTION_VGA_HEIGHT;
    }

    // Update the default settings from camera HAL
    icamera::Parameters parameter;
    int ret = icamera::camera_get_parameters(mCameraId, parameter);
    CheckAndLogError(ret != icamera::OK, ret, "failed to get parameters, ret %d", ret);

    android::CameraMetadata defaultRequestSettings;
    // Get static metadata
    MetadataConvert::HALCapabilityToStaticMetadata(parameter, &defaultRequestSettings, mCameraId);

    // Get defalut settings
    MetadataConvert::constructDefaultMetadata(mCameraId, &defaultRequestSettings);
    MetadataConvert::HALMetadataToRequestMetadata(parameter, &defaultRequestSettings, mCameraId);

    mDefaultRequestSettings[CAMERA3_TEMPLATE_PREVIEW] = defaultRequestSettings;
    MetadataConvert::updateDefaultRequestSettings(
        mCameraId, CAMERA3_TEMPLATE_PREVIEW, &mDefaultRequestSettings[CAMERA3_TEMPLATE_PREVIEW]);

    mResultProcessor = new ResultProcessor(mCameraId, callback_ops, this, mParameterResult);
    mCallbackOps = callback_ops;

    mPrivacyControl = std::make_unique<PrivacyControl>(mCameraId, callback_ops, this);
    // Register callback to icamera HAL
    icamera::camera_callback_ops_t::notify = RequestManager::callbackNotify;
    icamera::camera_callback_register(mCameraId,
                                      static_cast<icamera::camera_callback_ops_t*>(this));

    return icamera::OK;
}

int RequestManager::deinit() {
    LOG1("@%s", __func__);

    // Unregister callback to icamera HAL
    icamera::camera_callback_register(mCameraId, nullptr);

    if (mCameraDeviceStarted) {
        int ret = icamera::camera_device_stop(mCameraId);
        CheckAndLogError(ret != icamera::OK, ret, "failed to stop camera device, ret %d", ret);
        mCameraDeviceStarted = false;
    }

    uint32_t i = 0;
    while (i < mCamera3StreamVector.size()) {
        Camera3Stream* s = mCamera3StreamVector.at(i);
        s->drainAllPendingRequests();
        i++;
    }

    waitAllRequestsDone();
    LOG1("@%s done", __func__);

    if (mPrivacyStarted) {
        mPrivacyControl->stop();
        mPrivacyStarted = false;
    }

    return icamera::OK;
}

void RequestManager::callbackNotify(const icamera::camera_callback_ops* cb,
                                    const icamera::camera_msg_data_t& data) {
    LOG2("@%s, type %d", __func__, data.type);
    RequestManager* callback = const_cast<RequestManager*>(static_cast<const RequestManager*>(cb));

    callback->mResultProcessor->callbackNotify(data);
    callback->handleCallbackEvent(data);
}

void RequestManager::handleCallbackEvent(const icamera::camera_msg_data_t& data) {
    LOG2("<id%d>@%s", mCameraId, __func__);

    if (!icamera::PlatformData::swProcessingAlignWithIsp(mCameraId)) return;

    for (auto& stream : mCamera3HALStreams) {
        if (stream->getPostProcessType() != icamera::POST_PROCESS_NONE) {
            stream->sendEvent(data);
        }
    }
}

int RequestManager::analysisStreamList(camera3_stream_configuration_t* stream_list,
                                       camera3_stream_t** inputStream) {
    int ret = checkStreamRotation(stream_list);
    CheckAndLogError(ret != icamera::OK, icamera::BAD_VALUE, "Unsupport rotation degree!");

    uint32_t streamsNum = stream_list->num_streams;
    uint32_t operationMode = stream_list->operation_mode;
    LOG1("@%s, streamsNum:%d, operationMode:%d", __func__, streamsNum, operationMode);
    CheckAndLogError(streamsNum > kMaxStreamNum, icamera::BAD_VALUE,
                     "Number of streams %u exceeds limit %d", streamsNum, kMaxStreamNum);
    CheckAndLogError((operationMode != CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE &&
                      operationMode != CAMERA3_STREAM_CONFIGURATION_CONSTRAINED_HIGH_SPEED_MODE),
                     icamera::BAD_VALUE, "Unknown operation mode %d!", operationMode);

    int inputStreamNum = 0;
    int outStreamNum = 0;
    camera3_stream_t* stream = nullptr;

    for (uint32_t i = 0; i < streamsNum; i++) {
        stream = stream_list->streams[i];
        LOG1("@%s, Config stream (%s):%dx%d, f:%d, u:%d, buf num:%d, priv:%p", __func__,
             HalV3Utils::getCamera3StreamType(stream->stream_type), stream->width, stream->height,
             stream->format, stream->usage, stream->max_buffers, stream->priv);
        /*
         * 1, for CAMERA3_STREAM_INPUT stream, format YCbCr_420_888 is for YUV
         * reprocessing, other formats (like IMPLEMENTATION_DEFINED, RAW_OPAQUE) are
         * for RAW reprocessing.
         * 2, for CAMERA3_STREAM_BIDIRECTIONAL stream, it is for RAW reprocessing.
         * 3, for CAMERA3_STREAM_OUTPUT stream, if format is IMPLEMENTATION_DEFINED
         * and usage doesn't include COMPOSE or TEXTURE, it is for RAW reprocessing.
         * if format is RAW_OPAQUE, it is for RAW reprocessing.
         */
        if (stream_list->streams[i]->stream_type == CAMERA3_STREAM_INPUT) {
            inputStreamNum++;
            mInputStreamConfigured = true;
            stream_list->streams[i]->usage |=
                GRALLOC_USAGE_HW_CAMERA_WRITE | GRALLOC_USAGE_SW_READ_OFTEN;
            if (stream_list->streams[i]->format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
                *inputStream = stream_list->streams[i];
            } else {
                stream_list->streams[i]->usage |= GRALLOC_USAGE_HW_CAMERA_ZSL;
            }
            stream_list->streams[i]->max_buffers = 2;
        } else if (stream_list->streams[i]->stream_type == CAMERA3_STREAM_BIDIRECTIONAL) {
            inputStreamNum++;
            outStreamNum++;
            mInputStreamConfigured = true;
            stream_list->streams[i]->usage |= GRALLOC_USAGE_HW_CAMERA_ZSL;
        } else if (stream->stream_type == CAMERA3_STREAM_OUTPUT) {
            outStreamNum++;
            if (stream_list->streams[i]->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED &&
                mInputStreamConfigured) {
                if (!(stream_list->streams[i]->usage &
                      (GRALLOC_USAGE_HW_COMPOSER | GRALLOC_USAGE_HW_TEXTURE))) {
                    stream_list->streams[i]->usage |= GRALLOC_USAGE_HW_CAMERA_ZSL;
                }
            } else if (stream_list->streams[i]->format == HAL_PIXEL_FORMAT_RAW_OPAQUE) {
                stream_list->streams[i]->usage |= GRALLOC_USAGE_HW_CAMERA_ZSL;
            }
        } else {
            LOGE("@%s, Unknown stream type %d!", __func__, stream->stream_type);
            return icamera::BAD_VALUE;
        }
        // In ZSL case, RAW input and YUV input will be configured together.
        CheckAndLogError(inputStreamNum > 2, icamera::BAD_VALUE, "Too many input streams : %d !",
                         inputStreamNum);
    }
    CheckAndLogError(outStreamNum == 0, icamera::BAD_VALUE, "No output streams!");

    return icamera::OK;
}

int RequestManager::createHALStreams(camera3_stream_configuration_t* stream_list,
                                     int* halStreamFlag, int* halStreamNum,
                                     std::map<int, int>& streamToHALIndex,
                                     camera3_stream_t** userStreamForFd) {
    int ret;
    int outputStreamNum = 0;
    // Enable video pipe if yuv stream exists (for 3A stats data)
    bool needAssignPreviewStream = true;
    icamera::stream_t* yuvStream = nullptr;
    icamera::stream_t* blobStream = nullptr;
    uint32_t streamsNum = stream_list->num_streams;
    icamera::stream_t outputReqStreams[kMaxStreamNum];
    for (uint32_t i = 0; i < streamsNum; i++) {
        if (stream_list->streams[i]->stream_type == CAMERA3_STREAM_INPUT) continue;

        ret = HalV3Utils::fillHALStreams(mCameraId, *stream_list->streams[i],
                                         &outputReqStreams[outputStreamNum]);
        CheckAndLogError(ret != icamera::OK, ret, "failed to fill requestStreams[%d], ret:%d", ret,
                         outputStreamNum);

        if (stream_list->streams[i]->format == HAL_PIXEL_FORMAT_BLOB) {
            blobStream = &outputReqStreams[outputStreamNum];
        } else if (!yuvStream && !IS_ZSL_USAGE(stream_list->streams[i]->usage)) {
            yuvStream = &outputReqStreams[outputStreamNum];
        }

        if (outputReqStreams[outputStreamNum].usage == icamera::CAMERA_STREAM_PREVIEW ||
            outputReqStreams[outputStreamNum].usage == icamera::CAMERA_STREAM_VIDEO_CAPTURE)
            needAssignPreviewStream = false;

        outputStreamNum++;
    }
    if (needAssignPreviewStream) {
        if (yuvStream) {
            yuvStream->usage = icamera::CAMERA_STREAM_PREVIEW;
        } else if (blobStream) {
            blobStream->usage = icamera::CAMERA_STREAM_PREVIEW;
        }
    }

    CLEAR(mHALStream);
    *halStreamNum = chooseHALStreams(outputStreamNum, halStreamFlag, outputReqStreams);
    // halStreamNum should not exceed videoNum + 2 (1 opaque raw and 1 still)
    int maxSupportStreamNum = icamera::PlatformData::getVideoStreamNum() + 2;
    CheckAndLogError(*halStreamNum > maxSupportStreamNum || *halStreamNum <= 0, icamera::BAD_VALUE,
                     "failed to find HAL stream");

    // index of stream in mHALStream
    int halStreamIndex = 0;
    int videoStreamNum = 0;
    for (int i = 0; i < outputStreamNum; i++) {
        // fill HAL stream with right requestStreams object
        if (halStreamFlag[i] == i) {
            if (outputReqStreams[i].usage == icamera::CAMERA_STREAM_PREVIEW ||
                outputReqStreams[i].usage == icamera::CAMERA_STREAM_VIDEO_CAPTURE) {
                videoStreamNum++;
            }
            mHALStream[halStreamIndex] = outputReqStreams[i];
            streamToHALIndex[i] = halStreamIndex;
            halStreamIndex++;
        }
    }

    mFDStreamIndex = -1;
    /* If it supports one more video output to process face detection */
    bool isFaceEngineEnabled = icamera::PlatformData::isFaceDetectionSupported(mCameraId);
    // Mark one camera3Stream run face detection
    if (isFaceEngineEnabled) {
        LOG1("Face detection is enable");
        *userStreamForFd = choosePreviewStream(streamsNum, stream_list->streams);
        bool isIPUSupportFD = icamera::PlatformData::isIPUSupportFD(mCameraId) &&
                              (videoStreamNum < icamera::PlatformData::getVideoStreamNum());

        // Don't use additional stream for face detection as it will not be used when we
        // use external FD result, e.g. from FaceDetectionStreamManipulator.
        bool isUsingExternalFDResult =
            camera3::FaceDetectionResultCallbackManager::getInstance().isUsingExternalFDResult(
                mCameraId);

        if (*userStreamForFd && isIPUSupportFD && !isUsingExternalFDResult) {
            if (RESOLUTION_VGA_WIDTH * (*userStreamForFd)->height !=
                (*userStreamForFd)->width * RESOLUTION_VGA_HEIGHT) {
                mFDStreamsInfo.height = VGA_360P;
            } else {
                mFDStreamsInfo.height = RESOLUTION_VGA_HEIGHT;
            }
            // Try to add hal stream for face in the back of mHALStream
            mFDStreamsInfo.usage = (*userStreamForFd)->usage;
            mFDStreamsInfo.format = (*userStreamForFd)->format;
            CLEAR(mHALStream[*halStreamNum]);
            ret = HalV3Utils::fillHALStreams(mCameraId, mFDStreamsInfo, &mHALStream[*halStreamNum]);
            CheckAndLogError(ret != icamera::OK, ret, "failed to fill stream for fd ret:%d", ret);
            // Always bind ipu face detection to video pipe
            mHALStream[*halStreamNum].usage = icamera::CAMERA_STREAM_PREVIEW;

            icamera::stream_config_t streamCfg = {
                (*halStreamNum) + 1, mHALStream,
                icamera::camera_stream_configuration_mode_t::CAMERA_STREAM_CONFIGURATION_MODE_AUTO};
            ret = icamera::PlatformData::queryGraphSettings(mCameraId, &streamCfg);
            // Check if stream is supported or not
            bool isValid =
                icamera::PlatformData::isSupportedStream(mCameraId, mHALStream[*halStreamNum]);
            // Use mFDStreamIndex and userStreamForFd to indicate IPU output or user stream
            // for face detection
            if (ret == icamera::OK && isValid) {
                mFDStreamIndex = *halStreamNum;
                *userStreamForFd = nullptr;
                (*halStreamNum)++;
                LOG1("@%s, stream info for FD: %dx%d, f:%d, u:%d, index: %d", __func__,
                     mFDStreamsInfo.width, mFDStreamsInfo.height, mFDStreamsInfo.format,
                     mFDStreamsInfo.usage, mFDStreamIndex);
            }
        }
    }

    for (int i = 0; i < outputStreamNum; i++) {
        const icamera::stream_t& s = outputReqStreams[i];
        LOG1("@%s, requestStreams[%d]: w:%d, h:%d, f:%d, u:%d", __func__, i, s.width, s.height,
             s.format, s.usage);
    }

    for (int i = 0; i < *halStreamNum; i++) {
        const icamera::stream_t& s = mHALStream[i];
        LOG1("@%s, configured mHALStream[%d]: w:%d, h:%d, f:%d, u:%d", __func__, i, s.width,
             s.height, s.format, s.usage);
    }

    return icamera::OK;
}

int RequestManager::deviceConfigHALStreams(int halStreamNum) {
    CLEAR(mStreamCfg);
    mStreamCfg = {halStreamNum, mHALStream,
        icamera::camera_stream_configuration_mode_t::CAMERA_STREAM_CONFIGURATION_MODE_AUTO};

    int ret = icamera::camera_device_config_streams(mCameraId, &mStreamCfg);
    CheckAndLogError(ret != icamera::OK, ret, "failed to configure stream, ret %d", ret);

    return icamera::OK;
}

void RequestManager::createCamera3Stream(camera3_stream_configuration_t* stream_list,
                                         std::map<int, int>& streamToHALIndex, int* halStreamFlag,
                                         camera3_stream_t* inputReqStream,
                                         camera3_stream_t* userStreamForFd) {
    deleteStreams();
    int requestStreamIndex = 0;
    Camera3Stream* faceDetectionOwner = nullptr;
    // <listener stream, owner HAL stream index>
    std::map<Camera3StreamListener*, int32_t> listenerStreamOwners;
    std::map<int32_t, Camera3StreamHAL*> halStreams;  // <hal stream index, hal stream>
    for (uint32_t i = 0; i < stream_list->num_streams; i++) {
        camera3_stream_t* stream = stream_list->streams[i];
        if (stream->stream_type == CAMERA3_STREAM_INPUT) {
            continue;
        }

        /* use halStreamFlag[] to find it's HAL stream index in requestStreams
        ** streamToHALIndex to find it's HAL Stream index in mHALStream[]*/
        int halStreamIndex = streamToHALIndex[halStreamFlag[requestStreamIndex]];
        bool isHALStream = halStreamFlag[requestStreamIndex] == requestStreamIndex;
        Camera3Stream* s = nullptr;
        if (isHALStream) {
            Camera3StreamHAL* hs =
                new Camera3StreamHAL(mCameraId, mResultProcessor, mParameterResult,
                                     mHALStream[halStreamIndex].max_buffers,
                                     mHALStream[halStreamIndex], *stream, inputReqStream);
            mCamera3HALStreams.push_back(hs);
            halStreams[halStreamIndex] = hs;
            s = reinterpret_cast<Camera3Stream*>(hs);
        } else {
            Camera3StreamListener* ls =
                new Camera3StreamListener(mCameraId, mResultProcessor, mParameterResult,
                                          mHALStream[halStreamIndex].max_buffers,
                                          mHALStream[halStreamIndex], *stream, inputReqStream);
            listenerStreamOwners[ls] = halStreamIndex;
            s = reinterpret_cast<Camera3Stream*>(ls);
        }
        s->setActive(true);
        stream->priv = s;
        stream->max_buffers = mHALStream[halStreamIndex].max_buffers;
        stream->usage |= GRALLOC_USAGE_HW_CAMERA_WRITE | GRALLOC_USAGE_SW_READ_OFTEN |
                         GRALLOC_USAGE_SW_WRITE_NEVER;
        mCamera3StreamVector.push_back(s);

        if (stream->format == HAL_PIXEL_FORMAT_BLOB) {
            mPrivacyControl->configure(mHALStream[halStreamIndex], *stream);
        }

        requestStreamIndex++;
        LOGI("OUTPUT max buffer %d, usage %x, format %x, %dx%d", stream->max_buffers, stream->usage,
             stream->format, stream->width, stream->height);

        if (userStreamForFd == stream) {
            faceDetectionOwner = static_cast<Camera3Stream*>(stream->priv);
            LOG1("@%s, FD chooses stream information:format=%d, width=%d, height=%d", __func__,
                 stream->format, stream->width, stream->height);
        }
    }

    // bind streams to HAL streams
    for (auto& iter : listenerStreamOwners) {
        if (halStreams.find(iter.second) != halStreams.end()) {
            halStreams[iter.second]->addListener(iter.first);
        }
    }

    bool isFaceEngineEnabled = icamera::PlatformData::isFaceDetectionSupported(mCameraId);
    if (isFaceEngineEnabled) {
        if (mFDStreamIndex > 0) {
            // Use ipu output for face detection
            mFDStream = new PrivateStreamFD(mCameraId, mHALStream[mFDStreamIndex], mFDStreamsInfo);
            LOG1("@%s, Create PrivateStreamFD for face detection", __func__);
        } else if (faceDetectionOwner) {
            // Use user stream for face detection
            unsigned int maxFacesNum = icamera::PlatformData::getMaxFaceDetectionNumber(mCameraId);
            faceDetectionOwner->activateFaceDetection(maxFacesNum);
            LOG1("@%s, Create Camera3Stream for face detection", __func__);
        }
    }

}

int RequestManager::configureStreams(camera3_stream_configuration_t* stream_list) {
    LOG1("@%s", __func__);

    camera3_stream_t* inputReqStream = nullptr;
    int ret = analysisStreamList(stream_list, &inputReqStream);
    CheckAndLogError(ret != icamera::OK, ret, "stream list are not invalid");

    if (mCameraDeviceStarted) {
        ret = icamera::camera_device_stop(mCameraId);
        CheckAndLogError(ret != icamera::OK, ret, "failed to stop camera device, ret %d", ret);
        mCameraDeviceStarted = false;
    }

    mResultProcessor->clearRawBufferInfoMap();

    /*
     * Configure stream
     * 1.create HAL streams
     * 2.config HAL streams
     * 3.create camera3Stream
     */
    int halStreamFlag[kMaxStreamNum];
    int halStreamNum = 0;
    camera3_stream_t* userStreamForFd = nullptr;
    // first:stream index in requestStreams[], second:HAL stream index in mHALStream[]
    std::map<int, int> streamToHALIndex;
    ret = createHALStreams(stream_list, halStreamFlag, &halStreamNum, streamToHALIndex,
                           &userStreamForFd);
    CheckAndLogError(ret != icamera::OK, ret, "Failed to create HAL stream");

    ret = deviceConfigHALStreams(halStreamNum);
    CheckAndLogError(ret != icamera::OK, ret, "Failed to config HAL stream");

    createCamera3Stream(stream_list, streamToHALIndex, halStreamFlag, inputReqStream,
                        userStreamForFd);
    mSwPrivacyMode = false;

    bool videoOnly = true;
    for (int i = 0; i < halStreamNum; i++) {
        if (mHALStream[i].usage != icamera::CAMERA_STREAM_PREVIEW &&
            mHALStream[i].usage != icamera::CAMERA_STREAM_VIDEO_CAPTURE) {
            videoOnly = false;
        }
    }
    mResultProcessor->videoPipeOnly(videoOnly);

    return icamera::OK;
}

camera3_stream_t* RequestManager::choosePreviewStream(uint32_t streamsNum,
                                                      camera3_stream_t** streams) {
    LOG1("@%s", __func__);
    camera3_stream_t* preStream = nullptr;
    camera3_stream_t* yuvStream = nullptr;
    int maxWidth = MAX_FACE_FRAME_WIDTH;
    int maxHeight = MAX_FACE_FRAME_HEIGHT;
    int preStreamNum = -1;
    int yuvStreamNum = -1;
    int fdVendor = icamera::PlatformData::faceEngineVendor(mCameraId);

    for (uint32_t i = 0; i < streamsNum; i++) {
        camera3_stream_t* s = streams[i];
        if (!s || s->stream_type != CAMERA3_STREAM_OUTPUT ||
            ((s->width > maxWidth || s->height > maxHeight) &&
             (fdVendor != FACE_ENGINE_GOOGLE_FACESSD))) {
            continue;
        }

        LOG1("%s, stream information:format=%d, width=%d, height=%d", __func__, s->format, s->width,
             s->height);
        // We assume HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED stream is the
        // preview stream and it's requested in every capture request.
        // If there are two HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED streams,
        // We pick the smaller stream due to performance concern.
        if (s->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED && !IS_ZSL_USAGE(s->usage)) {
            if (preStream && preStream->width * preStream->height <= s->width * s->height) {
                continue;
            }
            preStream = s;
            preStreamNum = i;
        }

        if (s->format == HAL_PIXEL_FORMAT_YCbCr_420_888 && !IS_STILL_USAGE(s->usage)) {
            if (yuvStream && yuvStream->width * yuvStream->height <= s->width * s->height) {
                continue;
            }
            yuvStream = s;
            yuvStreamNum = i;
        }
    }

    if (preStreamNum >= 0) {
        return preStream;
    } else if (yuvStreamNum >= 0) {
        return yuvStream;
    }

    return nullptr;
}

int RequestManager::constructDefaultRequestSettings(int type, const camera_metadata_t** meta) {
    LOG1("@%s, type %d", __func__, type);

    if (mDefaultRequestSettings.count(type) == 0) {
        mDefaultRequestSettings[type] = mDefaultRequestSettings[CAMERA3_TEMPLATE_PREVIEW];
        MetadataConvert::updateDefaultRequestSettings(mCameraId, type,
                                                      &mDefaultRequestSettings[type]);
    }
    const camera_metadata_t* setting = mDefaultRequestSettings[type].getAndLock();
    *meta = setting;
    mDefaultRequestSettings[type].unlock(setting);

    return icamera::OK;
}

int RequestManager::handlePrivacySwitch(camera3_capture_request_t* request, bool* privacyMode) {
    bool stateChanged = false;
    *privacyMode = PrivacyControl::getPrivacyMode();

    if (*privacyMode != mSwPrivacyMode) {
        stateChanged = true;
        mSwPrivacyMode = *privacyMode;
    }
    LOG2("%s, privacyMode: %d, frameNumber: %d, stateChanged: %d", __func__,
         *privacyMode, request->frame_number, stateChanged);

    if (*privacyMode) {
        mPrivacyControl->processCaptureRequest(request);
    }

    if (stateChanged) {
        if (*privacyMode) {
            waitAllRequestsDone();

            // start PrivacyControl
            if (!mPrivacyStarted) {
                mPrivacyControl->start();
                mPrivacyStarted = true;
                LOG2("%s, Start the sw privacy mode", __func__);
            }

            // stop CameraDevice
            if (mCameraDeviceStarted) {
                icamera::camera_device_stop(mCameraId);
                mCameraDeviceStarted = false;
                LOG2("%s, Stop the physical camera device", __func__);
            }
        } else {
            LOG2("%s, Re-Configure device for privacy mode: ON -> OFF. privacyMode: %d",
                 __func__, *privacyMode);
            int ret = icamera::camera_device_config_streams(mCameraId, &mStreamCfg);
            CheckAndLogError(ret != icamera::OK, ret, "failed to re-configure stream");

            waitAllRequestsDone();

            // stop privacy mode
            if (mPrivacyStarted) {
                mPrivacyControl->stop();
                mPrivacyStarted = false;
                LOG2("%s, Stop the sw priority mode", __func__);
            }
        }
    }

    return icamera::OK;
}

int RequestManager::processCaptureRequest(camera3_capture_request_t* request) {
    CheckAndLogError(!request, icamera::UNKNOWN_ERROR, "@%s, request is nullptr", __func__);
    LOG2("<fn%u>@%s, input_buffer:%d, num_output_buffers:%d", request->frame_number, __func__,
         request->input_buffer ? 1 : 0, request->num_output_buffers);

    TRACE_LOG_POINT("RequestManager", __func__, MAKE_COLOR(request->frame_number),
                    request->frame_number);

    // Valid buffer and request
    CheckAndLogError(request->num_output_buffers > kMaxStreamNum, icamera::BAD_VALUE,
                     "@%s, num_output_buffers:%d", __func__, request->num_output_buffers);

    waitProcessRequest();

    if (request->settings) {
        MetadataConvert::dumpMetadata(request->settings);
        mLastSettings = request->settings;
    } else if (mLastSettings.isEmpty()) {
        LOGE("nullptr settings for the first reqeust!");
        return icamera::BAD_VALUE;
    }

    bool privacyMode;
    int ret = handlePrivacySwitch(request, &privacyMode);
    CheckAndLogError(ret != icamera::OK, icamera::UNKNOWN_ERROR,
                     "@%s, failed to handle privacy mode switch", __func__);
    if (privacyMode) {
        increaseRequestCount();
        return icamera::OK;
    }

    int index = -1;
    for (int i = 0; i < kMaxProcessRequestNum; i++) {
        if (!mCameraBufferInfo[i].frameInProcessing) {
            index = i;
            break;
        }
    }
    CheckAndLogError(index < 0, icamera::UNKNOWN_ERROR, "<fn%u>no empty CameraBufferInfo!",
                     request->frame_number);
    CLEAR(mCameraBufferInfo[index]);

    std::shared_ptr<Camera3Buffer> inputCam3Buf = nullptr;
    icamera::sensor_data_info_t opaqueRawInfo = {-1, 0};
    if (request->input_buffer) {
        CheckAndLogError(request->input_buffer->status == CAMERA3_BUFFER_STATUS_ERROR,
                         icamera::BAD_VALUE, "Error status for input camera3 buffer");

        inputCam3Buf = std::make_shared<Camera3Buffer>();
        icamera::status_t status = inputCam3Buf->init(request->input_buffer, mCameraId);
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to init CameraBuffer");
        status = inputCam3Buf->waitOnAcquireFence();
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to sync CameraBuffer");
        status = inputCam3Buf->lock();
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to lock buffer");

        camera_metadata_entry entry = mLastSettings.find(ANDROID_SENSOR_TIMESTAMP);
        if (entry.count == 1) {
            inputCam3Buf->setTimeStamp(entry.data.i64[0]);
            opaqueRawInfo.timestamp = entry.data.i64[0];
        }

        if (IS_ZSL_USAGE(request->input_buffer->stream->usage)) {
            mResultProcessor->checkAndChangeRawbufferInfo(&opaqueRawInfo.sequence,
                                                          &opaqueRawInfo.timestamp);
            LOG2("%s, sequence id %ld, timestamp %ld", __func__, opaqueRawInfo.sequence,
                 opaqueRawInfo.timestamp);
        }
    }

    icamera::Parameters param;
    param.setMakernoteMode(icamera::MAKERNOTE_MODE_OFF);
    param.setUserRequestId(static_cast<int32_t>(request->frame_number));

    for (uint32_t i = 0; i < request->num_output_buffers; i++) {
        CheckAndLogError(request->output_buffers[i].status == CAMERA3_BUFFER_STATUS_ERROR,
                         icamera::BAD_VALUE, "%s, Error status for output camera3 buffer: %u",
                         __func__, i);

        camera3_stream_t* aStream = request->output_buffers[i].stream;        // app stream
        Camera3Stream* lStream = static_cast<Camera3Stream*>(aStream->priv);  // local stream
        if (icamera::PlatformData::isEnableMkn(mCameraId) &&
            (mInputStreamConfigured || aStream->format == HAL_PIXEL_FORMAT_BLOB)) {
            param.setMakernoteMode(icamera::MAKERNOTE_MODE_JPEG);
        }

        ret = lStream->processRequest(request->frame_number,
                                      opaqueRawInfo.sequence >= 0 ? nullptr : inputCam3Buf,
                                      request->output_buffers[i], param);
        CheckAndLogError(ret != icamera::OK, ret, "Failed to process request, ret:%d", ret);
    }

    // Convert metadata to Parameters
    bool forceConvert = inputCam3Buf ? true : false;
    MetadataConvert::requestMetadataToHALMetadata(mLastSettings, &param, forceConvert);

    mResultProcessor->registerRequest(request, inputCam3Buf);
    icamera::camera_buffer_t* faceDetectionBuf = nullptr;
    if (!inputCam3Buf || opaqueRawInfo.sequence >= 0) {
        icamera::camera_buffer_t* buffer[kMaxStreamNum] = {nullptr};
        int numBuffers = 0;
        bool hasNotRaw = false;
        for (auto& stream : mCamera3HALStreams) {
            if (stream->fetchRequestBuffers(&mCameraBufferInfo[index].halBuffer[numBuffers],
                                            request->frame_number)) {
                mCameraBufferInfo[index].halBuffer[numBuffers].sequence = opaqueRawInfo.sequence;
                mCameraBufferInfo[index].halBuffer[numBuffers].timestamp = opaqueRawInfo.timestamp;
                buffer[numBuffers] = &mCameraBufferInfo[index].halBuffer[numBuffers];
                if (mCameraBufferInfo[index].halBuffer[numBuffers].s.usage !=
                    icamera::CAMERA_STREAM_OPAQUE_RAW) {
                    hasNotRaw = true;
                }
                numBuffers++;
            }
        }

        // it supports one more video output to process face detection, it should trigger
        // the corresponding camera3Stream run. Only when bufferFd is not null, it will be processed
        // Don't set FD buffer in cases, like Raw reprocessing or only Raw output.
        if (mFDStream && opaqueRawInfo.sequence < 0 && hasNotRaw) {
            faceDetectionBuf = mFDStream->fetchRequestBuffers(request->frame_number);
            if (faceDetectionBuf) {
                buffer[numBuffers] = faceDetectionBuf;
                numBuffers++;
                LOG2("%s, find available internal FD buffer for frame_number %d", __func__,
                     request->frame_number);
            }
        }

        ret = icamera::camera_stream_qbuf(mCameraId, buffer, numBuffers, &param);
        CheckAndLogError(ret != icamera::OK, ret, "@%s, camera_stream_qbuf fails,ret:%d", __func__,
                         ret);
    }

    if (!mCameraDeviceStarted) {
        LOGI("Start camera device in processCaptureRequest");
        ret = icamera::camera_device_start(mCameraId);
        CheckAndLogError(ret != icamera::OK, ret, "failed to start device, ret %d", ret);

        mCameraDeviceStarted = true;
    }
    increaseRequestCount();

    mCameraBufferInfo[index].frameInProcessing = true;
    mCameraBufferInfo[index].frameNumber = request->frame_number;

    // Check all streams because the HAL stream might be triggered by listener request
    for (auto& stream : mCamera3StreamVector) {
        stream->queueBufferDone(request->frame_number);
    }
    if (mFDStream && faceDetectionBuf) {
        mFDStream->queueBufferDone(request->frame_number);
    }

    return ret;
}

void RequestManager::dump(int fd) {
    LOG1("@%s", __func__);
}

int RequestManager::flush() {
    LOG1("@%s", __func__);

    const icamera::nsecs_t ONE_SECOND = 1000000000;
    icamera::nsecs_t startTime = icamera::CameraUtils::systemTime();

    waitAllRequestsDone();

    icamera::nsecs_t interval = icamera::CameraUtils::systemTime() - startTime;
    LOG2("@%s, line:%d, time spend:%ld us", __func__, __LINE__, interval / 1000);

    // based on API, -ENODEV (NO_INIT) error should be returned.
    CheckAndLogError(interval > ONE_SECOND, icamera::NO_INIT, "flush() > 1s, timeout:%ld us",
                     interval / 1000);

    return icamera::OK;
}

void RequestManager::deleteStreams() {
    LOG1("@%s", __func__);

    mCamera3HALStreams.clear();
    while (!mCamera3StreamVector.empty()) {
        Camera3Stream* s = mCamera3StreamVector.front();
        mCamera3StreamVector.erase(mCamera3StreamVector.begin());
        delete s;
    }

    if (mFDStream) {
        delete mFDStream;
        mFDStream = nullptr;
    }
}

int RequestManager::waitProcessRequest() {
    LOG2("@%s", __func__);
    std::unique_lock<std::mutex> lock(mRequestLock);
    // check if it is ready to process next request
    while (mRequestInProgress >= mHALStream[0].max_buffers) {
        std::cv_status ret = mRequestCondition.wait_for(
            lock, std::chrono::nanoseconds(kMaxDuration * SLOWLY_MULTIPLIER));
        if (ret == std::cv_status::timeout) {
            LOGW("%s, wait to process request time out", __func__);
            icamera::camera_msg_data_t data = {icamera::CAMERA_DEVICE_ERROR, {}};
            mResultProcessor->callbackNotify(data);
        }
    }

    return icamera::OK;
}

void RequestManager::increaseRequestCount() {
    std::lock_guard<std::mutex> l(mRequestLock);
    ++mRequestInProgress;
}

void RequestManager::returnRequestDone(uint32_t frameNumber) {
    LOG2("<fn%u>@%s", frameNumber, __func__);

    std::lock_guard<std::mutex> l(mRequestLock);

    // Update mCameraBufferInfo based on frameNumber
    for (int i = 0; i < kMaxProcessRequestNum; i++) {
        if (mCameraBufferInfo[i].frameNumber == frameNumber) {
            CLEAR(mCameraBufferInfo[i]);
            break;
        }
    }
    mRequestInProgress--;
    mRequestCondition.notify_one();

    for (auto& stream : mCamera3HALStreams) {
        stream->requestStreamDone(frameNumber);
    }

    if (mRequestInProgress == 0) {
        mWaitAllRequestsDone.notify_one();
    }

    mParameterResult->releaseParameter(frameNumber);
}

int RequestManager::checkStreamRotation(camera3_stream_configuration_t* stream_list) {
    int rotationDegree0 = -1, countOutputStream = 0;

    for (size_t i = 0; i < stream_list->num_streams; i++) {
        if (stream_list->streams[i]->stream_type != CAMERA3_STREAM_OUTPUT) {
            continue;
        }
        countOutputStream++;

        int rotationDegree = HalV3Utils::getRotationDegrees(*(stream_list->streams[i]));
        CheckAndLogError(rotationDegree < 0, icamera::BAD_VALUE, "Unsupport rotation degree!");

        if (countOutputStream == 1) {
            rotationDegree0 = rotationDegree;
        } else {
            CheckAndLogError(rotationDegree0 != rotationDegree, icamera::BAD_VALUE,
                             "rotationDegree0:%d, stream[%lu] rotationDegree:%d, not the same",
                             rotationDegree0, i, rotationDegree);
        }
    }

    return icamera::OK;
}

int RequestManager::chooseHALStreams(const uint32_t requestStreamNum, int* halStreamFlag,
                                     icamera::stream_t* halStreamList) {
    /* save streams with their configure index, the index in this deque is
     * their priority to be HWStream, from low to high
     */
    std::list<std::pair<icamera::stream_t*, int>> videoHALStreamOrder;
    std::list<std::pair<icamera::stream_t*, int>> stillHALStreamOrder;

    // save sorted hal streams with their configure index
    std::vector<std::pair<icamera::stream_t*, int>> requestStreams;

    int videoCount = 0, stillCount = 0, opaqueCount = 0;
    for (uint32_t i = 0; i < requestStreamNum; i++) {
        // set flags to it's index, every stream is a HAL stream by default
        halStreamFlag[i] = i;
        if (halStreamList[i].usage == icamera::CAMERA_STREAM_OPAQUE_RAW) {
            opaqueCount++;
        } else if (halStreamList[i].usage == icamera::CAMERA_STREAM_STILL_CAPTURE) {
            stillCount++;
        } else {
            videoCount++;
        }
    }

    int avaVideoSlot = icamera::PlatformData::getVideoStreamNum();
    // if HAL stream slots are enough, make all streams as HAL stream
    if (opaqueCount <= 1 && stillCount <= 1 && videoCount <= avaVideoSlot) {
        return requestStreamNum;
    }

    for (uint32_t i = 0; i < requestStreamNum; i++) {
        // clear the flags to invalid value
        halStreamFlag[i] = -1;
        requestStreams.push_back(std::make_pair(&halStreamList[i], i));
    }

    // sort stream by resolution from high to low, easy to find largest resolution stream
    std::sort(requestStreams.begin(), requestStreams.end(),
              [](std::pair<icamera::stream_t*, int>& s1, std::pair<icamera::stream_t*, int>& s2) {
                  if (s1.first->width * s1.first->height > s2.first->width * s2.first->height)
                      return true;
                  else
                      return false;
              });

    int activeHALNum = 0;
    bool perfStillIndex = false;
    int selectedVideoNum = 0, videoMaxResStreamIndex = -1;

    // Save the video stream with different resolution
    std::vector<icamera::stream_t*> videoHALStream;
    // where to insert a new mediam priority stream
    auto anchor = videoHALStreamOrder.end();

    for (uint32_t i = 0; i < requestStreamNum; i++) {
        icamera::stream_t* s = requestStreams[i].first;
        int index = requestStreams[i].second;

        if (s->usage == icamera::CAMERA_STREAM_OPAQUE_RAW) {
            // Choose hal stream for opaque stream, only 1 opaque stream
            halStreamFlag[index] = index;
            activeHALNum++;
        } else if (s->usage == icamera::CAMERA_STREAM_STILL_CAPTURE) {
            // Choose hal stream for still streams: same ratio is higher priority
            if (!perfStillIndex && HalV3Utils::isSameRatioWithSensor(s, mCameraId)) {
                perfStillIndex = true;
                stillHALStreamOrder.push_back(std::make_pair(s, index));
            } else {
                stillHALStreamOrder.push_front(std::make_pair(s, index));
            }
        } else {
            /*
             * videoHALStreamOrder store the stream by priority: from low to high
             * {[same resolution], [others(ascending)], [biggest resolution], [same ratio]}
             */
            // iterator video streams in requestStreams by resolution from high to low.
            if (videoMaxResStreamIndex == -1) {
                // insert max resolution stream to end as the first HAL stream candidate
                anchor = videoHALStreamOrder.insert(anchor, std::make_pair(s, index));
                videoMaxResStreamIndex = index;
                videoHALStream.push_back(s);

                // Mark it as one same ratio stream
                if (HalV3Utils::isSameRatioWithSensor(s, mCameraId)) {
                    selectedVideoNum++;
                }
            } else {
                bool found = false;
                for (auto& vs : videoHALStream) {
                    if (vs && s->width == vs->width && s->height == vs->height) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    // Same resolution stream put in the front of list, has lowest priority
                    videoHALStreamOrder.push_front(std::make_pair(s, index));
                } else {
                    if (selectedVideoNum < avaVideoSlot &&
                        HalV3Utils::isSameRatioWithSensor(s, mCameraId)) {
                        // Same ratio is the highest priority, push_back of the list, before other
                        // same ratio stream
                        videoHALStreamOrder.insert(
                            std::prev(videoHALStreamOrder.end(), selectedVideoNum),
                            std::make_pair(s, index));
                        selectedVideoNum++;
                        if (HalV3Utils::isSameRatioWithSensor(anchor->first, mCameraId))
                            anchor = std::prev(anchor);
                    } else {
                        // Other small streams are second priority,
                        // insert them before the biggest stream by ascending
                        anchor = videoHALStreamOrder.insert(anchor, std::make_pair(s, index));
                    }
                    videoHALStream.push_back(s);
                }
            }
        }
    }

    /* Re-process the videoHALStreamOrder, then configure all items to HAL
     * 1. videoHALStreamOrder stores HAL stream candidate with priority
     * from low to high, Remove the extra items in the front of this list
     */
    while (videoHALStreamOrder.size() > avaVideoSlot) {
        videoHALStreamOrder.erase(videoHALStreamOrder.begin());
    }

    LOG1("%s, videoHALStreamOrder size: %zu, stillHALStreamOrder: %zu", __func__,
         videoHALStreamOrder.size(), stillHALStreamOrder.size());

    // 2: set the flags to itself
    for (auto it = videoHALStreamOrder.begin(); it != videoHALStreamOrder.end(); ++it) {
        LOG1("%s, bind itself for video stream index: %d", __func__, it->second);
        halStreamFlag[it->second] = it->second;
        activeHALNum++;
    }

    if (videoHALStreamOrder.size() > 1) {
        float baseRatio = static_cast<float>(videoHALStreamOrder.front().first->width) /
                          videoHALStreamOrder.front().first->height;
        for (auto& vs : videoHALStreamOrder) {
            float streamRatio = static_cast<float>(vs.first->width) / vs.first->height;
            if (streamRatio != baseRatio) {
                LOG1("%s, baseRatio: %f, streamRatio: %f, there is FOV lose", __func__, baseRatio,
                     streamRatio);
            }
        }
    }

    // Select the still stream and its listener
    if (!stillHALStreamOrder.empty()) {
        activeHALNum++;
        for (uint32_t i = 0; i < requestStreamNum; i++) {
            // has only 1 Still HAL stream, other still stream should be listeners
            if (halStreamList[i].usage == icamera::CAMERA_STREAM_STILL_CAPTURE) {
                halStreamFlag[i] = stillHALStreamOrder.back().second;
                LOG1("%s, bind still stream %d, to index: %d", __func__, i,
                     stillHALStreamOrder.back().second);
            }
        }
    }

    // Select other video streams not been put into HAL slot, bind to the HAL stream
    for (uint32_t i = 0; i < requestStreamNum; i++) {
        /* skip the streams which have been selected to HAL
         * still, opaque and avaVideoSlot video streams
         */
        if (halStreamFlag[i] != -1) continue;

        // 1: same resolution
        auto it = videoHALStreamOrder.begin();
        for (; it != videoHALStreamOrder.end(); ++it) {
            if (halStreamList[i].width == it->first->width &&
                halStreamList[i].height == it->first->height) {
                halStreamFlag[i] = it->second;
                break;
            }
        }
        if (it != videoHALStreamOrder.end()) continue;

        // 2: same ratio
        it = videoHALStreamOrder.begin();
        for (; it != videoHALStreamOrder.end(); ++it) {
            if (halStreamList[i].width * it->first->height ==
                halStreamList[i].height * it->first->width) {
                halStreamFlag[i] = it->second;
                break;
            }
        }
        if (it != videoHALStreamOrder.end()) continue;

        // 3: bind to the biggest stream
        /*
         * The videoHALStreamOrder is sorted from small to big. Bind streams to the biggest
         * stream for better IQ when downscaling streams.
         */
        halStreamFlag[i] = videoHALStreamOrder.back().second;
    }

    LOG1("has %d HAL Streams", activeHALNum);
    for (int i = 0; i < requestStreamNum; i++)
        LOG1("user Stream %d bind to HAL Stream %d", i, halStreamFlag[i]);

    return activeHALNum;
}

void RequestManager::waitAllRequestsDone() {
    LOG1("@%s", __func__);

    std::unique_lock<std::mutex> lock(mRequestLock);
    while (mRequestInProgress > 0) {
        mWaitAllRequestsDone.wait(lock);
    }
}

}  // namespace camera3
