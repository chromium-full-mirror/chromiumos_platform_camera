/*
 * Copyright (C) 2017-2022 Intel Corporation
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

#pragma once

#include <hardware/camera3.h>

#include <mutex>
#include <vector>

#include "Camera3Stream.h"
#include "Camera3StreamHAL.h"
#include "Camera3StreamListener.h"
#include "HALv3Header.h"
#include "HALv3Interface.h"
#include "PlatformData.h"
#include "PrivateStream.h"
#include "PrivacyControl.h"
#include "ResultProcessor.h"

namespace camera3 {

class ParameterResult;

/**
 * \class RequestManager
 *
 * This class is used to handle requests. It has the following
 * roles:
 * - It instantiates ResultProcessor.
 */
class RequestManager : public RequestManagerCallback, public icamera::camera_callback_ops_t {
 public:
    RequestManager(int cameraId);
    virtual ~RequestManager();

    int init(const camera3_callback_ops_t* callback_ops);

    int deinit();

    int configureStreams(camera3_stream_configuration_t* stream_list);

    int constructDefaultRequestSettings(int type, const camera_metadata_t** meta);

    int processCaptureRequest(camera3_capture_request_t* request);

    void dump(int fd);

    int flush();

    void returnRequestDone(uint32_t frameNumber);

 private:
    int analysisStreamList(camera3_stream_configuration_t* stream_list,
                           camera3_stream_t** inputStream);
    int createHALStreams(camera3_stream_configuration_t* stream_list, int* halStreamFlag,
                         int* halStreamNum, std::map<int, int>& streamToHALIndex,
                         camera3_stream_t** userStreamForFd);
    int deviceConfigHALStreams(int halStreamNum);
    void createCamera3Stream(camera3_stream_configuration_t* stream_list,
                             std::map<int, int>& streamToHALIndex, int* halStreamFlag,
                             camera3_stream_t* inputReqStream, camera3_stream_t* userStreamForFd);
    void deleteStreams();
    void increaseRequestCount();
    int waitProcessRequest();
    void waitAllRequestsDone();
    camera3_stream_t* choosePreviewStream(uint32_t streamsNum, camera3_stream_t** streams);
    int checkStreamRotation(camera3_stream_configuration_t* stream_list);

    static void callbackNotify(const icamera::camera_callback_ops* cb,
                               const icamera::camera_msg_data_t& data);
    void handleCallbackEvent(const icamera::camera_msg_data_t& data);
    int handlePrivacySwitch(camera3_capture_request_t* request, bool* privacyMode);

 private:
    static const uint32_t kMaxStreamNum = 8;
    const uint64_t kMaxDuration = 2000000000;  // 2000ms

    static const int kMaxProcessRequestNum = 10;
    struct CameraBufferInfo {
        icamera::camera_buffer_t halBuffer[kMaxStreamNum];
        uint32_t frameNumber;
        bool frameInProcessing;
    };
    struct CameraBufferInfo mCameraBufferInfo[kMaxProcessRequestNum];

    int mCameraId;
    std::unique_ptr<PrivacyControl> mPrivacyControl;
    bool mSwPrivacyMode;
    bool mPrivacyStarted;

    const camera3_callback_ops_t* mCallbackOps;
    bool mCameraDeviceStarted;
    ResultProcessor* mResultProcessor;
    ParameterResult* mParameterResult;

    std::map<int, android::CameraMetadata> mDefaultRequestSettings;
    std::vector<Camera3Stream*> mCamera3StreamVector;
    std::vector<Camera3StreamHAL*> mCamera3HALStreams;  // Save HAL streams only
    bool mInputStreamConfigured;

    std::condition_variable mRequestCondition;
    // mRequestLock is used to protect mRequestInProgress
    std::mutex mRequestLock;
    uint32_t mRequestInProgress;
    std::condition_variable mWaitAllRequestsDone;

    android::CameraMetadata mLastSettings;
    icamera::stream_t mHALStream[kMaxStreamNum];
    PrivateStream* mFDStream;
    camera3_stream_t mFDStreamsInfo;
    int mFDStreamIndex;
    icamera::stream_config_t mStreamCfg;

    /* choose HAL stream to do qbuf/dqbuf from stream list.
     * halStreamFlag: array keeps the result.
     * halStreamFlag[i] = n means the halStreams[i] is the Listener of
     * halStreams[n], if i==n, then the stream is HAL stream.
     * return value is the total Number of HAL streams
     */
    int chooseHALStreams(const uint32_t requestStreamNum, int* halStreamFlag,
                         icamera::stream_t* halStreams);
};

}  // namespace camera3
