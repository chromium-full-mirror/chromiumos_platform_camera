/*
 * Copyright (C) 2023 Intel Corporation
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
#include <sys/time.h>

#include <mutex>
#include <queue>
#include <vector>
#include <memory>

#include "HALv3Header.h"
#include "HALv3Interface.h"
#include "PlatformData.h"
#include "ResultProcessor.h"
#include "Camera3Buffer.h"
#include "PostProcessorBase.h"
#include "Thread.h"

namespace camera3 {

/**
 * \class PrivacyControl
 *
 * This class is used to handle the request in privacy mode
 */
class PrivacyControl : public icamera::Thread {
 public:
    PrivacyControl(int cameraId, const camera3_callback_ops_t* callback,
                   RequestManagerCallback* requestManagerCallback);
    virtual ~PrivacyControl() override;
    int start();
    void stop();

    int configure(const icamera::stream_t& halStream, const camera3_stream_t& stream);
    int processCaptureRequest(camera3_capture_request_t* request);

    static void setPrivacyMode(bool enable);
    static bool getPrivacyMode();

 private:
    static const uint32_t kMaxStreamNum = 8;
    struct CaptureRequest {
        uint32_t frameNumber;
        std::vector<std::shared_ptr<Camera3Buffer>> outputCam3Bufs;
        camera3_stream_buffer_t outputBuffers[kMaxStreamNum];
        camera3_stream_buffer_t* inputBuffer;
        std::shared_ptr<MetadataMemory> metadata;
    };
    virtual bool threadLoop() override;

    void updateMetadataResult(android::CameraMetadata* settings);

 private:
    const int mCameraId;
    const uint64_t kMaxDuration = 2000000000;  // 2000ms
    uint64_t mLastTimestamp;
    MetadataMemory* mLastSettings;
    bool mThreadRunning;

    static std::mutex gPrivacyLock;
    static bool gPrivacyMode;

    std::mutex mLock;
    std::condition_variable mRequestCondition;
    const camera3_callback_ops_t* mCallbackOps;
    std::queue<std::shared_ptr<CaptureRequest>> mCaptureRequest;
    RequestManagerCallback* mRequestManagerCallback;

    std::unique_ptr<icamera::JpegProcess> mJpegProcessor;
    // Use to hold one black frame for user BLOB stream
    std::shared_ptr<Camera3Buffer> mInternalBuf;
};
}  // namespace camera3
