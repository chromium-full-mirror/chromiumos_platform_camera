/*
 * Copyright (C) 2017-2023 Intel Corporation
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

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "FaceDetection.h"
#include "PostProcessor.h"
#include "ResultProcessor.h"
#include "Thread.h"

namespace camera3 {

struct StreamComInfo {
    std::shared_ptr<Camera3Buffer> cam3Buf;
    uint32_t frameNumber;
    int64_t sequence;
};

struct CaptureResult {
    uint32_t frameNumber;
    std::shared_ptr<Camera3Buffer> outputCam3Buf;
    camera3_stream_buffer_t outputBuffer;
    buffer_handle_t handle;
    std::shared_ptr<Camera3Buffer> inputCam3Buf;
    icamera::Parameters param;

    std::shared_ptr<StreamComInfo> hCapture;  // Related device capture info
};

class ParameterResult;

/**
 * \class Camera3Stream
 *
 * This class is used to handle requests. It has the following
 * roles:
 * - It instantiates PostProcessor.
 */
class Camera3Stream : public icamera::Thread {
 public:
    Camera3Stream(int cameraId, CallbackEventInterface* callback, ParameterResult* result,
                  uint32_t maxNumReqInProc, const icamera::stream_t& halStream,
                  const camera3_stream_t& stream, const camera3_stream_t* inputStream = nullptr);
    virtual ~Camera3Stream();

    virtual bool threadLoop() { return false; }
    virtual void requestExit();

    int processRequest(uint32_t frameNumber, const std::shared_ptr<Camera3Buffer>& inputCam3Buf,
                       const camera3_stream_buffer_t& outputBuffer,
                       const icamera::Parameters& param);

    void queueBufferDone(uint32_t frameNumber);
    virtual int setActive(bool state);
    bool isActive() { return mStreamState; }
    void activateFaceDetection(unsigned int maxFaceNum);
    int getPostProcessType() { return mPostProcessType; }

    // drain all pending requests
    void drainAllPendingRequests();

    /* Get the device capture info of this stream for requests
     * Return false if this request doesn't request device capture for this stream.
     * Otherwise, valid cam3Buf will return alternative buffer that can be enqueued to device.
     */
    bool requestHALCapture(uint32_t frameNumber, std::shared_ptr<Camera3Buffer>* cam3Buf);

 protected:
    bool handleNewFrame(std::shared_ptr<StreamComInfo> hCapture,
                        std::shared_ptr<CaptureResult> result);
    int handleCaptureRequest(std::shared_ptr<Camera3Buffer> inBuf,
                             std::shared_ptr<Camera3Buffer> outBuf, icamera::Parameters* parameter);
    int handleReprocessRequest(std::shared_ptr<Camera3Buffer> inBuf,
                               std::shared_ptr<Camera3Buffer> outBuf,
                               icamera::Parameters* parameter);

    void drainRequest();

    virtual bool isLockUserBufferNeeded(const std::shared_ptr<Camera3Buffer>& inputCam3Buf);

 protected:
    const uint64_t kMaxDuration = 2000000000;  // 2000ms

    int mCameraId;
    std::condition_variable mBufferDoneCondition;
    std::mutex mLock;

    CallbackEventInterface* mEventCallback;
    ParameterResult* mParameterResult;

    int mPostProcessType;
    std::unique_ptr<PostProcessor> mPostProcessor;

    bool mStreamState;
    icamera::stream_t mHALStream;
    uint32_t mMaxNumReqInProc;

    camera3_stream_t mStream;

    typedef enum {
        PROCESS_REQUESTS,  // Normal working status.
        PEND_PROCESS,      // Pending process when stopping camera.
        DRAIN_REQUESTS     // Draining all requests after stopped camera.
    } StreamStatus;
    StreamStatus mStreamStatus;

    /* key is frame number, value is CaptureResult */
    // Save received requests temporarily util the related buffers are queued to device for capture
    std::map<uint32_t, std::shared_ptr<CaptureResult>> mPendingRequests;
    // Save requests that wait for device capture
    std::map<uint32_t, std::shared_ptr<CaptureResult>> mCaptureResultMap;

    icamera::FaceDetection* mFaceDetection;
    int mInputPostProcessType;
    std::unique_ptr<PostProcessor> mInputPostProcessor;
    std::unique_ptr<camera3_stream_t> mInputStream;
};

}  // namespace camera3
