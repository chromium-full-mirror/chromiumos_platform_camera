/*
 * Copyright (C) 2021 Intel Corporation
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

#define LOG_TAG Camera3StreamListener

#include "aal/Camera3StreamListener.h"

#include <utility>
#include <vector>

#include "CameraLog.h"
#include "Utils.h"

namespace camera3 {

Camera3StreamListener::Camera3StreamListener(int cameraId, CallbackEventInterface* callback,
                                             ParameterResult* result, uint32_t maxNumReqInProc,
                                             const icamera::stream_t& halStream,
                                             const camera3_stream_t& stream,
                                             const camera3_stream_t* inputStream)
        : Camera3Stream(cameraId, callback, result, maxNumReqInProc, halStream, stream,
                        inputStream) {
    LOG1("[%p]@%s, listener stream", this, __func__);
}

Camera3StreamListener::~Camera3StreamListener() {}

bool Camera3StreamListener::threadLoop() {
    LOG2("[%p]@%s", this, __func__);

    if (mStreamStatus == DRAIN_REQUESTS) {
        drainRequest();
        return true;
    }

    if (!waitCaptureResultReady()) {
        return true;
    }

    auto captureResult = mCaptureResultMap.begin();
    std::shared_ptr<CaptureResult> result = captureResult->second;
    uint32_t frameNumber = captureResult->first;

    std::shared_ptr<Camera3Buffer> inputCam3Buf = result->inputCam3Buf;
    std::shared_ptr<StreamComInfo> halOutput = nullptr;

    if (!inputCam3Buf) {
        // listener stream get the buffer from HAL stream
        std::unique_lock<std::mutex> lock(mLock);
        CheckAndLogError(mHALStreamOutput.find(frameNumber) == mHALStreamOutput.end(), true,
                         "<fn%u>[%p] can't find HAL stream output", frameNumber, this);

        halOutput = mHALStreamOutput[frameNumber];
        mHALStreamOutput.erase(frameNumber);
    }

    {
        std::unique_lock<std::mutex> lock(mLock);
        mCaptureResultMap.erase(frameNumber);
    }

    if (!handleNewFrame(halOutput, result)) return false;

    return true;
}

void Camera3StreamListener::notifyCaptureBufferReady(
    uint32_t frameNumber, const std::shared_ptr<StreamComInfo>& halOutput) {
    LOG2("[%p] @%s", this, __func__);
    std::lock_guard<std::mutex> l(mLock);
    if (mCaptureResultMap.find(frameNumber) != mCaptureResultMap.end()) {
        mHALStreamOutput[frameNumber] = halOutput;
        mBufferDoneCondition.notify_one();
    }
}

bool Camera3StreamListener::isLockUserBufferNeeded(
    const std::shared_ptr<Camera3Buffer>& inputCam3Buf) {
    return true;
}

bool Camera3StreamListener::waitCaptureResultReady() {
    std::unique_lock<std::mutex> lock(mLock);
    /* 1st loop, listener stream wait on the CaptureResult
     * BufferDoneCondition if the CaptureResultVector not empty.
     * 2nd loop, the CaptureResult is not empty, the listener stream should wait
     * HAL Stream send out buffer ready event if it doesn't have inputCam3Buf.
     * 3rd loop, both vecotr are not empty, return true.
     */
    // HAL stream and listener stream should wait RequestManager notification
    bool needWaitBufferReady = mCaptureResultMap.empty();
    // listeners stream should wait HAL output buffer if not has input buffer
    if (!mCaptureResultMap.empty()) {
        auto captureResult = mCaptureResultMap.begin();
        std::shared_ptr<CaptureResult> result = captureResult->second;
        needWaitBufferReady = !result->inputCam3Buf && mHALStreamOutput.empty();
    }

    if (mStreamStatus == PEND_PROCESS) needWaitBufferReady = true;
    if (mStreamStatus == DRAIN_REQUESTS && !mCaptureResultMap.empty()) return false;

    if (needWaitBufferReady) {
        std::cv_status ret = mBufferDoneCondition.wait_for(
            lock, std::chrono::nanoseconds(kMaxDuration * SLOWLY_MULTIPLIER));
        if (!mCaptureResultMap.empty() && ret == std::cv_status::timeout) {
            LOGW("[%p]%s, wait buffer ready time out", this, __func__);
        }
        // return false to make the threadLoop run again
        return false;
    }

    return true;
}

}  // namespace camera3
