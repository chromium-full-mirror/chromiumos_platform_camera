/*
 * Copyright (C) 2021-2022 Intel Corporation
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

#define LOG_TAG Camera3StreamHAL

#include "aal/Camera3StreamHAL.h"

#include <utility>
#include <vector>

#include "CameraLog.h"
#include "PlatformData.h"
#include "HALv3Utils.h"
#include "Utils.h"

namespace camera3 {

Camera3StreamHAL::Camera3StreamHAL(int cameraId, CallbackEventInterface* callback,
                                   ParameterResult* result, uint32_t maxNumReqInProc,
                                   const icamera::stream_t& halStream,
                                   const camera3_stream_t& stream,
                                   const camera3_stream_t* inputStream)
        : Camera3Stream(cameraId, callback, result, maxNumReqInProc, halStream, stream,
                        inputStream),
          mBufferPool(nullptr) {
    LOG1("[%p]@%s, HAL stream", this, __func__);
    mQueuedBuffer.clear();
    mBufferPool = std::unique_ptr<Camera3BufferPool>(new Camera3BufferPool());
}

Camera3StreamHAL::~Camera3StreamHAL() {}

void Camera3StreamHAL::sendEvent(const icamera::camera_msg_data_t& data) {
    LOG2("@%s receive sof event: %ld", __func__, data.data.buffer_ready.timestamp);

    std::lock_guard<std::mutex> sofLock(mSofLock);
    mSofCondition.notify_one();
}

int Camera3StreamHAL::setActive(bool state) {
    if (mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW) {
        if (!mStreamState && state) {
            mBufferPool->createBufferPool(
                mCameraId, mMaxNumReqInProc, mHALStream.width, mHALStream.height,
                HalV3Utils::V4l2FormatToHALFormat(mHALStream.format), mStream.usage);
            LOG2("@%s, HAL stream create BufferPool", __func__);
        } else if (mStreamState && !state) {
            if (mBufferPool) {
                mBufferPool->destroyBufferPool();
            }
        }
    }

    Camera3Stream::setActive(state);
    return icamera::OK;
}

bool Camera3StreamHAL::isLockUserBufferNeeded(const std::shared_ptr<Camera3Buffer>& inputCam3Buf) {
    return (!mListeners.empty() || Camera3Stream::isLockUserBufferNeeded(inputCam3Buf));
}

bool Camera3StreamHAL::threadLoop() {
    LOG2("[%p] @%s", this, __func__);

    if (mStreamStatus == DRAIN_REQUESTS) {
        drainRequest();
        return true;
    }

    if (!waitCaptureResultReady()) {
        return true;
    }

    std::shared_ptr<CaptureResult> result = nullptr;
    uint32_t frameNumber = 0;

    {
        std::unique_lock<std::mutex> lock(mLock);
        if (!mCaptureResultMap.empty()) {
            auto captureResult = mCaptureResultMap.begin();
            result = captureResult->second;
            frameNumber = captureResult->first;
        }
    }
    CheckAndLogError(!result, true, "no valid capture request");

    std::shared_ptr<Camera3Buffer> inputCam3Buf = result->inputCam3Buf;
    std::shared_ptr<StreamComInfo> halOutput = nullptr;

    if (!inputCam3Buf) {
        CheckAndLogError(!result->hCapture, true, "no HAL capture for request %u", frameNumber);
        icamera::camera_buffer_t* buffer = nullptr;
        LOG2("[%p]dqbuf for frameNumber %u", this, frameNumber);
        int ret = icamera::camera_stream_dqbuf(mCameraId, mHALStream.id, &buffer, nullptr);
        if (ret == icamera::NO_INIT) {
            LOG2("[%p] stream exiting for frameNumber %u", this, frameNumber);
            mStreamStatus = PEND_PROCESS;
            return true;
        }
        CheckAndLogError(ret != icamera::OK || !buffer, true,
                         "[%p]failed to dequeue buffer for frameNumber %u, ret %d", this,
                         frameNumber, ret);
        LOG2("[%p]@%s, buffer->timestamp:%lu addr %p", this, __func__, buffer->timestamp,
             buffer->addr);

        {
            std::unique_lock<std::mutex> lock(mLock);
            if (buffer->frameNumber != frameNumber &&
                mCaptureResultMap.find(buffer->frameNumber) != mCaptureResultMap.end()) {
                result = mCaptureResultMap[buffer->frameNumber];
                frameNumber = buffer->frameNumber;
            }
            CheckAndLogError(buffer->addr != result->hCapture->cam3Buf->data(), true,
                             "can't identify the buffer source");
            halOutput = result->hCapture;
        }
        halOutput->sequence = buffer->sequence;
        halOutput->cam3Buf->setTimeStamp(buffer->timestamp);

        // sync before notify listeners
        handleSofAlignment(frameNumber);

        for (auto& iter : mListeners) {
            iter->notifyCaptureBufferReady(frameNumber, halOutput);
        }
    }

    {
        std::unique_lock<std::mutex> lock(mLock);
        mCaptureResultMap.erase(frameNumber);
    }

    // HAL stream is triggered by listener, itself not requested, start next loop
    if (!result->outputCam3Buf) return true;

    if (!handleNewFrame(halOutput, result)) return false;

    return true;
}

void Camera3StreamHAL::addListener(Camera3StreamListener* listener) {
    mListeners.push_back(listener);
}

/* fetch the buffers will be queued to Hal, the buffer has 3 sources:
 * 1st using HAL stream's request, then the buffer->addr should equal
 *     request->cam3Buf->addr()
 * 2nd if using listener buffer directly
 * 3rd using bufferpool, the buffer from pool is stored in mQueuedBuffer
 */
bool Camera3StreamHAL::fetchRequestBuffers(icamera::camera_buffer_t* buffer, uint32_t frameNumber) {
    int requestStreamCount = 0;
    std::shared_ptr<Camera3Buffer> buf = nullptr;

    // check if any one provided a buffer in listeners
    for (auto& iter : mListeners) {
        if (iter->requestHALCapture(frameNumber, &buf)) requestStreamCount++;
    }

    // if HAL stream has a buffer, use HAL stream's buffer to qbuf/dqbuf
    if (requestHALCapture(frameNumber, &buf)) requestStreamCount++;

    if (!requestStreamCount) {
        // no stream requested
        return false;
    }
    LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);

    /* Fix me... if has 2 or more streams, use the same buffer.
     ** if >= 2 streams request in same frame, we use buffer pool temporary.
     ** to do: prefer to use user buffer to avoid memcpy
     */
    if (!buf || requestStreamCount >= 2) {
        LOG2("[%p]@%s get buffer from pool", this, __func__);
        CheckAndLogError(!mBufferPool, false, "no buffer pool for StreamComInfo");
        buf = mBufferPool->acquireBuffer();
        CheckAndLogError(buf == nullptr, false, "<fn%u>no available internal buffer", frameNumber);
        // using buffer pool, store the buffer, then can return it when frame done
        mQueuedBuffer[frameNumber] = buf;
    }

    *buffer = buf->getHalBuffer();
    // Fill the specific setting
    buffer->s.usage = mHALStream.usage;
    buffer->s.id = mHALStream.id;
    buffer->frameNumber = frameNumber;

    std::shared_ptr<StreamComInfo> hCapture = std::make_shared<StreamComInfo>();
    hCapture->cam3Buf = buf;
    hCapture->frameNumber = frameNumber;

    std::lock_guard<std::mutex> l(mLock);
    if (mPendingRequests.find(frameNumber) == mPendingRequests.end()) {
        std::shared_ptr<CaptureResult> result = std::make_shared<CaptureResult>();
        LOG2("<fn%u>[%p]@%s, only listener requested", frameNumber, this, __func__);
        if (result) {
            result->frameNumber = frameNumber;
            result->outputCam3Buf = nullptr;
            result->inputCam3Buf = nullptr;
            mPendingRequests[frameNumber] = result;
        }
    }
    mPendingRequests[frameNumber]->hCapture = hCapture;

    return true;
}

void Camera3StreamHAL::handleSofAlignment(uint32_t frameNumber) {
    if (!icamera::PlatformData::swProcessingAlignWithIsp(mCameraId)) return;

    bool needAlignment = false;
    if (mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW &&
        mPostProcessType != icamera::POST_PROCESS_NONE) {
        needAlignment = true;
    } else {
        std::shared_ptr<Camera3Buffer> cam3Buf = nullptr;
        for (auto& iter : mListeners) {
            if (!iter->requestHALCapture(frameNumber, &cam3Buf)) {
                continue;
            }
            needAlignment = true;
        }
    }
    if (!needAlignment) return;

    std::unique_lock<std::mutex> sofLock(mSofLock);
    std::cv_status ret =
        mSofCondition.wait_for(sofLock, std::chrono::nanoseconds(kMaxDuration * SLOWLY_MULTIPLIER));

    if (ret == std::cv_status::timeout) {
        LOGW("%s, [%p] wait sof timeout, skip alignment this time", __func__, this);
    }
    LOG2("%s, [%p] running post processing align with sof event", __func__, this);
}

bool Camera3StreamHAL::waitCaptureResultReady() {
    std::unique_lock<std::mutex> lock(mLock);
    /* 1st loop, the HAL stream wait on the CaptureResult
     * BufferDoneCondition if the CaptureResultVector not empty.
     * 2nd loop, the CaptureResult is not empty, and the HAL stream will start to dqbuf
     * if it doesn't have inputCam3Buf.
     */
    // HAL stream and listener stream should wait RequestManager notification
    bool needWaitBufferReady = mCaptureResultMap.empty();
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

void Camera3StreamHAL::requestStreamDone(uint32_t frameNumber) {
    /* release buffers. if the buffer used to qbuf/dqbuf is from listener or HAL
     * stream, it will be released in its stream, because now we use buffer pool
     * to sync buffer between frames.
     */
    std::unique_lock<std::mutex> lock(mLock);
    if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end()) {
        LOG2("<fn%u>[%p] @%s", frameNumber, this, __func__);
        // if HAL stream using buffer from pool to qbuf/dqbuf, return it
        mBufferPool->returnBuffer(mQueuedBuffer[frameNumber]);
        mQueuedBuffer.erase(frameNumber);
    }
}

}  // namespace camera3
