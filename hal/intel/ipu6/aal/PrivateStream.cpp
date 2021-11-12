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

#define LOG_TAG PrivateStream

#include "aal/PrivateStream.h"

#include <stdlib.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "CameraDump.h"
#include "Errors.h"
#include "PlatformData.h"

namespace camera3 {
#define MAX_PRIVATE_STREAM_BUF_NUM 8

PrivateStream::PrivateStream(int cameraId, const icamera::stream_t& halStream,
                             const camera3_stream_t& stream)
        : mCameraId(cameraId),
          mStreamState(false),
          mHALStream(halStream),
          mInitialized(false) {
    LOG1("[%p]@%s, stream:%dx%d, format:%d, type:%d", this, __func__, mHALStream.width,
         mHALStream.height, mHALStream.format, stream.stream_type);

    std::string threadName = "PrivateStream-";
    threadName += std::to_string(mHALStream.id);

    // Run PrivateStream thread
    int ret = icamera::Thread::run(threadName);
    CheckAndLogError(ret != icamera::OK, VOID_VALUE, "[%p]@%s Failed to run thread.", this,
                     __func__);

    mBufferPool = std::unique_ptr<Camera3BufferPool>(new Camera3BufferPool());
    // Create the buffer pool with DMA handle buffer
    ret = mBufferPool->createBufferPool(mCameraId, MAX_PRIVATE_STREAM_BUF_NUM, stream.width,
                                        stream.height, stream.format, stream.usage);
    CheckAndLogError(ret != icamera::OK, VOID_VALUE, "[%p]@%s Failed to createBufferPool.", this,
                     __func__);

    mInitialized = true;
}

PrivateStream::~PrivateStream() {
    LOG1("[%p]@%s", this, __func__);

    if (mBufferPool) {
        mBufferPool->destroyBufferPool();
    }
}

// fetch the buffer will be queued to Hal
icamera::camera_buffer_t* PrivateStream::fetchRequestBuffers(uint32_t frameNumber) {
    CheckAndLogError(mInitialized == false, nullptr, "mInitialized is false");

    LOG2("<req%u>[%p]@%s", frameNumber, this, __func__);

    std::shared_ptr<Camera3Buffer> buf = mBufferPool->acquireBuffer();
    CheckAndLogError(buf == nullptr, nullptr, "@%s no available internal buffer", __func__);
    std::unique_lock<std::mutex> lock(mLock);
    // using buffer pool, store the buffer, then can return it when frame done
    mQueuedBuffer[frameNumber] = buf;
    icamera::camera_buffer_t buffer = buf->getHalBuffer();
    // Fill the specific setting
    buffer.s.usage = mHALStream.usage;
    buffer.s.id = mHALStream.id;
    mBuffers[frameNumber] = buffer;

    LOG2("<id%d:req%u>@%s, mHALStream.id:%d, fetch buf addr:%p will run fd", mCameraId, frameNumber,
         __func__, mHALStream.id, buf->data());

    return &mBuffers[frameNumber];
}

void PrivateStream::returnBuffer(uint32_t frameNumber) {
    CheckAndLogError(mInitialized == false, VOID_VALUE, "mInitialized is false");
    LOG2("<req%u>[%p]@%s", frameNumber, this, __func__);

    // release buffer
    if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end()) {
        LOG2("<id%d:req%u>@%s mHALStream.id:%d, return buf addr:%p", mCameraId, frameNumber,
             __func__, mHALStream.id, mQueuedBuffer[frameNumber]->data());
        // if HAL stream using buffer from pool to qbuf/dqbuf, return it
        mBufferPool->returnBuffer(mQueuedBuffer[frameNumber]);
        mQueuedBuffer.erase(frameNumber);
    }

    if (mBuffers.find(frameNumber) != mBuffers.end()) {
        mBuffers.erase(frameNumber);
    }
}

void PrivateStream::queueBufferDone(uint32_t frameNumber) {
    CheckAndLogError(mInitialized == false, VOID_VALUE, "mInitialized is false");
    LOG2("<req%u>[%p]@%s", frameNumber, this, __func__);
    std::lock_guard<std::mutex> l(mLock);
    mPrivateResult.push(frameNumber);
    mBufferDoneCondition.notify_one();
}

void PrivateStream::requestExit() {
    LOG2("[%p]@%s", this, __func__);

    icamera::Thread::requestExit();
    std::lock_guard<std::mutex> l(mLock);

    mBufferDoneCondition.notify_one();
}

bool PrivateStream::threadLoop() {
    LOG2("[%p]@%s", this, __func__);

    uint32_t frameNumber;
    {
        std::unique_lock<std::mutex> lock(mLock);
        if (mPrivateResult.empty()) {
            std::cv_status ret = mBufferDoneCondition.wait_for(
                lock, std::chrono::nanoseconds(kMaxDuration * SLOWLY_MULTIPLIER));
            if (ret == std::cv_status::timeout) {
                LOGW("[%p]%s, wait buffer ready time out", this, __func__);
            }
            return true;
        }

        frameNumber = mPrivateResult.front();
        mPrivateResult.pop();
    }

    // dequeue buffer from HAL
    icamera::camera_buffer_t* buffer = nullptr;
    LOG2("[%p]@%s, dqbuf for frameNumber %d, mHALStream.id:%d", this, __func__, frameNumber,
         mHALStream.id);
    int ret = icamera::camera_stream_dqbuf(mCameraId, mHALStream.id, &buffer, nullptr);
    if (ret == icamera::NO_INIT) {
        std::unique_lock<std::mutex> lock(mLock);
        if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end()) {
            returnBuffer(frameNumber);
        }
        LOG2("<req%u>[%p] stream exiting", frameNumber, this);
        return true;
    }
    CheckAndLogError(ret != icamera::OK || !buffer, true, "[%p]failed to dequeue buffer, ret %d",
                     this, ret);
    LOG2("<req%u>[%p]@%s, buffer->timestamp:%lu addr %p", frameNumber, this, __func__,
         buffer->timestamp, buffer->addr);
    std::shared_ptr<Camera3Buffer> outCam3Buf = nullptr;
    {
        std::unique_lock<std::mutex> lock(mLock);
        if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end()) {
            if (mQueuedBuffer[frameNumber]->data() == buffer->addr) {
                outCam3Buf = mQueuedBuffer[frameNumber];
            } else {
                returnBuffer(frameNumber);
            }
        }
        CheckAndLogError(!outCam3Buf, true, "<req%u>can't identify the buffer source", frameNumber);
    }

    processFrame(frameNumber, outCam3Buf);
    outCam3Buf->dumpImage(frameNumber, icamera::DUMP_AAL_OUTPUT, V4L2_PIX_FMT_NV12);

    {
        std::unique_lock<std::mutex> lock(mLock);
        returnBuffer(frameNumber);
    }
    return true;
}

PrivateStreamFD::PrivateStreamFD(int cameraId, const icamera::stream_t& halStream,
                                 const camera3_stream_t& stream)
        : PrivateStream(cameraId, halStream, stream),
          mFaceDetection(nullptr) {
    LOG2("[%p]@%s, stream:%dx%d, format:%d, type:%d", this, __func__, mHALStream.width,
         mHALStream.height, mHALStream.format, stream.stream_type);
    unsigned int maxFacesNum = icamera::PlatformData::getMaxFaceDetectionNumber(mCameraId);
    mFaceDetection = icamera::FaceDetection::createInstance(mCameraId, maxFacesNum, mHALStream.id,
                                                            mHALStream.width, mHALStream.height,
                                                            stream.format, stream.usage);
}

PrivateStreamFD::~PrivateStreamFD() {
    LOG2("[%p]@%s", this, __func__);

    PrivateStream::requestExit();

    if (mFaceDetection) {
        icamera::FaceDetection::destoryInstance(mCameraId);
        mFaceDetection = nullptr;
    }
}

void PrivateStreamFD::processFrame(uint32_t frameNumber,
                                   const std::shared_ptr<Camera3Buffer>& ccBuf) {
    LOG2("<req%u>@%s", frameNumber, __func__);

    if (mFaceDetection) {
        mFaceDetection->runFaceDetectionBySync(ccBuf);
    }
}
}  // namespace camera3
