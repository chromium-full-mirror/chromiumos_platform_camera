/*
 * Copyright (C) 2021-2023 Intel Corporation
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

PrivateStream::PrivateStream(int cameraId, const icamera::stream_t& halStream,
                             const camera3_stream_t& stream, int maxBuf)
        : mCameraId(cameraId),
          mStreamState(false),
          mHALStream(halStream),
          mInitialized(false),
          mCurrentBufIdx(-1) {
    LOG1("[%p]@%s, stream:%dx%d, format:%d, type:%d", this, __func__, mHALStream.width,
         mHALStream.height, mHALStream.format, stream.stream_type);

    std::string threadName = "PrivateStream-";
    threadName += std::to_string(mHALStream.id);

    // Run PrivateStream thread
    int ret = icamera::Thread::run(threadName);
    CheckAndLogError(ret != icamera::OK, VOID_VALUE, "[%p]@%s Failed to run thread.", this,
                     __func__);

    mBuffers.resize(maxBuf);
    mBufferPool = std::unique_ptr<Camera3BufferPool>(new Camera3BufferPool());
    // Create the buffer pool with DMA handle buffer
    ret = mBufferPool->createBufferPool(mCameraId, mBuffers.size(), stream.width,
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

    LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);

    std::unique_lock<std::mutex> lock(mLock);
    if (mQueuedBuffer.size() == mBuffers.size()) return nullptr;

    std::shared_ptr<Camera3Buffer> buf = mBufferPool->acquireBuffer();
    CheckAndLogError(buf == nullptr, nullptr, "@%s no available internal buffer", __func__);
    // using buffer pool, store the buffer, then can return it when frame done
    mQueuedBuffer[frameNumber] = buf;

    mCurrentBufIdx = (mCurrentBufIdx  + 1) % mBuffers.size();
    mBuffers[mCurrentBufIdx] = buf->getHalBuffer();
    // Fill the specific setting
    mBuffers[mCurrentBufIdx].s.usage = mHALStream.usage;
    mBuffers[mCurrentBufIdx].s.id = mHALStream.id;
    mBuffers[mCurrentBufIdx].frameNumber = frameNumber;

    LOG2("<id%d:req%u>@%s, mHALStream.id:%d, fetch buf addr:%p will run fd", mCameraId, frameNumber,
         __func__, mHALStream.id, buf->data());

    return &mBuffers[mCurrentBufIdx];
}

void PrivateStream::queueBufferDone(uint32_t frameNumber) {
    CheckAndLogError(mInitialized == false, VOID_VALUE, "mInitialized is false");
    LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);
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
        LOG2("<fn%u>[%p] stream exiting", frameNumber, this);
        return true;
    }
    CheckAndLogError(ret != icamera::OK || !buffer, true, "[%p]failed to dequeue buffer, ret %d",
                     this, ret);
    LOG2("<fn%u>[%p]@%s, buffer->timestamp:%lu addr %p, flags %x", frameNumber, this, __func__,
         buffer->timestamp, buffer->addr, buffer->flags);
    std::shared_ptr<Camera3Buffer> outCam3Buf = nullptr;
    {
        std::unique_lock<std::mutex> lock(mLock);
        if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end() &&
            mQueuedBuffer[frameNumber]->data() == buffer->addr) {
            outCam3Buf = mQueuedBuffer[frameNumber];
        }
        CheckAndLogError(!outCam3Buf, true, "<fn%u>can't identify the buffer source", frameNumber);
    }

    // Only flush buffer can be used to do post process
    if (!(buffer->flags & icamera::BUFFER_FLAG_NO_FLUSH)) {
        processFrame(frameNumber, outCam3Buf);
    }

    outCam3Buf->dumpImage(frameNumber, icamera::DUMP_AAL_OUTPUT, V4L2_PIX_FMT_NV12);

    {
        std::unique_lock<std::mutex> lock(mLock);
        // release buffer
        if (mQueuedBuffer.find(frameNumber) != mQueuedBuffer.end()) {
            LOG2("<id%d:req%u>@%s mHALStream.id:%d, return buf addr:%p", mCameraId, frameNumber,
                 __func__, mHALStream.id, mQueuedBuffer[frameNumber]->data());
            if (!isLocalBuf(outCam3Buf)) {
                // if HAL stream using buffer from pool to qbuf/dqbuf, return it
                mBufferPool->returnBuffer(mQueuedBuffer[frameNumber]);
            }
            mQueuedBuffer.erase(frameNumber);
        }
    }
    return true;
}

PrivateStreamFD::PrivateStreamFD(int cameraId, const icamera::stream_t& halStream,
                                 const camera3_stream_t& stream)
        : PrivateStream(cameraId, halStream, stream,
                        icamera::PlatformData::faceEngineRunningInterval(cameraId)),
          mFaceDetection(nullptr),
          mFaceBuffer(nullptr) {
    LOG2("[%p]@%s, stream:%dx%d, format:%d, type:%d", this, __func__, mHALStream.width,
         mHALStream.height, mHALStream.format, stream.stream_type);
    unsigned int maxFacesNum = icamera::PlatformData::getMaxFaceDetectionNumber(mCameraId);
    mFaceDetection = icamera::FaceDetection::createInstance(mCameraId, maxFacesNum, mHALStream.id,
                                                            mHALStream.width, mHALStream.height,
                                                            stream.format, stream.usage);

    mFaceBuffer = MemoryUtils::allocateHandleBuffer(stream.width, stream.height, stream.format,
                                                    stream.usage, cameraId);
    if (mFaceBuffer == nullptr) {
        LOGE("Failed to alloc face buffer");
        mInitialized = false;
    } else if (mFaceBuffer->lock() != icamera::OK) {
        mFaceBuffer = nullptr;
        LOGE("Failed to init face buffer");
        mInitialized = false;
    }
}

PrivateStreamFD::~PrivateStreamFD() {
    LOG2("[%p]@%s", this, __func__);

    PrivateStream::requestExit();

    PrivateStream::requestExitAndWait();

    if (mFaceDetection) {
        icamera::FaceDetection::destoryInstance(mCameraId);
        mFaceDetection = nullptr;
    }
}

void PrivateStreamFD::processFrame(uint32_t frameNumber,
                                   const std::shared_ptr<Camera3Buffer>& ccBuf) {
    LOG2("<fn%u>@%s", frameNumber, __func__);

    if (mFaceDetection) {
        mFaceDetection->runFaceDetection(ccBuf, true);
    }
}

bool PrivateStreamFD::isLocalBuf(const std::shared_ptr<Camera3Buffer>& ccBuf) {
    return ccBuf == mFaceBuffer;
}

// fetch the buffer will be queued to Hal
icamera::camera_buffer_t* PrivateStreamFD::fetchRequestBuffers(uint32_t frameNumber) {
    CheckAndLogError(mInitialized == false, nullptr, "mInitialized is false");

    LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);

    {
    std::unique_lock<std::mutex> lock(mLock);
    if (mQueuedBuffer.size() == mBuffers.size()) return nullptr;

    // fetch local buffer once for mBuffers.size() times
    if ((mCurrentBufIdx  + 1) % mBuffers.size() == 0) {
        mCurrentBufIdx = (mCurrentBufIdx  + 1) % mBuffers.size();

        mQueuedBuffer[frameNumber] = mFaceBuffer;

        mBuffers[mCurrentBufIdx] = mFaceBuffer->getHalBuffer();
        // Fill the specific setting
        mBuffers[mCurrentBufIdx].s.usage = mHALStream.usage;
        mBuffers[mCurrentBufIdx].s.id = mHALStream.id;
        mBuffers[mCurrentBufIdx].frameNumber = frameNumber;

        LOG2("<id%d:req%u>@%s, mHALStream.id:%d, fetch buf addr:%p", mCameraId, frameNumber,
             __func__, mHALStream.id, mFaceBuffer->data());

        return &mBuffers[mCurrentBufIdx];
    }
    }

    auto buf = PrivateStream::fetchRequestBuffers(frameNumber);
    CheckAndLogError(buf == nullptr, nullptr, "@%s no available internal buffer", __func__);

    // set flag to disable flush
    mBuffers[mCurrentBufIdx].flags |= icamera::BUFFER_FLAG_NO_FLUSH;

    return &mBuffers[mCurrentBufIdx];
}

}  // namespace camera3
