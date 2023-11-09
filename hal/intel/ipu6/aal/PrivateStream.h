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

#pragma once

#include <hardware/camera3.h>

#include <map>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "Camera3BufferPool.h"
#include "FaceDetection.h"
#include "Thread.h"

namespace camera3 {

/**
 * \class PrivateStream
 *
 * This class is used to handle requests. It has the following
 */
class PrivateStream : public icamera::Thread {
 public:
    PrivateStream(int cameraId, const icamera::stream_t& halStream, const camera3_stream_t& stream,
                  int maxBuf);
    virtual ~PrivateStream();

    virtual bool threadLoop();
    void queueBufferDone(uint32_t frameNumber);
    virtual icamera::camera_buffer_t* fetchRequestBuffers(uint32_t frameNumber);

 protected:
    virtual void requestExit();
    virtual void processFrame(uint32_t frameNumber,
                              const std::shared_ptr<Camera3Buffer>& ccBuf) = 0;
    virtual bool isLocalBuf(const std::shared_ptr<Camera3Buffer>& ccBuf) = 0;

 protected:
    int mCameraId;
    std::condition_variable mBufferDoneCondition;
    std::mutex mLock;

    bool mStreamState;
    icamera::stream_t mHALStream;
    std::unique_ptr<Camera3BufferPool> mBufferPool;

    std::queue<uint32_t> mPrivateResult;
    std::vector<icamera::camera_buffer_t> mBuffers;

    // save buffer obj when HAL choose buffer from pool to do qbuf/dqbuf
    std::unordered_map<uint32_t, std::shared_ptr<Camera3Buffer>> mQueuedBuffer;
    const uint64_t kMaxDuration = 2000000000;  // 2000ms

    bool mInitialized;
    int mCurrentBufIdx;
};

class PrivateStreamFD : public PrivateStream {
 public:
    PrivateStreamFD(int cameraId, const icamera::stream_t& halStream,
                    const camera3_stream_t& stream);
    virtual ~PrivateStreamFD();
    virtual icamera::camera_buffer_t* fetchRequestBuffers(uint32_t frameNumber);

 protected:
    virtual void processFrame(uint32_t frameNumber, const std::shared_ptr<Camera3Buffer>& ccBuf);
    virtual bool isLocalBuf(const std::shared_ptr<Camera3Buffer>& ccBuf);

 private:
    icamera::FaceDetection* mFaceDetection;
    std::shared_ptr<Camera3Buffer> mFaceBuffer;
};

}  // namespace camera3
