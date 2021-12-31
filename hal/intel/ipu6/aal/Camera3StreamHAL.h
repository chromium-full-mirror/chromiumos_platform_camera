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

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "Camera3BufferPool.h"
#include "Camera3Stream.h"
#include "Camera3StreamListener.h"

namespace camera3 {

class Camera3StreamHAL : public Camera3Stream {
 public:
    Camera3StreamHAL(int cameraId, CallbackEventInterface* callback, ParameterResult* result,
                     uint32_t maxNumReqInProc, const icamera::stream_t& halStream,
                     const camera3_stream_t& stream, const camera3_stream_t* inputStream = nullptr);
    virtual ~Camera3StreamHAL();

    virtual bool threadLoop();

    int setActive(bool state);
    void sendEvent(const icamera::camera_msg_data_t& data);
    void addListener(Camera3StreamListener* listener);
    // fetch the buffers will be queued to Hal, HAL stream only
    bool fetchRequestBuffers(icamera::camera_buffer_t* buffer, uint32_t frameNumber);
    // called by RequestManager indicates the frame is done, release buffers
    void requestStreamDone(uint32_t frameNumber);

 private:
    void handleSofAlignment(uint32_t frameNumber);

    /**
     * Wait capture buffer result ready, called in ThreadLoop, return false if need to wait,
     * return true to continue the threadloop.
     */
    bool waitCaptureResultReady();
    virtual bool isLockUserBufferNeeded(const std::shared_ptr<Camera3Buffer>& inputCam3Buf);

 private:
    std::vector<Camera3StreamListener*> mListeners;

    std::condition_variable mSofCondition;
    std::mutex mSofLock;

    std::unique_ptr<Camera3BufferPool> mBufferPool;
    // save buffer obj when HAL choose buffer from pool to do qbuf/dqbuf
    std::unordered_map<uint32_t, std::shared_ptr<Camera3Buffer>> mQueuedBuffer;
};

}  // namespace camera3
