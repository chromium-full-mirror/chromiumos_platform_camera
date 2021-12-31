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

#include <hardware/camera3.h>

#include <unordered_map>
#include <memory>

#include "Camera3Stream.h"

namespace camera3 {

class Camera3StreamListener : public Camera3Stream {
 public:
    Camera3StreamListener(int cameraId, CallbackEventInterface* callback, ParameterResult* result,
                          uint32_t maxNumReqInProc, const icamera::stream_t& halStream,
                          const camera3_stream_t& stream,
                          const camera3_stream_t* inputStream = nullptr);
    virtual ~Camera3StreamListener();

    virtual bool threadLoop();

    void notifyCaptureBufferReady(uint32_t frameNumber,
                                  const std::shared_ptr<StreamComInfo>& halOutput);

 private:
    /**
     * Wait capture buffer result ready, called in ThreadLoop, return false if need to wait,
     * return true to continue the threadloop.
     */
    bool waitCaptureResultReady();
    virtual bool isLockUserBufferNeeded(const std::shared_ptr<Camera3Buffer>& inputCam3Buf);

 private:
    // HAL streams output result, listener stream will wait on it before process
    std::unordered_map<uint32_t, std::shared_ptr<StreamComInfo>> mHALStreamOutput;
};

}  // namespace camera3
