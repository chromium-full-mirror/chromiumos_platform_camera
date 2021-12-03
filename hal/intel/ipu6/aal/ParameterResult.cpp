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

#define LOG_TAG ParameterResult

#include "aal/ParameterResult.h"

#include "ICamera.h"
#include "iutils/CameraLog.h"

namespace camera3 {

ParameterResult::ParameterResult(int cameraId) : mCameraId(cameraId) {}

ParameterResult::~ParameterResult() {
    std::lock_guard<std::mutex> l(mLock);
    for (auto it : mParameterResultMap) {
        if (it.second != nullptr) delete (it.second);
    }
    mParameterResultMap.clear();
}

icamera::Parameters* ParameterResult::getParameter(uint32_t frameNumber, int64_t sequence) {
    LOG2("<fn%u:seq%ld>@%s", frameNumber, sequence, __func__);
    {
        std::lock_guard<std::mutex> l(mLock);
        if (mParameterResultMap.find(frameNumber) != mParameterResultMap.end()) {
            return mParameterResultMap[frameNumber];
        }
    }
    icamera::Parameters* parameter = new icamera::Parameters;
    icamera::camera_get_parameters(mCameraId, *parameter, sequence);
    {
        std::lock_guard<std::mutex> l(mLock);
        mParameterResultMap[frameNumber] = parameter;
    }

    return parameter;
}

void ParameterResult::releaseParameter(uint32_t frameNumber) {
    LOG2("<fn%u>@%s", frameNumber, __func__);
    std::lock_guard<std::mutex> l(mLock);
    if (mParameterResultMap.find(frameNumber) != mParameterResultMap.end()) {
        delete mParameterResultMap[frameNumber];
        mParameterResultMap.erase(frameNumber);
    }
}

}  // namespace camera3
