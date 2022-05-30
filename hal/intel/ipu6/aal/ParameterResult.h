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

#include <mutex>
#include <unordered_map>

#include "Parameters.h"
#include "Utils.h"

namespace camera3 {

/**
 * \class ParameterResult
 *
 * This class is used to manage camera metadata result got from libcamhal.
 */
class ParameterResult {
 public:
    explicit ParameterResult(int cameraId);
    ~ParameterResult();

    icamera::Parameters* getParameter(uint32_t frameNumber, int64_t sequence);
    void releaseParameter(uint32_t frameNumber);

 private:
    int mCameraId;

    std::mutex mLock;  // Guard mParameterResultMap
    // first: frame number, second: Parameters
    std::unordered_map<uint32_t, icamera::Parameters*> mParameterResultMap;

 private:
    DISALLOW_COPY_AND_ASSIGN(ParameterResult);
};

}  // namespace camera3
