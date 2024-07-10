/*
 * Copyright (C) 2022 Intel Corporation
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
#define LOG_TAG FaceDetectionResultCallbackManager

#include "aal/FaceDetectionResultCallbackManager.h"

#include <base/no_destructor.h>
#include <base/synchronization/lock.h>
#include <cros-camera/cros_camera_hal.h>

#include "iutils/CameraLog.h"

namespace camera3 {

FaceDetectionResultCallbackManager& FaceDetectionResultCallbackManager::getInstance() {
    static base::NoDestructor<FaceDetectionResultCallbackManager> instance;
    return *instance;
}

void FaceDetectionResultCallbackManager::setCallback(int cameraId,
                                                     cros::FaceDetectionResultCallback callback) {
    base::AutoLock l(mLock);
    if (mCallbacks.count(cameraId) != 0) {
        LOGE("%s: cameraId %d is occupied.", __func__, cameraId);
    }
    mCallbacks[cameraId] = callback;
}

std::optional<cros::FaceDetectionResult>
FaceDetectionResultCallbackManager::getFaceDetectionResult(int cameraId) {
    base::AutoLock l(mLock);
    if (mCallbacks.count(cameraId) == 0) {
        return std::nullopt;
    }
    return mCallbacks.at(cameraId).Run();
}

void FaceDetectionResultCallbackManager::unsetCallback(int cameraId) {
    base::AutoLock l(mLock);
    if (mCallbacks.count(cameraId) != 0) {
        mCallbacks.erase(cameraId);
    }
}

bool FaceDetectionResultCallbackManager::isUsingExternalFDResult(int cameraId) {
    base::AutoLock l(mLock);
    return mCallbacks.count(cameraId) != 0;
}

}  // namespace camera3
