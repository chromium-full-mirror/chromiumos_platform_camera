/*
 * Copyright (C) 2022-2023 Intel Corporation
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

#define LOG_TAG IntelICBMServer

#include "modules/sandboxing/server/IntelICBMServer.h"

#include <sys/stat.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <string>

#include "CameraLog.h"
#include "iutils/Utils.h"

namespace icamera {

#define GET_FUNC_CALL(member, fnName)                                            \
    do {                                                                         \
        mIC2Api->member = (IC2ApiHandle::pFn##member)dlsym(mIC2Handle, #fnName); \
        if (mIC2Api->member == nullptr) {                                        \
            LOGE("@%s: LOADING: " #fnName "failed: %s", __func__, dlerror());    \
            return false;                                                        \
        }                                                                        \
        LOG2("@%s: LOADING: " #fnName "= %x", __func__, mIC2Api->member);        \
    } while (0)

bool IntelICBMServer::loadIC2Library() {
    std::string fullName = IC2_LIB_NAME;
    if (mLibRootPath.length() > 0) fullName = mLibRootPath + IC2_LIB_NAME;

    mIC2Handle = dlopen(fullName.c_str(), RTLD_NOW);
    CheckAndLogError(!mIC2Handle, false, "%s, failed to open library: %s, error: %s", __func__,
                     fullName.c_str(), dlerror());

    GET_FUNC_CALL(query_version, iaic_query_version);
    GET_FUNC_CALL(startup, iaic_startup);
    GET_FUNC_CALL(query_features, iaic_query_features);
    GET_FUNC_CALL(set_loglevel, iaic_set_loglevel);
    GET_FUNC_CALL(create_session, iaic_create_session);
    GET_FUNC_CALL(close_session, iaic_close_session);
    GET_FUNC_CALL(execute, iaic_execute);
    GET_FUNC_CALL(set_data, iaic_set_data);
    GET_FUNC_CALL(get_data, iaic_get_data);
    return true;
}

IntelICBMServer::IntelICBMServer() {
    mIC2Api = std::make_shared<IC2ApiHandle>();
}

IntelICBMServer::~IntelICBMServer() {
    mIC2Api = nullptr;
    if (mIC2Handle) dlclose(mIC2Handle);
    mIC2Handle = nullptr;
}

int IntelICBMServer::setup(ICBMInitInfo* initParam) {
    if (initParam->libPath != nullptr) {
        struct stat buffer;
        // update the path when the level-zero libs exist
        if (stat((std::string(initParam->libPath) + LIBFS_LIB_PREFIX + IC2_LIB_NAME).c_str(),
                 &buffer) == 0)
            mLibRootPath = std::string(initParam->libPath) + LIBFS_LIB_PREFIX;
    }

    // if lib path is not ready, IC2 will try load level-zero from system path
    if (mLibRootPath.length() != 0) {
        // set level0 path to system env, IC2 will load level0 libs from lib path
        LOG1("%s  ICBM lib path %s", __func__, mLibRootPath.c_str());
        setenv(LEVEL_ZERO_GPU_LIB, mLibRootPath.c_str(), 1);
        setenv(LEVEL_ZERO_GPU_CONFIG,
               (std::string(initParam->libPath) + LIBFS_CONFIG_PREFIX).c_str(), 1);
    }

    if (mIC2Handle == nullptr && !loadIC2Library()) return icamera::NO_INIT;
    mIntelICBM = std::unique_ptr<IntelICBM>(new IntelICBM());
    return mIntelICBM->setup(initParam, mIC2Api);
}

int IntelICBMServer::shutdown(const ICBMReqInfo& reqInfo) {
    CheckAndLogError(!mIntelICBM, UNKNOWN_ERROR, "%s, no ICBM instance", __func__);
    int ret = mIntelICBM->shutdown(reqInfo);
    mIntelICBM = nullptr;

    return ret >= 0 ? OK : ret;
}

int IntelICBMServer::processFrame(const ICBMReqInfo& reqInfo) {
    CheckAndLogError(!mIntelICBM, UNKNOWN_ERROR, "%s, no ICBM instance", __func__);
    return mIntelICBM->processFrame(reqInfo);
}

}  // namespace icamera
