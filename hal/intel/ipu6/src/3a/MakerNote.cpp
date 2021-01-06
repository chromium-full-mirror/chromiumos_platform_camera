/*
 * Copyright (C) 2018-2020 Intel Corporation.
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

#define LOG_TAG "MakerNote"

#include "src/3a/MakerNote.h"

#include <memory>

#include "CameraDevice.h"
#include "iutils/CameraLog.h"
#include "iutils/Errors.h"

namespace icamera {

MakerNote::MakerNote() : mMknState(UNINIT) {
    LOG1("@%s", __func__);

    for (int i = 0; i < MAX_MAKER_NOTE_LIST_SIZE; i++) {
        std::shared_ptr<MakernoteData> data = std::shared_ptr<MakernoteData>(new MakernoteData());
        mMakernoteDataList.push_back(data);
    }

    mMknState = INIT;
}

MakerNote::~MakerNote() {
    LOG1("@%s", __func__);

    AutoMutex lock(mMknLock);

    mMakernoteDataList.clear();
}

int MakerNote::saveMakernoteData(int cameraId, camera_makernote_mode_t makernoteMode,
                                 int64_t sequence, TuningMode tuningMode) {
    LOG1("@%s", __func__);
    if (makernoteMode == MAKERNOTE_MODE_OFF) return OK;

    AutoMutex lock(mMknLock);
    CheckError(mMknState != INIT, BAD_VALUE, "@%s, mkn isn't initialized", __func__);

    ia_mkn_trg mknTrg =
        (makernoteMode == MAKERNOTE_MODE_JPEG ? ia_mkn_trg_section_1 : ia_mkn_trg_section_2);
    std::shared_ptr<MakernoteData> data = mMakernoteDataList.front();

    // Only getMKN returns OK, it will change data->mknData, otherwise it will not be changed.
    IntelCca* intelCcaHandle = IntelCca::getInstance(cameraId, tuningMode);
    CheckError(!intelCcaHandle, BAD_VALUE, "@%s, Failed to get IntelCca instance", __func__);
    ia_err iaErr = intelCcaHandle->getMKN(mknTrg, &data->mknData);
    int ret = AiqUtils::convertError(iaErr);
    CheckError(ret != OK, ret, "@%s, Failed to getMKN", __func__);

    mMakernoteDataList.pop_front();
    data->sequence = sequence;
    data->timestamp = 0;
    LOG2("@%s, saved makernote %d for sequence %ld", __func__, makernoteMode, sequence);

    mMakernoteDataList.push_back(data);
    return OK;
}

void MakerNote::updateTimestamp(int64_t sequence, uint64_t timestamp) {
    LOG1("@%s", __func__);
    AutoMutex lock(mMknLock);
    CheckError(mMknState != INIT, VOID_VALUE, "@%s, mkn isn't initialized", __func__);

    for (auto rit = mMakernoteDataList.rbegin(); rit != mMakernoteDataList.rend(); ++rit) {
        if ((*rit)->sequence == sequence) {
            LOG2("@%s, found sequence %ld for request sequence %ld, timestamp %ld", __func__,
                 (*rit)->sequence, sequence, timestamp);
            (*rit)->timestamp = timestamp;
            break;
        }
    }
}

void MakerNote::acquireMakernoteData(uint64_t timestamp, Parameters* param) {
    LOG1("@%s", __func__);
    AutoMutex lock(mMknLock);
    CheckError(mMknState != INIT, VOID_VALUE, "@%s, mkn isn't initialized", __func__);

    for (auto rit = mMakernoteDataList.rbegin(); rit != mMakernoteDataList.rend(); ++rit) {
        if ((*rit)->timestamp > 0 && timestamp >= (*rit)->timestamp) {
            LOG2("@%s, found timestamp %ld for request timestamp %ld", __func__, (*rit)->timestamp,
                 timestamp);
            param->setMakernoteData((*rit)->mknData.buf, (*rit)->mknData.size);
            break;
        }
    }
}

}  // namespace icamera
