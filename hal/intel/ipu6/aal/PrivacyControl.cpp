/*
 * Copyright (C) 2023 Intel Corporation
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

#define LOG_TAG PrivacyControl

#include "aal/PrivacyControl.h"

#include <cros-camera/camera_buffer_manager.h>

#include <algorithm>
#include <string>

#include "Errors.h"
#include "HALv3Utils.h"
#include "Utils.h"
#include "MetadataConvert.h"

namespace camera3 {

bool PrivacyControl::gPrivacyMode = false;
std::mutex PrivacyControl::gPrivacyLock;

PrivacyControl::PrivacyControl(int cameraId, const camera3_callback_ops_t* callback,
                               RequestManagerCallback* requestManagerCallback)
        : mCameraId(cameraId),
          mLastTimestamp(0L),
          mLastSettings(nullptr),
          mThreadRunning(false),
          mCallbackOps(callback),
          mRequestManagerCallback(requestManagerCallback),
          mJpegProcessor(nullptr),
          mInternalBuf(nullptr) {
    LOG1("@<id:%d>%s", mCameraId, __func__);

    mLastSettings = new MetadataMemory();
    mJpegProcessor = std::make_unique<icamera::JpegProcess>(mCameraId);
}

PrivacyControl::~PrivacyControl() {
    stop();
    delete mLastSettings;

    std::lock_guard<std::mutex> l(mLock);
    while (mCaptureRequest.size() > 0) {
        mCaptureRequest.pop();
    }
}

int PrivacyControl::start() {
    LOG1("@%s", __func__);

    std::lock_guard<std::mutex> l(mLock);
    std::string threadName = "PrivacyControl";
    run(threadName);

    // Set the initial timestamp of privacy mode to monotonic
    // time which is same as physical camera device
    struct timespec t = {};
    clock_gettime(CLOCK_MONOTONIC, &t);
    mLastTimestamp = (uint64_t)(t.tv_sec) * 1000000000UL +
                     (uint64_t)(t.tv_nsec) / 1000UL * 1000UL;

    mThreadRunning = true;
    mRequestCondition.notify_one();

    return icamera::OK;
}

void PrivacyControl::stop() {
    LOG1("@%s", __func__);

    icamera::Thread::requestExit();

    {
        std::lock_guard<std::mutex> l(mLock);
        mThreadRunning = false;
        mRequestCondition.notify_one();
    }

    icamera::Thread::requestExitAndWait();
}

void PrivacyControl::setPrivacyMode(bool enable) {
    std::lock_guard<std::mutex> l(gPrivacyLock);
    gPrivacyMode = enable;
}

bool PrivacyControl::getPrivacyMode() {
    std::lock_guard<std::mutex> l(gPrivacyLock);
    LOG2("%s, mode: %d", __func__, gPrivacyMode);
    return gPrivacyMode;
}

int PrivacyControl::configure(const icamera::stream_t& halStream,
                              const camera3_stream_t& stream) {
    if (mInternalBuf && (mInternalBuf->width() != stream.width ||
        mInternalBuf->height() != stream.height)) {
        mInternalBuf.reset();
        mInternalBuf = nullptr;
    }
    if (!mInternalBuf) {
        mInternalBuf = MemoryUtils::allocateHandleBuffer(
                stream.width, stream.height, HalV3Utils::V4l2FormatToHALFormat(halStream.format),
                stream.usage, mCameraId);
        if (!mInternalBuf || mInternalBuf->lock() != icamera::OK) {
            mInternalBuf = nullptr;
            LOGE("%s, Failed to allocate the internal buffer", __func__);
            return icamera::UNKNOWN_ERROR;
        }

        int offset = mInternalBuf->width() * mInternalBuf->height();
        memset(mInternalBuf->data(), 16, offset);
        memset((static_cast<char*>(mInternalBuf->data()) + offset), 128,
               (mInternalBuf->size() - offset));
    }

    return icamera::OK;
}

int PrivacyControl::processCaptureRequest(camera3_capture_request_t* request) {
    CheckAndLogError(!request, icamera::BAD_VALUE, "@%s, request is nullptr", __func__);
    LOG2("<fn%d>@%s buffer count %d", request->frame_number, __func__, request->num_output_buffers);

    std::shared_ptr<CaptureRequest> captureReq = std::make_shared<CaptureRequest>();
    captureReq->frameNumber = request->frame_number;
    captureReq->inputBuffer = request->input_buffer;

    // Copy settings
    if (request->settings) {
        mLastSettings->copyMetadata(request->settings);
    }
    captureReq->metadata = std::make_shared<MetadataMemory>();
    captureReq->metadata->copyMetadata(mLastSettings->getMemory());

    CLEAR(captureReq->outputBuffers);
    captureReq->outputCam3Bufs.clear();
    for (int i = 0; i < request->num_output_buffers; i++) {
        const camera3_stream_t* stream = request->output_buffers[i].stream;
        if (stream->stream_type != CAMERA3_STREAM_OUTPUT) continue;

        std::shared_ptr<Camera3Buffer> ccBuf = std::make_shared<Camera3Buffer>();
        icamera::status_t status = ccBuf->init(&request->output_buffers[i], mCameraId);
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE,
                         "%s, Failed to init Camera3Buffer for index(%d)", __func__, i);
        status = ccBuf->waitOnAcquireFence();
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE,
                         "%s, Failed to sync Camera3Buffer for index(%d)", __func__, i);
        status = ccBuf->lock();
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE,
                         "%s, Failed to lock Camera3Buffer for index(%d)", __func__, i);
        captureReq->outputCam3Bufs.push_back(ccBuf);
        captureReq->outputBuffers[i] = request->output_buffers[i];
    }

    std::lock_guard<std::mutex> lock(mLock);
    mCaptureRequest.push(captureReq);

    if (mThreadRunning) {
        mRequestCondition.notify_one();
    }

    return icamera::OK;
}

void PrivacyControl::updateMetadataResult(android::CameraMetadata* settings) {
    int64_t rollingShutter = 33333000;
    settings->update(ANDROID_SENSOR_ROLLING_SHUTTER_SKEW, &rollingShutter, 1);

    uint8_t aeState = ANDROID_CONTROL_AE_STATE_CONVERGED;
    camera_metadata_entry entry = settings->find(ANDROID_CONTROL_AE_LOCK);
    if (entry.count == 1 && entry.data.u8[0] == ANDROID_CONTROL_AE_LOCK_ON) {
        aeState = ANDROID_CONTROL_AE_STATE_LOCKED;
    }
    settings->update(ANDROID_CONTROL_AE_STATE, &aeState, 1);

    uint8_t awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    entry = settings->find(ANDROID_CONTROL_AWB_LOCK);
    if (entry.count == 1 && entry.data.u8[0] == ANDROID_CONTROL_AWB_LOCK_ON) {
        awbState = ANDROID_CONTROL_AE_STATE_LOCKED;
    }
    settings->update(ANDROID_CONTROL_AWB_STATE, &awbState, 1);

    int faceIds[1] = {-1};
    settings->update(ANDROID_STATISTICS_FACE_IDS, faceIds, 1);

    int64_t exposureTime = 33333000;
    settings->update(ANDROID_SENSOR_EXPOSURE_TIME, &exposureTime, 1);

    uint8_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
    entry = settings->find(ANDROID_CONTROL_AF_TRIGGER);
    if (entry.count == 1) {
        afTrigger = entry.data.u8[0];
    }

    uint8_t afMode = ANDROID_CONTROL_AF_MODE_AUTO;
    entry = settings->find(ANDROID_CONTROL_AF_MODE);
    if (entry.count == 1) {
        afMode = entry.data.u8[0];
    }

    uint8_t afState = ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED;
    if ((afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO ||
        afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE) &&
        afTrigger == ANDROID_CONTROL_AF_TRIGGER_START) {
        afState = ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    }
    settings->update(ANDROID_CONTROL_AF_STATE, &afState, 1);

    uint8_t lensState = 0;
    settings->update(ANDROID_LENS_STATE, &lensState, 1);
}

bool PrivacyControl::threadLoop() {
    LOG2("[%p]@%s", this, __func__);

    std::shared_ptr<CaptureRequest> captureReq = nullptr;
    {
        std::unique_lock<std::mutex> lock(mLock);
        if (!mThreadRunning) return false;

        if (mCaptureRequest.empty()) {
            std::cv_status ret = mRequestCondition.wait_for(
                lock, std::chrono::nanoseconds(kMaxDuration * SLOWLY_MULTIPLIER));

            // Already stopped
            if (!mThreadRunning) return false;

            if (ret == std::cv_status::timeout) {
                LOG2("[%p]%s, wait request time out", this, __func__);
            }
            return true;
        }

        captureReq = mCaptureRequest.front();
        mCaptureRequest.pop();
    }
    CheckAndLogError(!captureReq, false, "@%s, the captureReq is nullptr", __func__);

    struct timeval curTime = {};
    gettimeofday(&curTime, nullptr);

    // Set the max fps in the range
    int frameInterval = 1000000.0 / 30;
    android::CameraMetadata* meta = captureReq->metadata->getMetadata();
    if (meta) {
        camera_metadata_entry entry = meta->find(ANDROID_CONTROL_AE_TARGET_FPS_RANGE);
        if (entry.count == 2) {
            frameInterval = 1000000.0 / entry.data.i32[0];
        }
    }

    mLastTimestamp += frameInterval * 1000UL;
    LOG2("%s, set target frame interval: %d, timestamp: %lu", __func__, frameInterval,
         mLastTimestamp);

    // notify shutter done
    camera3_notify_msg_t notifyMsg = {};
    notifyMsg.type = CAMERA3_MSG_SHUTTER;
    notifyMsg.message.shutter.frame_number = captureReq->frameNumber;
    notifyMsg.message.shutter.timestamp = mLastTimestamp;

    if (!captureReq->inputBuffer && meta) {
        meta->update(ANDROID_SENSOR_TIMESTAMP,
                     reinterpret_cast<const int64_t*>(&mLastTimestamp), 1);
    } else if (meta) {
        // update shutter timestamp if there is input stream
        camera_metadata_entry entry = meta->find(ANDROID_SENSOR_TIMESTAMP);
        if (entry.count == 1) {
            notifyMsg.message.shutter.timestamp = entry.data.i64[0];
        }
    }
    mCallbackOps->notify(mCallbackOps, &notifyMsg);

    if (meta) {
        updateMetadataResult(meta);
    }

    for (size_t i = 0; i < captureReq->outputCam3Bufs.size(); i++) {
        camera3_stream_t* stream = captureReq->outputBuffers[i].stream;
        if (stream->stream_type != CAMERA3_STREAM_OUTPUT || IS_ZSL_USAGE(stream->usage))
            continue;

        if (stream->format == HAL_PIXEL_FORMAT_BLOB) {
            icamera::Parameters param;
            bool forceConvert = captureReq->inputBuffer ? true : false;
            MetadataConvert::requestMetadataToHALMetadata(*(captureReq->metadata->getMetadata()),
                                                          &param, forceConvert);
            mJpegProcessor->doPostProcessing(mInternalBuf, param, captureReq->outputCam3Bufs[i]);
        } else {
            // fill black feeds for all output buffers
            int offset = stream->width * stream->height;
            memset(captureReq->outputCam3Bufs[i]->data(), 16, offset);
            memset((static_cast<char*>(captureReq->outputCam3Bufs[i]->data()) + offset), 128,
                   (captureReq->outputCam3Bufs[i]->size() - offset));
        }

        captureReq->outputCam3Bufs[i]->unlock();
        captureReq->outputCam3Bufs[i]->deinit();
        captureReq->outputCam3Bufs[i]->getFence(&(captureReq->outputBuffers[i]));
    }

    // notify callback event
    camera3_capture_result_t result = {};
    result.frame_number = captureReq->frameNumber;
    result.output_buffers = captureReq->outputBuffers;
    result.num_output_buffers = captureReq->outputCam3Bufs.size();
    result.result = captureReq->metadata->getMemory();
    result.partial_result = 1;
    result.input_buffer = captureReq->inputBuffer;
    result.num_physcam_metadata = 0;
    result.physcam_ids = nullptr;
    result.physcam_metadata = nullptr;

    struct timeval tmpCurTime = {};
    gettimeofday(&tmpCurTime, nullptr);
    int duration = static_cast<int>(tmpCurTime.tv_usec - curTime.tv_usec +
                                    ((tmpCurTime.tv_sec - curTime.tv_sec) * 1000000UL));

    LOG2("%s, sleep to wait FPS duration: %ld", __func__, frameInterval - duration);
    if (frameInterval - duration < 0) {
        LOG2("@%s, take too long to fill the buffers, skip sleep", __func__);
    } else {
        usleep(frameInterval - duration);
    }
    mCallbackOps->process_capture_result(mCallbackOps, &result);

    mRequestManagerCallback->returnRequestDone(captureReq->frameNumber);
    LOG2("@%s handle frame %d completed", __func__, captureReq->frameNumber);

    return true;
}

}  // namespace camera3
