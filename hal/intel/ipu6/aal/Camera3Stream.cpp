/*
 * Copyright (C) 2017-2023 Intel Corporation
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

#define LOG_TAG Camera3Stream

#include "Camera3Stream.h"

#include <utility>
#include <vector>
#include <memory>

#include "CameraDump.h"
#include "ParameterResult.h"
#include "PlatformData.h"
#include "Utils.h"

namespace camera3 {
Camera3Stream::Camera3Stream(int cameraId, CallbackEventInterface* callback,
                             ParameterResult* result, uint32_t maxNumReqInProc,
                             const icamera::stream_t& halStream, const camera3_stream_t& stream,
                             const camera3_stream_t* inputStream)
        : mCameraId(cameraId),
          mEventCallback(callback),
          mParameterResult(result),
          mPostProcessType(icamera::POST_PROCESS_NONE),
          mStreamState(false),
          mHALStream(halStream),
          mMaxNumReqInProc(maxNumReqInProc),
          mStream(stream),
          mStreamStatus(PROCESS_REQUESTS),
          mFaceDetection(nullptr),
          mInputPostProcessType(icamera::POST_PROCESS_NONE) {
    LOG1("[%p]@%s, buf num:%d, inputStream:%p, stream:%dx%d, format:%d, type:%d", this, __func__,
         mMaxNumReqInProc, inputStream, mHALStream.width, mHALStream.height, mHALStream.format,
         mStream.stream_type);

    mPostProcessor = std::unique_ptr<PostProcessor>(new PostProcessor(mCameraId, stream));

    if (inputStream) {
        LOG2("@%s, inputStream: width:%d, height:%d, format:%d", __func__, inputStream->width,
             inputStream->height, inputStream->format);

        mInputPostProcessor = std::unique_ptr<PostProcessor>(new PostProcessor(mCameraId, stream));
        mInputStream = std::unique_ptr<camera3_stream_t>(new camera3_stream_t);
        *mInputStream = *inputStream;
    }
}

Camera3Stream::~Camera3Stream() {
    LOG1("[%p]@%s", this, __func__);

    setActive(false);

    for (auto& iter : mCaptureResultMap) {
        std::shared_ptr<Camera3Buffer>& buf = iter.second->outputCam3Buf;
        if (buf && buf->isLocked()) {
            buf->unlock();
        }
    }

    std::lock_guard<std::mutex> l(mLock);
    mCaptureResultMap.clear();
}

void Camera3Stream::drainAllPendingRequests() {
    std::lock_guard<std::mutex> l(mLock);

    mStreamStatus = DRAIN_REQUESTS;
    for (auto& iter : mPendingRequests) mCaptureResultMap.insert(iter);
    mPendingRequests.clear();

    mBufferDoneCondition.notify_one();
}

void Camera3Stream::drainRequest() {
    LOG1("[%p] %s", this, __func__);
    if (mCaptureResultMap.empty()) {
        mStreamStatus = PEND_PROCESS;
        return;
    }

    auto captureResult = mCaptureResultMap.begin();
    std::shared_ptr<CaptureResult> result = captureResult->second;
    uint32_t frameNumber = captureResult->first;

    CheckAndLogError(result == nullptr, VOID_VALUE, "<fn%u>%s, capture result is nullptr",
                     frameNumber, __func__);
    std::shared_ptr<Camera3Buffer>& ccBuf = result->outputCam3Buf;
    if (ccBuf) {
        camera3_stream_buffer* buf = &result->outputBuffer;

        CheckAndLogError(ccBuf == nullptr, VOID_VALUE, "<fn%u>%s, ccBuf is nullptr", frameNumber,
                         __func__);
        if (ccBuf->isLocked()) {
            ccBuf->unlock();
        }
        ccBuf->deinit();
        ccBuf->getFence(buf);

        buf->status = CAMERA3_BUFFER_STATUS_ERROR;

        // notify frame done
        BufferEvent bufferEvent = {frameNumber, buf, 0, -1};
        mEventCallback->bufferDone(bufferEvent);
    }
    mCaptureResultMap.erase(frameNumber);
}

bool Camera3Stream::handleNewFrame(std::shared_ptr<StreamComInfo> hCapture,
                                   std::shared_ptr<CaptureResult> result) {
    std::shared_ptr<Camera3Buffer> inputCam3Buf = result->inputCam3Buf;
    uint32_t frameNumber = result->frameNumber;
    icamera::Parameters* parameter = nullptr;
    uint64_t timestamp = 0;
    if (!inputCam3Buf) {
        if (mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW) {
            parameter = mParameterResult->getParameter(frameNumber, hCapture->sequence);
        }
        timestamp = hCapture->cam3Buf->getHalBuffer().timestamp;
    }

    // start process buffers, HAL stream and listeners will do the same process
    std::shared_ptr<Camera3Buffer>& ccBuf = result->outputCam3Buf;
    CheckAndLogError(!ccBuf, false, "<fn%u>ccBuf is nullptr", frameNumber);

    if (inputCam3Buf) {
        // notify shutter/metadata done for yuv reprocessing cases
        ShutterEvent shutterEvent = {frameNumber, timestamp};
        mEventCallback->shutterDone(shutterEvent);

        MetadataEvent event = {frameNumber, parameter};
        mEventCallback->metadataDone(event);
    } else if (mHALStream.usage == icamera::CAMERA_STREAM_OPAQUE_RAW) {
        // notify shutter done before buffer done for raw output
        ShutterEvent shutterEvent = {frameNumber, timestamp};
        mEventCallback->shutterDone(shutterEvent);
    }

    int dumpOutputFmt = V4L2_PIX_FMT_NV12;
    int ret = icamera::OK;
    if (!inputCam3Buf && mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW) {
        LOG2("<fn%u>%s, hal buffer: %p, ccBuf address: %p", hCapture->frameNumber, __func__,
             hCapture->cam3Buf->data(), ccBuf->data());
        if (mPostProcessType & icamera::POST_PROCESS_JPEG_ENCODING)
            dumpOutputFmt = V4L2_PIX_FMT_JPEG;
        ret = handleCaptureRequest(hCapture->cam3Buf, ccBuf, parameter);
    } else if (inputCam3Buf) {
        LOG2("<fn%u>[%p]@%s process input", frameNumber, this, __func__);
        if (mInputPostProcessType & icamera::POST_PROCESS_JPEG_ENCODING)
            dumpOutputFmt = V4L2_PIX_FMT_JPEG;
        inputCam3Buf->dumpImage(frameNumber, icamera::DUMP_AAL_INPUT, V4L2_PIX_FMT_NV12);
        ret = handleReprocessRequest(inputCam3Buf, ccBuf, &result->param);
    }
    CheckAndLogError(ret != icamera::OK, true, "doPostProcessing fails");

    // Face only run in preview stream, so parameter mustn't be nullptr in this Camera3Stream
    if (mFaceDetection) {
        uint8_t faceDetectMode = icamera::FD_MODE_OFF;
        if (parameter) {
            parameter->getFaceDetectMode(faceDetectMode);
        }
        if ((icamera::PlatformData::isFaceAeEnabled(mCameraId) ||
             faceDetectMode != icamera::FD_MODE_OFF) &&
            mFaceDetection->faceRunningByCondition() &&
            (ccBuf->isLocked() || ccBuf->lock() == icamera::OK)) {
            mFaceDetection->runFaceDetection(ccBuf);
        }
    }

    if (mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW) {
        if (dumpOutputFmt == V4L2_PIX_FMT_JPEG) {
            ccBuf->dumpImage(frameNumber, icamera::DUMP_AAL_OUTPUT | icamera::DUMP_JPEG_BUFFER,
                             dumpOutputFmt);
        } else {
            ccBuf->dumpImage(frameNumber, icamera::DUMP_AAL_OUTPUT, dumpOutputFmt);
        }
    }
    if (ccBuf->isLocked()) {
        ccBuf->unlock();
    }
    ccBuf->deinit();
    ccBuf->getFence(&result->outputBuffer);

    // notify frame done
    BufferEvent bufferEvent = {frameNumber, &result->outputBuffer, 0, -1};
    if (mHALStream.usage == icamera::CAMERA_STREAM_OPAQUE_RAW) {
        bufferEvent.sequence = hCapture->sequence;
        bufferEvent.timestamp = timestamp;
    }
    mEventCallback->bufferDone(bufferEvent);

    return true;
}

int Camera3Stream::handleCaptureRequest(std::shared_ptr<Camera3Buffer> inBuf,
                                        std::shared_ptr<Camera3Buffer> outBuf,
                                        icamera::Parameters* parameter) {
    icamera::camera_buffer_t inBufHandle = inBuf->getHalBuffer();

    if (mPostProcessType & icamera::POST_PROCESS_JPEG_ENCODING) {
        icamera::PlatformData::acquireMakernoteData(mCameraId, inBufHandle.timestamp, parameter);
    }
    // handle normal postprocess
    if (mPostProcessType != icamera::POST_PROCESS_NONE) {
        icamera::status_t status = mPostProcessor->doPostProcessing(inBuf, *parameter, outBuf);
        CheckAndLogError(status != icamera::OK, status,
                         "<seq%ld>@%s, doPostProcessing fails, mPostProcessType:%d",
                         inBufHandle.sequence, __func__, mPostProcessType);
    } else if (inBuf->data() != outBuf->data()) {
        MEMCPY_S(outBuf->data(), outBuf->size(), inBuf->data(), inBuf->size());
    }
    return icamera::OK;
}

int Camera3Stream::handleReprocessRequest(std::shared_ptr<Camera3Buffer> inBuf,
                                          std::shared_ptr<Camera3Buffer> outBuf,
                                          icamera::Parameters* parameter) {
    if (mInputPostProcessType & icamera::POST_PROCESS_JPEG_ENCODING) {
        icamera::PlatformData::acquireMakernoteData(mCameraId, inBuf->getTimeStamp(), parameter);
    }

    if (mInputPostProcessType != icamera::POST_PROCESS_NONE) {
        icamera::status_t status = mInputPostProcessor->doPostProcessing(inBuf, *parameter, outBuf);
        CheckAndLogError(status != icamera::OK, status,
                         "@%s, doPostProcessing fails, mInputPostProcessType:%d", __func__,
                         mInputPostProcessType);
    } else {
        MEMCPY_S(outBuf->data(), outBuf->size(), inBuf->data(), inBuf->size());
    }
    return icamera::OK;
}

void Camera3Stream::requestExit() {
    LOG1("[%p]@%s", this, __func__);

    icamera::Thread::requestExit();
    std::lock_guard<std::mutex> l(mLock);

    mBufferDoneCondition.notify_one();

    if (mFaceDetection) {
        icamera::FaceDetection::destoryInstance(mCameraId);
        mFaceDetection = nullptr;
    }

}

bool Camera3Stream::isLockUserBufferNeeded(const std::shared_ptr<Camera3Buffer>& inputCam3Buf) {
    return (mPostProcessType != icamera::POST_PROCESS_NONE || inputCam3Buf != nullptr ||
            icamera::CameraDump::isDumpTypeEnable(icamera::DUMP_AAL_OUTPUT));
}

int Camera3Stream::processRequest(uint32_t frameNumber,
                                  const std::shared_ptr<Camera3Buffer>& inputCam3Buf,
                                  const camera3_stream_buffer_t& outputBuffer,
                                  const icamera::Parameters& param) {
    LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);

    std::shared_ptr<Camera3Buffer> ccBuf = std::make_shared<Camera3Buffer>();
    icamera::status_t status = ccBuf->init(&outputBuffer, mCameraId);
    CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to init CameraBuffer");
    status = ccBuf->waitOnAcquireFence();
    CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to sync CameraBuffer");
    if (isLockUserBufferNeeded(inputCam3Buf)) {
        status = ccBuf->lock();
        CheckAndLogError(status != icamera::OK, icamera::BAD_VALUE, "Failed to lock buffer");
    }

    std::shared_ptr<CaptureResult> pendingReq = std::make_shared<CaptureResult>();
    pendingReq->frameNumber = frameNumber;
    pendingReq->outputCam3Buf = ccBuf;
    pendingReq->outputBuffer = outputBuffer;
    pendingReq->handle = *outputBuffer.buffer;
    pendingReq->outputBuffer.buffer = &pendingReq->handle;
    pendingReq->inputCam3Buf = inputCam3Buf;
    pendingReq->param = param;
    pendingReq->hCapture = nullptr;
    std::lock_guard<std::mutex> l(mLock);
    mPendingRequests[frameNumber] = pendingReq;

    return icamera::OK;
}

void Camera3Stream::queueBufferDone(uint32_t frameNumber) {
    std::lock_guard<std::mutex> l(mLock);

    if (mPendingRequests.find(frameNumber) != mPendingRequests.end()) {
        LOG2("<fn%u>[%p]@%s", frameNumber, this, __func__);
        mCaptureResultMap[frameNumber] = mPendingRequests[frameNumber];
        mPendingRequests.erase(frameNumber);
        mStreamStatus = PROCESS_REQUESTS;
        mBufferDoneCondition.notify_one();
    }
}

int Camera3Stream::setActive(bool state) {
    if (!mStreamState && state) {
        std::string threadName = "Cam3Stream-";
        threadName += std::to_string(mHALStream.id);

        // Run Camera3Stream thread
        run(threadName);

        if (mHALStream.usage != icamera::CAMERA_STREAM_OPAQUE_RAW) {
            // configure the post processing.
            // Note: the mHALStream may be changed after calling this function
            mPostProcessor->configure(mStream, mHALStream);
            mPostProcessType = mPostProcessor->getPostProcessType();
            LOG2("@%s, mPostProcessType:%d", __func__, mPostProcessType);
        }

        if (mInputPostProcessor) {
            mInputPostProcessor->configure(mStream, *mInputStream.get());
            mInputPostProcessType = mInputPostProcessor->getPostProcessType();
        }
        LOG1("[%p]@%s state %d, mPostProcessType %d, mInputPostProcessor %d", this, __func__, state,
             mPostProcessType, mInputPostProcessType);
    } else if (mStreamState && !state) {
        LOG1("[%p]@%s state %d", this, __func__, state);
        mPostProcessType = icamera::POST_PROCESS_NONE;

        if (mInputPostProcessor) {
            mInputPostProcessType = icamera::POST_PROCESS_NONE;
        }

        // Exit Camera3Stream thread
        requestExit();
    }

    mStreamState = state;

    return icamera::OK;
}

void Camera3Stream::activateFaceDetection(unsigned int maxFaceNum) {
    LOG1("<id%d>[%p]@%s maxFaceNum %d", mCameraId, this, __func__, maxFaceNum);

    mFaceDetection =
        icamera::FaceDetection::createInstance(mCameraId, maxFaceNum, mHALStream.id, mStream.width,
                                               mStream.height, mStream.format, mStream.usage);
}

bool Camera3Stream::requestHALCapture(uint32_t frameNumber,
                                      std::shared_ptr<Camera3Buffer>* cam3Buf) {
    std::lock_guard<std::mutex> l(mLock);
    std::shared_ptr<CaptureResult> result = nullptr;

    if (mPendingRequests.find(frameNumber) != mPendingRequests.end()) {
        result = mPendingRequests[frameNumber];
        if (result->inputCam3Buf) return false;
        if (mPostProcessType == icamera::POST_PROCESS_NONE && cam3Buf) {
            *cam3Buf = result->outputCam3Buf;
        }
        return true;
    }
    return false;
}

}  // namespace camera3
