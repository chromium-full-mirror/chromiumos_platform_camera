/*
 * Copyright (C) 2020 Intel Corporation.
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

#define LOG_TAG "IntelCPUAlgoServer"

#include "modules/sandboxing/server/IntelCPUAlgoServer.h"

#include <base/logging.h>
#include <ia_log.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <memory>
#include <string>

#include "iutils/Utils.h"

namespace icamera {

IntelCPUAlgoServer::IntelCPUAlgoServer(IntelAlgoServer* server) : RequestHandler(server) {
    LOGIPC("@%s", __func__);
}

void IntelCPUAlgoServer::handleRequest(const MsgReq& msg) {
    uint32_t req_id = msg.req_id;
    int32_t buffer_handle = msg.buffer_handle;

    ShmInfo info = {};
    status_t status = getIntelAlgoServer()->getShmInfo(buffer_handle, &info);
    if (status != OK) {
        LOGE("@%s, Invalid buffer handle", __func__);
        getIntelAlgoServer()->returnCallback(req_id, UNKNOWN_ERROR, buffer_handle);
        return;
    }

    size_t requestSize = info.size;
    void* addr = info.addr;
    LOGIPC("@%s, req_id:%d:%s, requestSize:%zu, addr:%p, buffer_handle:%d", __func__, req_id,
           IntelAlgoIpcCmdToString(static_cast<IPC_CMD>(req_id)), requestSize, addr, buffer_handle);

    switch (req_id) {
        case IPC_FD_INIT:
            status = mFaceDetection.init(addr, requestSize);
            break;
        case IPC_FD_RUN: {
            FaceDetectionRunParams* palParams = static_cast<FaceDetectionRunParams*>(addr);
            void* addrImage = nullptr;
            if (palParams->bufferHandle >= 0) {
                ShmInfo imageDataInfo;
                status = getIntelAlgoServer()->getShmInfo(palParams->bufferHandle, &imageDataInfo);
                if (status != OK) {
                    LOGE("%s, the buffer handle for image data is invalid", __func__);
                    break;
                }
                addrImage = imageDataInfo.addr;
            }
            status = mFaceDetection.run(addr, requestSize, addrImage);
            break;
        }
        case IPC_FD_DEINIT:
            status = mFaceDetection.deinit(addr, requestSize);
            break;
        case IPC_GRAPH_ADD_KEY:
            mGraph.addCustomKeyMap();
            break;
        case IPC_GRAPH_PARSE:
            mGraph.parse(addr, requestSize);
            break;
        case IPC_GRAPH_RELEASE_NODES:
            mGraph.releaseGraphNodes();
            break;
        case IPC_GRAPH_CONFIG_STREAMS:
            mGraph.configStreams(addr, requestSize);
            break;
        case IPC_GRAPH_GET_CONFIG_DATA:
            mGraph.getGraphConfigData(addr, requestSize);
            break;
        case IPC_GRAPH_GET_CONNECTION:
            mGraph.pipelineGetConnections(addr, requestSize);
            break;
        case IPC_GRAPH_GET_PG_ID:
            mGraph.getPgIdForKernel(addr, requestSize);
            break;
        case IPC_CCA_INIT:
            status = mCca.init(addr, requestSize);
            break;
        case IPC_CCA_SET_STATS:
            status = mCca.setStats(addr, requestSize);
            break;
        case IPC_CCA_RUN_AEC:
            status = mCca.runAEC(addr, requestSize);
            break;
        case IPC_CCA_RUN_AIQ:
            status = mCca.runAIQ(addr, requestSize);
            break;
        case IPC_CCA_RUN_LTM:
            status = mCca.runLTM(addr, requestSize);
            break;
        case IPC_CCA_UPDATE_ZOOM:
            status = mCca.updateZoom(addr, requestSize);
            break;
        case IPC_CCA_RUN_DVS:
            status = mCca.runDVS(addr, requestSize);
            break;
        case IPC_CCA_RUN_AIC: {
            status = UNKNOWN_ERROR;
            intel_cca_run_aic_data* params = static_cast<intel_cca_run_aic_data*>(addr);
            if (params->palDataHandle >= 0) {
                ShmInfo info = {};
                status = getIntelAlgoServer()->getShmInfo(params->palDataHandle, &info);
                if (status != OK) {
                    LOGE("%s, the buffer handle for image data is invalid", __func__);
                    break;
                }
                params->palOutData.data = info.addr;

                status = mCca.runAIC(addr, requestSize);
            }
            break;
        }
        case IPC_CCA_GET_CMC:
            status = mCca.getCMC(addr, requestSize);
            break;
        case IPC_CCA_GET_MKN:
            status = mCca.getMKN(addr, requestSize);
            break;
        case IPC_CCA_GET_AIQD:
            status = mCca.getAiqd(addr, requestSize);
            break;
        case IPC_CCA_UPDATE_TUNING:
            mCca.updateTuning(addr, requestSize);
            break;
        case IPC_CCA_DEINIT:
            status = mCca.deinit();
            break;
        case IPC_CCA_GET_VERSION:
            status = mCca.getVersion(addr, requestSize);
            break;
        case IPC_CCA_DECODE_STATS: {
            intel_cca_decode_stats_data* params = static_cast<intel_cca_decode_stats_data*>(addr);
            ShmInfo info = {};
            status = getIntelAlgoServer()->getShmInfo(params->statsHandle, &info);
            if (status != OK) {
                LOGE("%s, the buffer handle for stats data is invalid", __func__);
                break;
            }
            status = mCca.decodeStats(addr, requestSize, info.addr);
            break;
        }
        case IPC_CCA_GET_PAL_SIZE:
            status = mCca.getPalDataSize(addr, requestSize);
            break;
        case IPC_PG_PARAM_INIT:
            status = mPGParam.init(addr, requestSize);
            break;
        case IPC_PG_PARAM_PREPARE: {
            pg_param_prepare_params* prepareParams = static_cast<pg_param_prepare_params*>(addr);
            ShmInfo palDataInfo = {};
            status = getIntelAlgoServer()->getShmInfo(prepareParams->ipuParamHandle, &palDataInfo);
            if (status != OK) {
                LOGE("%s, the buffer handle for pal data is invalid", __func__);
                break;
            }
            status = mPGParam.prepare(addr, requestSize, palDataInfo.addr);
            break;
        }
        case IPC_PG_PARAM_ALLOCATE_PG:
            status = mPGParam.allocatePGBuffer(addr, requestSize);
            break;
        case IPC_PG_PARAM_GET_FRAG_DESCS:
            status = mPGParam.getFragmentDescriptors(addr, requestSize);
            break;
        case IPC_PG_PARAM_PREPARE_PROGRAM:
            status = mPGParam.setPGAndPrepareProgram(addr, requestSize);
            break;
        case IPC_PG_PARAM_REGISTER_PAYLOADS:
            status = mPGParam.registerPayloads(addr, requestSize);
            break;
        case IPC_PG_PARAM_ENCODE: {
            pg_param_encode_params* encodeParams = static_cast<pg_param_encode_params*>(addr);
            ShmInfo palDataInfo = {};
            status = getIntelAlgoServer()->getShmInfo(encodeParams->ipuParamHandle, &palDataInfo);
            if (status != OK) {
                LOGE("%s, the buffer handle for pal data is invalid", __func__);
                break;
            }
            status = mPGParam.updatePALAndEncode(addr, requestSize, palDataInfo.addr);
            break;
        }
        case IPC_PG_PARAM_DECODE: {
            pg_param_decode_params* decodeParams = static_cast<pg_param_decode_params*>(addr);
            ShmInfo decodeInfo = {};
            status = getIntelAlgoServer()->getShmInfo(decodeParams->clientStatsHandle, &decodeInfo);
            if (status == OK) {
                status = mPGParam.decode(addr, requestSize, decodeInfo.addr);
            } else {
                LOGE("%s, the buffer handle for stats data is invalid", __func__);
            }
            break;
        }
        case IPC_PG_PARAM_DEINIT:
            mPGParam.deinit(addr, requestSize);
            break;
        default:
            LOGE("@%s, req_id:%d is not defined", __func__, req_id);
            status = UNKNOWN_ERROR;
            break;
    }

    LOGIPC("@%s, req_id:%d:%s, status:%d", __func__, req_id,
           IntelAlgoIpcCmdToString(static_cast<IPC_CMD>(req_id)), status);
    getIntelAlgoServer()->returnCallback(req_id, status, buffer_handle);
}
}  // namespace icamera
