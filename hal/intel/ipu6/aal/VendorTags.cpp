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

#define LOG_TAG VendorTags

#include "aal/VendorTags.h"

#include "iutils/CameraLog.h"
#include "iutils/Utils.h"
#include "src/metadata/icamera_metadata_base.h"

namespace camera3 {

#include "../src/metadata/vendor_metadata_tag_info.c"

int VendorTags::getTagCount(const vendor_tag_ops_t* v) {
    UNUSED(v);
    int tagCount = 0;
    for (int i = 0; i < INTEL_VENDOR_SECTION_COUNT; i++) {
        tagCount += vendor_metadata_section_bounds[i][1] - vendor_metadata_section_bounds[i][0];
    }

    LOG2("%s, vendor tag count: %d", __func__, tagCount);
    return tagCount;
}

void VendorTags::getAllTags(const vendor_tag_ops_t* v, uint32_t* tag_array) {
    UNUSED(v);
    CheckAndLogError(tag_array == nullptr, VOID_VALUE, "@%s: tag_array is nullptr", __func__);

    for (int i = 0; i < INTEL_VENDOR_SECTION_COUNT; i++) {
        uint32_t tag = vendor_metadata_section_bounds[i][0];
        for (; tag < vendor_metadata_section_bounds[i][1]; tag++) {
            *tag_array++ = tag;
        }
    }
}

const char* VendorTags::getSectionName(const vendor_tag_ops_t* v, uint32_t tag) {
    UNUSED(v);

    uint32_t tag_section = tag >> 16;
    if (tag_section >= INTEL_VENDOR_CAMERA_SECTION &&
        tag_section < INTEL_VENDOR_CAMERA_SECTION_END) {
        LOG2("@%s, tag: 0x%x, section name: %s", __func__, tag,
             vendor_metadata_section_names[tag_section - INTEL_VENDOR_CAMERA_SECTION]);
        return vendor_metadata_section_names[tag_section - INTEL_VENDOR_CAMERA_SECTION];
    }

    return nullptr;
}

const char* VendorTags::getTagName(const vendor_tag_ops_t* v, uint32_t tag) {
    UNUSED(v);
    uint32_t tag_section = tag >> 16;
    uint32_t tag_index = tag & 0xFFFF;

    if (tag_section >= INTEL_VENDOR_CAMERA_SECTION &&
        tag_section < INTEL_VENDOR_CAMERA_SECTION_END) {
        tag_section -= INTEL_VENDOR_CAMERA_SECTION;
        if (tag >= vendor_metadata_section_bounds[tag_section][0] &&
            tag < vendor_metadata_section_bounds[tag_section][1]) {
            LOG2("@%s, tag: 0x%x, name: %s", __func__, tag,
                 vendor_tag_info[tag_section][tag_index].tag_name);
            return vendor_tag_info[tag_section][tag_index].tag_name;
        }
    }

    return nullptr;
}

int VendorTags::getTagType(const vendor_tag_ops_t* v, uint32_t tag) {
    UNUSED(v);
    uint32_t tag_section = tag >> 16;
    uint32_t tag_index = tag & 0xFFFF;

    if (tag_section >= INTEL_VENDOR_CAMERA_SECTION &&
        tag_section < INTEL_VENDOR_CAMERA_SECTION_END) {
        tag_section -= INTEL_VENDOR_CAMERA_SECTION;
        if (tag >= vendor_metadata_section_bounds[tag_section][0] &&
            tag < vendor_metadata_section_bounds[tag_section][1]) {
            LOG2("@%s, tag: 0x%x, type: %d", __func__, tag,
                 vendor_tag_info[tag_section][tag_index].tag_type);
            return vendor_tag_info[tag_section][tag_index].tag_type;
        }
    }

    return -1;
}

void VendorTags::getVendorTagOps(vendor_tag_ops_t* ops) {
    CheckAndLogError(ops == nullptr, VOID_VALUE, "@%s: vendor tag ops is nullptr", __func__);

    ops->get_tag_count = getTagCount;
    ops->get_all_tags = getAllTags;
    ops->get_section_name = getSectionName;
    ops->get_tag_name = getTagName;
    ops->get_tag_type = getTagType;
}

}  // namespace camera3
