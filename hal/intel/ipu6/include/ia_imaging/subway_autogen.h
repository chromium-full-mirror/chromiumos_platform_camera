/*
 * Copyright (C) 2017-2020 Intel Corporation.
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
/****************************************
 * Description: packed subway api
 ****************************************/
#ifndef SUBWAY_SUBWAY_H_
#define SUBWAY_SUBWAY_H_
#ifndef __cplusplus
#include <stdbool.h>  // bool type in C
#endif

#ifdef _WIN32
struct ID3D11Resource;
struct ID3D12Resource;
struct ID3D12Heap;
#endif

// AUTOGEN PLACEHOLDER
enum iaic_features {
  tnr7us_l0,
};

enum iaic_gfx_type {
  iaic_gfx_none,   //!< NOLINT. Default 0, not the graphic memory type
  iaic_d3d11_res,  //!< NOLINT. Represents to ID3D11Resource type
  iaic_d3d12_res,  //!< NOLINT. Represents to ID3D12Resource type
  iaic_d3d12_heap  //!< NOLINT. Represents to ID3D12Heap type
};

// NOLINTNEXTLINE(readability-identifier-naming)
enum iaic_media_format { iaic_nv12, iaic_uint16 };

// NOLINTNEXTLINE(readability-identifier-naming)
enum iaic_log_level { trace = 0, debug, info, warning, error, fatal };

enum iaic_session_status {
  iaic_uninitialized,  //!< NOLINT. The graph session isn't "initialized".
  iaic_opening,        //!< NOLINT. The graph session is on initializing.
  iaic_opened,         //!< NOLINT. The graph session is ready to run.
  iaic_closed,         //!< NOLINT. The graph session is "closed".
  iaic_critical        //!< NOLINT. The graph session encounters any error.
};

// NOLINTNEXTLINE(readability-identifier-naming)
struct iaic_memory_shape {
  unsigned long long total_bytes;  // NOLINT
  unsigned long long width;        // NOLINT
  unsigned long long height;       // NOLINT
  unsigned long long row_pitch;    // NOLINT
};

// NOLINTNEXTLINE
struct iaic_memory {
  /// Specify a port name which you need to set/get.
  /// The port name has a syntax [calculator tag:input/output name]
  const char* port_name;

  /// Specify a feature name to set/get.
  /// If not specified, matching the first one.
  const char* feature_name;

  /// Specify data pointer. If `has_gfx` is true, the data is regarded
  /// as a pointer to ID3D11Resource.
  union {
    void* p;
#ifdef _WIN32
    struct ID3D11Resource* r;
    struct ID3D12Resource* r12;
    struct ID3D12Heap* h;
#endif
  };

  /// Specify the shape of the data, size[0] represents the total bytes,
  /// size[1] represents width for 2D buffer, size[2] represents height
  /// for 2D buffer, size[3] represents row pitch in bytes.
  union {
    unsigned long long size[4];  // NOLINT
    struct iaic_memory_shape shape;
  };
  /// Reserved since only NV12 will be processed.
  enum iaic_media_format media_type;

  /// If true, the data is a handle to GPU 2D texture, otherwise it's a
  /// CPU linear buffer.
  union {
    bool has_gfx;            //!< @deprecated for backward compatibility
    enum iaic_gfx_type gfx;  //!< graphic memory type of union pointer `p`.
  };

  /// Pointer to next memory.
  struct iaic_memory* next;
};

enum iaic_dev_type { iaic_default, iaic_d3d11, iaic_d3d12 };

// NOLINTNEXTLINE
struct iaic_options {
  bool profiling;         //!< profile each calculator
  bool blocked_init;      //!< blocked until initialization finished
  bool async_mode;        //!< schedule node in async mode
  unsigned int threads;   //!< thread number to build kernels
  void* external_device;  //!< external device
  enum iaic_dev_type external_device_type;  //!< external device type
};

// NOLINTNEXTLINE
struct iaic_join_desc {
  struct iaic_join_desc* next;
  const char* src_feature_name;
  const char* src_port_name;
  const char* dst_feature_name;
  const char* dst_port_name;
};

typedef unsigned int iaic_session;  // NOLINT(modernize-use-using)

#define _F(c) iaic_get_feature_name(c)

/* clang-format off */
/**
 * @brief Get the current lib's version
 * @param [out] major: major no.
 * @param [out] minor: minor no.
 * @param [out] patch: patch no.
 */
/*
void iaic_query_version(int* major, int* minor, int* patch);
*/

/**
 * @brief Get the registered feature names.
 * @param [out] features: the list of feature names, separated by comma.
 * @param [out] length: length of the string.
 */
/*
void iaic_query_features(const char* features, size_t* length);
*/

/**
 * @brief Initialize the lib, must call before any APIs below.
 */
/*
void iaic_startup();
*/

/**
 * @brief Shutdown the lib and release all resources. Must call for each
 * `iaic_startup`.
 */
/*
void iaic_shutdown();
*/

/**
 * @brief Set the logging level.
 * @param [in] level: range from [trace, fatal)
 */
/*
void iaic_set_loglevel(iaic_log_level level);
*/

/**
 * @brief Create or reset a session.
 *
 * This function is thread-safe and should not throw any exceptions.
 * If the creation is successful, the `uid` is stored internally, otherwise
 * `uid` is undefined.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] feature: a string for the registered feature to create, create
 *   multiple features on a same uid will combine the features' graph.
 * @param [in] opt: optional, specify configuration for the session.
 */
/*
void iaic_create_session(iaic_session uid, const char* feature, iaic_options opt);
*/

/**
 * @brief Close a feature of a session
 *
 * This function is thread-safe and should not throw any exceptions.
 * This call will remove the feature and all of its resources.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] feature: a string for the registered feature to create, create
 *   multiple features on a same uid will combine the features' graph.
 */
/*
void iaic_close_session(iaic_session uid, const char* feature);
*/

/**
 * @brief Join multiple sessions.
 *
 * This function is thread-safe and should not throw any exceptions.
 * This call connects features by bridging their given ports.
 * The joined ports will be removed from I/O maps.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] desc: a description struct to tell how to connect features.
 */
/*
extern "C" void iaic_join_session(iaic_session uid, iaic_join_desc desc);
*/

/**
 * @brief Disjoin sessions.
 *
 * This function is thread-safe and should not throw any exceptions.
 * This call is the revert of iaic_join_session. Disconnect features and restore
 * I/O maps.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] desc: a description struct to tell where to disconnect features.
 */
/*
extern "C" void iaic_disjoin_session(iaic_session uid, iaic_join_desc desc);
*/

/**
 * @brief Pause the feature of a session.
 *
 * Furthur execution will skip this feature.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] feature: a string for the registered feature to create, create
 *   multiple features on a same uid will combine the features' graph.
 */
/*
void iaic_pause_feature(iaic_session uid, const char* feature);
*/

/**
 * @brief Resume the feature of a session.
 *
 * Do nothing if the feature is not created or paused.
 *
 * @param [in] uid: a unique id for different session, if a id has already been
 *   created, then it is reset.
 * @param [in] feature: a string for the registered feature to create, create
 *   multiple features on a same uid will combine the features' graph.
 */
/*
void iaic_resume_feature(iaic_session uid, const char* feature);
*/

/**
 * @brief Execute the session.
 *
 * This function is thread-safe and should not throw any exceptions.
 * If the uid is undefined, this function does nothing.
 * It assumes the session has only one input stream and at most one output stream, if
 * there're more than 1 input/output streams, use `SubwayPackSetData` for other stream data.
 * If data name is not found, no data will be bound. The graph may crash if there is no
 * input stream bound by `SubwayPackSetData`.
 *
 * @param [in] uid: the unique id used in the session creation, specify to that session.
 * @param [in] in: specify input stream data, in.name can be omitted and it bound to the
 *   first MediaSource.
 * @param [in] out: specify output stream data, out.name can be omitted and it bound to
 *   the first MediaDrain.
 */
/*
bool iaic_execute(iaic_session uid, iaic_memory in, iaic_memory out);
*/

/**
 * @brief Bind the data to session source.
 *
 * This function is thread-safe for different `uid` and not thread-safe for same `uid`.
 * This function should not throw any exceptions.
 * If the size of the data exceeds buffer boundary, it will be truncated.
 *
 * @param [in] uid: the unique id used in the session creation, specify to that session.
 * @param [in] data: typeless raw binary data.
 */
/*
void iaic_set_data(iaic_session uid, iaic_memory& data);
*/

/**
 * @brief Get the session output.
 *
 * This function is thread-safe for different `uid` and not thread-safe for same `uid`.
 * This function should not throw any exceptions.
 *
 * @param [in] uid: the unique id used in the session creation, specify to that session.
 * @param [inout] data: copy output data to data.p, user should allocate enough buffer.
 *   If data.p is nullptr, this function will populate data.size and return directly.
 *   if data.has_gfx is true, the internal texture handle is assigned to data.r.
 */
/*
void iaic_get_data(iaic_session uid, iaic_memory& data);
*/

/**
 * @brief Get the session status.
 *
 * This function is thread-safe for different `uid` and not thread-safe for same
 * `uid`. This function should not throw any exceptions.
 *
 * @param [in] uid: the unique id used in the session creation, specify to that
 *   session.
 * @param [in] feature: a string for the registered feature to create, create
 *   multiple features on a same uid will combine the features' graph.
 */
/*
iaic_session_status iaic_get_status(iaic_session uid, const char* feature) {
*/

/// @brief Get feature string based on coded enum class
/*
extern "C" char* iaic_get_feature_name(int code) { return nullptr; }
*/
/* clang-format on */
#endif  // SUBWAY_SUBWAY_H_
