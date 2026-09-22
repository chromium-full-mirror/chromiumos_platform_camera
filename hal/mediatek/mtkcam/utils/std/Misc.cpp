/*
 * Copyright (C) 2019 MediaTek Inc.
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

#define LOG_TAG "MtkCam/Utils"
//
#include "MyUtils.h"
//
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <property_service/property.h>
#include <property_service/property_lib.h>

/******************************************************************************
 *
 ******************************************************************************/

namespace NSCam {
namespace Utils {
namespace {

class ScopedFd {
 public:
  explicit ScopedFd(int fd = -1) : fd_(fd) {}
  ~ScopedFd() { reset(); }
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ScopedFd(ScopedFd&& other) noexcept : fd_(other.release()) {}
  ScopedFd& operator=(ScopedFd&& other) noexcept {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  int get() const { return fd_; }
  bool valid() const { return fd_ >= 0; }

  int release() {
    int fd = fd_;
    fd_ = -1;
    return fd;
  }

  void reset(int fd = -1) {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }

 private:
  int fd_;
};

}  // namespace

/******************************************************************************
 *
 ******************************************************************************/

/******************************************************************************
 * @brief dump call stack
 ******************************************************************************/
void dumpCallStack(char const* prefix) {}

/******************************************************************************
 *
 ******************************************************************************/
static bool do_mkdir(char const* const path, unsigned int const mode) {
  struct stat st;
  //
  if (0 != ::lstat(path, &st)) {
    //  Directory does not exist.
    if (0 != ::mkdir(path, mode) && EEXIST != errno) {
      MY_LOGE("fail to mkdir [%s]: %d[%s]", path, errno, ::strerror(errno));
      return false;
    }
    if (0 != ::lstat(path, &st) || !S_ISDIR(st.st_mode)) {
      MY_LOGE("!S_ISDIR");
      return false;
    }
  } else if (!S_ISDIR(st.st_mode)) {
    MY_LOGE("!S_ISDIR");
    return false;
  }
  //
  return true;
}

/******************************************************************************
 * @brief make all directories in path.
 *
 * @details
 * @note
 *
 * @param[in] path: a specified path to create.
 * @param[in] mode: the argument specifies the permissions to use, like 0777
 *                 (the same to that in mkdir).
 *
 * @return
 * -    true indicates success
 * -    false indicates failure
 *****************************************************************************/
bool makePath(char const* const path, unsigned int const mode) {
  bool ret = true;
  char* copypath = strdup(path);
  char* pp = copypath;
  char* sp;
  while (ret && 0 != (sp = strchr(pp, '/'))) {
    if (sp != pp) {
      *sp = '\0';
      ret = do_mkdir(copypath, mode);
      *sp = '/';
    }
    pp = sp + 1;
  }
  if (ret) {
    ret = do_mkdir(path, mode);
  }
  free(copypath);
  return ret;
}

bool saveBufToFileAt(int dir_fd,
                     char const* const fname,
                     unsigned char* const buf,
                     unsigned int const size) {
  if (dir_fd < 0 || fname == nullptr || fname[0] == '\0' ||
      ::strchr(fname, '/') != nullptr || ::strcmp(fname, ".") == 0 ||
      ::strcmp(fname, "..") == 0) {
    MY_LOGE("invalid arguments for saveBufToFileAt");
    return false;
  }
  if (size > 0 && buf == nullptr) {
    MY_LOGE("null buffer with non-zero size for [%s]", fname);
    return false;
  }

  struct stat dir_st;
  if (::fstat(dir_fd, &dir_st) != 0 || !S_ISDIR(dir_st.st_mode)) {
    MY_LOGE("invalid parent directory fd for [%s]", fname);
    return false;
  }

  if (::unlinkat(dir_fd, fname, 0) != 0 && errno != ENOENT) {
    MY_LOGE("failed to unlink existing [%s]: %s", fname, ::strerror(errno));
    return false;
  }

  constexpr mode_t kFileMode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  MY_LOGD("opening file [%s]", fname);
  ScopedFd fd(::openat(dir_fd, fname,
                       O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                       kFileMode));
  if (!fd.valid()) {
    MY_LOGE("failed to create file [%s]: %s", fname, ::strerror(errno));
    return false;
  }
  if (::fchmod(fd.get(), kFileMode) != 0) {
    MY_LOGE("failed to set mode on [%s]: %s", fname, ::strerror(errno));
    fd.reset();
    ::unlinkat(dir_fd, fname, 0);
    return false;
  }

  uint32_t written = 0;
  int cnt = 0;
  MY_LOGD("writing %d bytes to file [%s]", size, fname);
  while (written < size) {
    ssize_t nw = ::write(fd.get(), buf + written, size - written);
    if (nw < 0) {
      if (errno == EINTR) {
        continue;
      }
      MY_LOGE("failed to write to file [%s]: %s", fname, ::strerror(errno));
      fd.reset();
      ::unlinkat(dir_fd, fname, 0);
      return false;
    }
    written += static_cast<uint32_t>(nw);
    cnt++;
  }
  MY_LOGD("done writing %d bytes to file [%s] in %d passes", size, fname, cnt);
  int raw_fd = fd.release();
  if (::close(raw_fd) != 0) {
    MY_LOGE("failed to close file [%s]: %s", fname, ::strerror(errno));
    ::unlinkat(dir_fd, fname, 0);
    return false;
  }
  return true;
}

/******************************************************************************
 * @brief save the buffer to the file
 *
 * @details
 *
 * @note
 *
 * @param[in] fname: The file name
 * @param[in] buf: The buffer to save
 * @param[in] size: The size in bytes to save
 *
 * @return
 * -   true indicates success
 * -   false indicates failure
 ******************************************************************************/
bool saveBufToFile(char const* const fname,
                   unsigned char* const buf,
                   unsigned int const size) {
  if (fname == nullptr || fname[0] == '\0') {
    MY_LOGE("empty file path");
    return false;
  }
  size_t path_len = ::strlen(fname);
  if (fname[path_len - 1] == '/') {
    MY_LOGE("file path [%s] ends with slash", fname);
    return false;
  }

  std::vector<std::string> segments;
  const char* p = fname;
  while (*p != '\0') {
    while (*p == '/') {
      ++p;
    }
    if (*p == '\0') {
      break;
    }
    const char* end = ::strchr(p, '/');
    std::string seg =
        (end == nullptr) ? std::string(p) : std::string(p, end - p);
    if (seg == "..") {
      MY_LOGE("parent traversal rejected in [%s]", fname);
      return false;
    }
    if (seg != ".") {
      segments.push_back(std::move(seg));
    }
    if (end == nullptr) {
      break;
    }
    p = end;
  }

  if (segments.empty()) {
    MY_LOGE("no leaf filename in [%s]", fname);
    return false;
  }

  ScopedFd dir_fd(::open(fname[0] == '/' ? "/" : ".",
                         O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  if (!dir_fd.valid()) {
    MY_LOGE("failed to open base directory for [%s]: %s", fname,
            ::strerror(errno));
    return false;
  }

  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    ScopedFd next_fd(::openat(dir_fd.get(), segments[i].c_str(),
                              O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
    if (!next_fd.valid()) {
      MY_LOGE("failed to traverse segment [%s] in [%s]: %s",
              segments[i].c_str(), fname, ::strerror(errno));
      return false;
    }
    dir_fd = std::move(next_fd);
  }

  return saveBufToFileAt(dir_fd.get(), segments.back().c_str(), buf, size);
}

/******************************************************************************
 * @brief load the file to the buffer
 *
 * @details
 *
 * @note
 *
 * @param[in] fname: The file name
 * @param[out] buf: The output buffer
 * @param[in] capacity: The buffer capacity in bytes
 *
 * @return
 * -   The loaded size in bytes.
 ******************************************************************************/
unsigned int loadFileToBuf(char const* const fname,
                           unsigned char* const buf,
                           unsigned int const capacity) {
  int cnt = 0;
  uint32_t readCnt = 0;
  unsigned int size = capacity;

  if (fname == nullptr || buf == nullptr) {
    return 0;
  }

  MY_LOGD("opening file [%s]", fname);
  ScopedFd fd(::open(fname, O_RDONLY | O_NOFOLLOW | O_CLOEXEC));
  if (!fd.valid()) {
    MY_LOGE("failed to open file [%s]: %s", fname, strerror(errno));
    return readCnt;
  }
  //
  if (size == 0) {
    off_t end_pos = ::lseek(fd.get(), 0, SEEK_END);
    if (end_pos < 0 || ::lseek(fd.get(), 0, SEEK_SET) < 0) {
      return 0;
    }
    size = static_cast<unsigned int>(end_pos);
  }
  //
  MY_LOGD("read %d bytes from file [%s]", size, fname);
  while (readCnt < size) {
    ssize_t nr = ::read(fd.get(), buf + readCnt, size - readCnt);
    if (nr < 0) {
      if (errno == EINTR) {
        continue;
      }
      MY_LOGE("failed to read from file [%s]: %s", fname, strerror(errno));
      break;
    }
    if (nr == 0) {
      break;
    }
    readCnt += static_cast<uint32_t>(nr);
    cnt++;
  }
  MY_LOGD("done reading %d bytes to file [%s] in %d passes", size, fname, cnt);

  return readCnt;
}

/******************************************************************************
 *
 ******************************************************************************/
};  // namespace Utils
};  // namespace NSCam

static int32_t determinePersistLogLevel() {
  int32_t level =
      ::property_get_int32("persist.vendor.mtk.camera.log_level", -1);
  MY_LOGD("###### get camera log property =%d", level);
  if (-1 == level) {
    level = 3;  // MTKCAM_LOG_LEVEL_DEFAULT;
  }
  return level;
}

__BEGIN_DECLS
static int32_t gLogLevel = determinePersistLogLevel();
int mtkcam_testLog(char const* /*tag*/, int prio) {
  switch (prio) {
    case 'V':
      return (gLogLevel >= 4);
    case 'D':
      return (gLogLevel >= 3);
    case 'I':
      return (gLogLevel >= 2);
    case 'W':
      return (gLogLevel >= 1);
    case 'E':
      return (1);
    default:
      break;
  }
  return 0;
}

void setLogLevelToEngLoad(bool is_camera_on_off_timing,
                          bool set_log_level_to_eng,
                          int logCount) {
#if (((1 == MTKCAM_USER_DEBUG_LOAD) && (0 == MTKCAM_USER_DBG_LOG_OFF)) || \
     (1 == MTKCAM_ENG_LOAD))
  char value[16];
  if (is_camera_on_off_timing) {
    int32_t mtk_internal =
        ::property_get_int32("ro.vendor.mtklog_internal", -1);
    if (set_log_level_to_eng) {
      if (logCount == -1) {
        snprintf(value, sizeof(value), "%d", MTKCAM_ANDROID_LOG_MUCH_COUNT);
      } else {
        snprintf(value, sizeof(value), "%d", logCount);
      }
      if (mtk_internal == 1) {
        property_set("vendor.logmuch.value", value);
        MY_LOGI("###### set log level to %s", value);
      } else {
        MY_LOGI(
            "[enter camera]not mtk_internal_project (%d), no need to change "
            "log level",
            mtk_internal);
      }
    } else {
      if (mtk_internal == 1) {
        property_set("vendor.logmuch.value", "0");
        MY_LOGI("###### set log level to default");
      } else {
        MY_LOGI(
            "[exit camera]not mtk_internal_project (%d), no need to change log "
            "level",
            mtk_internal);
      }
    }
  }
#endif
}

__END_DECLS
