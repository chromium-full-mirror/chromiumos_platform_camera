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

#define LOG_TAG "EEPROM_UPDATE"

#include <errno.h>
#include <fcntl.h>
#include <property_lib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>

#include <mtkcam/utils/std/common.h>
#include <mtkcam/utils/std/Log.h>
#include <mtkcam/utils/std/Misc.h>
#include <mtkcam/utils/TuningUtils/TuningPlatformInfo.h>

using NSCam::TuningUtils::PlatformInfo;
using NSCam::TuningUtils::TuningPlatformInfo;

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

// Opens /run/camera/EEPROM step-by-step without following symlinks and verifies
// that the resulting directory inode is owned by root without group/other write
// permissions.
ScopedFd OpenVerifiedEepromDir() {
  constexpr int kDirFlags = O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC;

  ScopedFd root_fd(::open("/", kDirFlags));
  if (!root_fd.valid()) {
    CAM_LOGE("failed to open /: %s", ::strerror(errno));
    return ScopedFd();
  }

  ScopedFd run_fd(::openat(root_fd.get(), "run", kDirFlags));
  if (!run_fd.valid()) {
    CAM_LOGE("failed to open /run: %s", ::strerror(errno));
    return ScopedFd();
  }

  struct stat st;
  if (::fstat(run_fd.get(), &st) != 0 || !S_ISDIR(st.st_mode) ||
      st.st_uid != 0 || (st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
    CAM_LOGE("/run failed ownership or mode verification");
    return ScopedFd();
  }

  ScopedFd camera_fd(::openat(run_fd.get(), "camera", kDirFlags));
  if (!camera_fd.valid()) {
    CAM_LOGE("failed to open /run/camera: %s", ::strerror(errno));
    return ScopedFd();
  }

  if (::mkdirat(camera_fd.get(), "EEPROM", 0755) != 0) {
    if (errno != EEXIST) {
      CAM_LOGE("mkdirat EEPROM failed: %s", ::strerror(errno));
      return ScopedFd();
    }
    // Clear any pre-existing symlink, regular file, or empty directory before
    // retrying mkdirat; non-empty root-owned directories remain and are
    // verified via fstat below.
    if (::unlinkat(camera_fd.get(), "EEPROM", 0) == 0 ||
        ::unlinkat(camera_fd.get(), "EEPROM", AT_REMOVEDIR) == 0) {
      if (::mkdirat(camera_fd.get(), "EEPROM", 0755) != 0) {
        CAM_LOGE("mkdirat EEPROM retry failed: %s", ::strerror(errno));
        return ScopedFd();
      }
    }
  }

  ScopedFd eeprom_fd(::openat(camera_fd.get(), "EEPROM", kDirFlags));
  if (!eeprom_fd.valid()) {
    CAM_LOGE("openat EEPROM failed: %s", ::strerror(errno));
    return ScopedFd();
  }

  if (::fstat(eeprom_fd.get(), &st) != 0) {
    CAM_LOGE("fstat EEPROM failed: %s", ::strerror(errno));
    return ScopedFd();
  }
  if (!S_ISDIR(st.st_mode) || st.st_uid != 0 ||
      (st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
    CAM_LOGE("untrusted EEPROM directory uid=%u mode=%o", st.st_uid,
             st.st_mode);
    return ScopedFd();
  }
  if (st.st_gid != 0 && ::fchown(eeprom_fd.get(), 0, 0) != 0) {
    CAM_LOGE("fchown EEPROM failed: %s", ::strerror(errno));
    return ScopedFd();
  }
  if ((st.st_mode & 07777) != 0755 && ::fchmod(eeprom_fd.get(), 0755) != 0) {
    CAM_LOGE("fchmod EEPROM failed: %s", ::strerror(errno));
    return ScopedFd();
  }
  if (::fstat(eeprom_fd.get(), &st) != 0 || !S_ISDIR(st.st_mode) ||
      st.st_uid != 0 || st.st_gid != 0 ||
      (st.st_mode & (S_IWGRP | S_IWOTH)) != 0) {
    CAM_LOGE("EEPROM directory post-check failed");
    return ScopedFd();
  }

  return eeprom_fd;
}

bool IsValidSysfsNvmemPath(const std::string& path) {
  constexpr char kPrefix[] = "/sys/bus/nvmem/devices/";
  constexpr char kSuffix[] = "/nvmem";
  constexpr size_t kPrefixLen = sizeof(kPrefix) - 1;
  constexpr size_t kSuffixLen = sizeof(kSuffix) - 1;

  if (path.size() <= kPrefixLen + kSuffixLen ||
      path.compare(0, kPrefixLen, kPrefix) != 0 ||
      path.compare(path.size() - kSuffixLen, kSuffixLen, kSuffix) != 0) {
    return false;
  }

  std::string dev_name =
      path.substr(kPrefixLen, path.size() - kPrefixLen - kSuffixLen);
  return !dev_name.empty() && dev_name != "." && dev_name != ".." &&
         dev_name.find('/') == std::string::npos;
}

}  // namespace

int main(int argc, char* const argv[]) {
  ScopedFd eeprom_fd = OpenVerifiedEepromDir();
  if (!eeprom_fd.valid()) {
    return -1;
  }

  PlatformInfo pInfo;
  TuningPlatformInfo pPlatformInfo;
  if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0') {
    pPlatformInfo.getTuningInfo(&pInfo, argv[1]);
  } else {
    pPlatformInfo.getTuningInfo(&pInfo);
  }

  unsigned char eeprom_buffer[4096] = {};
  unsigned int file_size = 0;
  if (IsValidSysfsNvmemPath(pInfo.wf_sensor.eeprom_path)) {
    file_size =
        NSCam::Utils::loadFileToBuf(pInfo.wf_sensor.eeprom_path.c_str(),
                                    eeprom_buffer, sizeof(eeprom_buffer));
  } else {
    CAM_LOGE("rejected untrusted wf_sensor eeprom_path [%s]",
             pInfo.wf_sensor.eeprom_path.c_str());
  }
  if (!NSCam::Utils::saveBufToFileAt(eeprom_fd.get(), "main_sensor_eeprom",
                                     eeprom_buffer, file_size)) {
    return -1;
  }

  ::memset(eeprom_buffer, 0, sizeof(eeprom_buffer));
  file_size = 0;
  if (IsValidSysfsNvmemPath(pInfo.uf_sensor.eeprom_path)) {
    file_size =
        NSCam::Utils::loadFileToBuf(pInfo.uf_sensor.eeprom_path.c_str(),
                                    eeprom_buffer, sizeof(eeprom_buffer));
  } else {
    CAM_LOGE("rejected untrusted uf_sensor eeprom_path [%s]",
             pInfo.uf_sensor.eeprom_path.c_str());
  }
  if (!NSCam::Utils::saveBufToFileAt(eeprom_fd.get(), "sub_sensor_eeprom",
                                     eeprom_buffer, file_size)) {
    return -1;
  }

  return 0;
}
