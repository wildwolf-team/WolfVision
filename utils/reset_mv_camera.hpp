#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <fmt/color.h>

namespace utils {

inline bool resetMVCamera() {
  static const auto identifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "reset_mv_camera");
  static const auto identifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "reset_mv_camera");
  static const std::vector<std::string> vendor_id{"f622", "080b"};
  bool status{false};
  fmt::print("[{}] Starting mindvision camera soft reset\n", identifier_green);
  for (const auto& _id : vendor_id) {
    std::string cmd{
        "lsusb -d : | awk '{split($0, i, \":\"); split(i[1], j, \" \"); print(\"/dev/bus/usb/\"j[2]\"/\"j[4])}'"};
    std::string result{""};
    FILE* pipe = popen(cmd.insert(9, _id).c_str(), "r");
    if (!pipe) {
      fmt::print("[{}] Error, cannot open shell buffer\n", identifier_red);
    }
    try {
      char buffer[128];
      while (fgets(buffer, sizeof buffer, pipe) != NULL) {
        result += buffer;
      }
    } catch (...) {
      pclose(pipe);
      throw;
    }
    pclose(pipe);
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
    if (!result.empty()) {
      fmt::print("[{}] Performing soft reset on device: {}\n", identifier_green, result);
      int fd{open(result.c_str(), O_WRONLY)};
      if (fd < 0) {
        fmt::print("[{}] Error, fcntl cannot open device: {}\n", identifier_red, result);
      }
      int rc = ioctl(fd, USBDEVFS_RESET, 0);
      if (rc < 0) {
        fmt::print("[{}] Error, ioctl cannot reset device: {}\n", identifier_red, result);
      } else {
        close(fd);
        status = true;
        fmt::print("[{}] Reset device '{}' success\n", identifier_green, result);
      }
    }
  }
  if (!status) {
    fmt::print("[{}] Error, cannot apply soft reset\n", identifier_red);
  }
  return status;
}

}  // namespace utils
