#pragma once

#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace utils {
bool resetMVCamera() {
  bool status{false};
  const std::vector<std::string> vendor_id{"f622", "080b"};
  std::cout << "[resetMVCamera] Starting MindVision Camera soft reset\n";
  for (const auto& _id : vendor_id) {
    std::string cmd{
        "lsusb -d : | awk '{split($0, i, \":\"); split(i[1], j, \" \"); "
        "print(\"/dev/bus/usb/\"j[2]\"/\"j[4])}'"};
    std::string result{""};
    FILE* pipe = popen(cmd.insert(9, _id).c_str(), "r");
    if (!pipe) {
      std::cout << "[resetMVCamera] Error, shell buffer open failed\n";
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
      std::cout << "[resetMVCamera] Performing soft reset on device: " << result
                << std::endl;
      int fd{open(result.c_str(), O_WRONLY)};
      if (fd < 0) {
        std::cout << "[resetMVCamera] Error, fcntl cannot open device\n";
      }
      int rc = ioctl(fd, USBDEVFS_RESET, 0);
      if (rc < 0) {
        status = false;
        std::cout << "[resetMVCamera] Error, ioctl cannot reset device\n";
      } else {
        close(fd);
        status = true;
        std::cout << "[resetMVCamera] Reset device '" << result
                  << "' success\n";
      }
    }
  }
  if (!status) {
    std::cout << "[resetMVCamera] Error, cannot apply soft reset\n";
  }
  return status;
}
}  // namespace utils
