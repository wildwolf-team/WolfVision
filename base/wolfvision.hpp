#pragma once

#include <fmt/core.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "basic_armor.hpp"
#include "mv_video_capture.hpp"
#include "uart_serial.hpp"
namespace wolfvision {
class Connector {
 private:
  cv::Mat src_img_;
  mindvision::RM_VideoCapture mv_capture_ =
      mindvision::RM_VideoCapture(mindvision::CameraParam(
          0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));
  basic_armor::RM_ArmorDetector armor_ =
      basic_armor::RM_ArmorDetector("configs/armor/basic_armor_config.xml");
  uart::SerialPort serial_ =
      uart::SerialPort("configs/serial/uart_serial_config.xml");

 public:
  void run();
  Connector();
  ~Connector();
};
}  // namespace wolfvision
