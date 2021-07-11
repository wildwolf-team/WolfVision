#include "wolfvision.hpp"

int main() {
  auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "wolfvision");

  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier , __VERSION__);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);

  cv::Mat src_img_;

  mindvision::VideoCapture mv_capture_ = mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

  uart::SerialPort serial_ =
    uart::SerialPort(fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));

  return 0;
}
