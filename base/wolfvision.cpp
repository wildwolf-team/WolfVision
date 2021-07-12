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

  cv::VideoCapture cap_ = cv::VideoCapture(0);

  basic_armor::Detector basic_armor_ = 
    basic_armor::Detector(fmt::format("{}{}", CONFIG_FILE_PATH,"/armor/basic_armor_config.xml"));

  

  while (true)
  {
    if(mv_capture_.isindustryimgInput()){
      src_img_ = mv_capture_.image();
    }
    else{
      cap_.read(src_img_);
    }
    if(!src_img_.empty())
    {
      serial_.updateReceiveInformation();
      switch (serial_.returnReceiveMode())
      {
      case uart::SUP_SHOOT:
        serial_.writeData(basic_armor_.writeBasicArmorData(src_img_, serial_.returnReceive()));
        break;
      case uart::ENERGY_AGENCY:
        break;
      case uart::SENTRY_MODE:
        serial_.writeData(basic_armor_.writeSentryArmorData(src_img_, serial_.returnReceive(), 3700));
        break;
      case uart::BASE_MODE:
        break;
      case uart::TOP_MODE:
        serial_.writeData(basic_armor_.writeTopArmorData(src_img_, serial_.returnReceive()));
        break;
      default:
        break;
      }
      basic_armor_.freeMemory();
    }
  }
  
  return 0;
}
