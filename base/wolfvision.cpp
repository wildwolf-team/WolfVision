#include "wolfvision.hpp"

int main() {
  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);

  cv::Mat src_img_, roi_img_;

  mindvision::VideoCapture mv_capture_ = mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

  uart::SerialPort serial_ =
    uart::SerialPort(fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));

  cv::VideoCapture cap_ = cv::VideoCapture(0);

  basic_armor::Detector basic_armor_ = basic_armor::Detector(
      fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));

  basic_buff::Detector basic_buff_ = basic_buff::Detector(
      fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));

  basic_pnp::PnP pnp_ = basic_pnp::PnP(
      fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_554.xml"),
      fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  fps::FPS global_fps_;

  basic_roi::RoI roi_;
  while (true) {
    global_fps_.getTick();

    if (mv_capture_.isindustryimgInput()) {
      src_img_ = mv_capture_.image();
    } else {
      cap_.read(src_img_);
    }

    if (!src_img_.empty()) {
      serial_.updateReceiveInformation();
      switch (serial_.returnReceiveMode()) {
      case uart::SUP_SHOOT:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                        basic_armor_.returnFinalArmorDistinguish(0),
                        basic_armor_.returnFinalArmorRotatedRect(0));
        }

        serial_.updataWriteData(pnp_.returnYawAngle(),
                                pnp_.returnPitchAngle(),
                                pnp_.returnDepth(),
                                basic_armor_.returnArmorNum(),
                                0);
        break;
      case uart::ENERGY_AGENCY:
        serial_.writeData(basic_buff_.runTask(src_img_, serial_.returnReceive()));
        break;
      case uart::SENTRY_MODE:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                        basic_armor_.returnFinalArmorDistinguish(0),
                        basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  pnp_.returnDepth(),
                                  basic_armor_.returnArmorNum(),
                                  0);
        } else {
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  pnp_.returnDepth(),
                                  basic_armor_.returnLostCnt() > 0 ? 1 : 0,
                                  0);
        }
        break;
      case uart::BASE_MODE:
        break;
      case uart::TOP_MODE:
        roi_img_ = roi_.returnROIResultMat(src_img_);
        cv::imshow("roi", roi_img_);
        if (basic_armor_.runBasicArmor(roi_img_, serial_.returnReceive())) {
          basic_armor_.fixFinalArmorCenter(0, roi_.returnRectTl());
          roi_.setLastRoiRect(basic_armor_.returnFinalArmorRotatedRect(0));
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                        basic_armor_.returnFinalArmorDistinguish(0),
                        basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  pnp_.returnDepth(),
                                  basic_armor_.returnArmorNum(),
                                  0);
        } else {
          serial_.updataWriteData(-pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  pnp_.returnDepth(),
                                  basic_armor_.returnLostCnt() > 0 ? 1 : 0,
                                  0);
        }
        roi_.setLastRoiSuccess(basic_armor_.returnArmorNum());
        break;
      default:
        break;
      }
    }
    mv_capture_.cameraReleasebuff();
    basic_armor_.freeMemory();
    if (cv::waitKey(1) == 'q') {
      return 0;
    }
    global_fps_.calculateFPSGlobal();
  }

  return 0;
}
