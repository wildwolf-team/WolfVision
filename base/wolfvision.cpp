#include "wolfvision.hpp"

int main() {
  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);

  cv::Mat src_img_, roi_img_;
  mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

  uart::SerialPort serial_ = uart::SerialPort(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));

  cv::VideoCapture cap_ = cv::VideoCapture(0);

  basic_armor::Detector basic_armor_ = basic_armor::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));

  basic_buff::Detector basic_buff_ = basic_buff::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));

  basic_pnp::PnP pnp_ = basic_pnp::PnP(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_407.xml"), fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  onnx_inferring::model model_ = onnx_inferring::model(
    fmt::format("{}{}", SOURCE_PATH, "/module/ml/mnist-8.onnx"));

  Record_mode::Record record_ = Record_mode::Record(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_mode.yaml"),
                                                    fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/" + to_string(record_.Path_H) + ".avi"),
                                                    cv::Size(1280, 800));  // 记得修改分辨率

  basic_roi::RoI save_roi;
  fps::FPS       global_fps_;

  basic_roi::RoI roi_;
  while (true) {
    global_fps_.getTick();
    if (mv_capture_->isindustryimgInput()) {
      src_img_ = mv_capture_->image();
    } else {
      cap_.read(src_img_);
    }
    if (!src_img_.empty()) {
      serial_.updateReceiveInformation();

      switch (serial_.returnReceiveMode()) {
      case uart::SUP_SHOOT:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
        }
        serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        break;
      case uart::ENERGY_AGENCY:
        serial_.writeData(basic_buff_.runTask(src_img_, serial_.returnReceive()));
        break;
      case uart::SENTRY_STRIKE_MODE:
        if (basic_armor_.sentryMode(src_img_, serial_.returnReceive())) {
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

        break;
      case uart::TOP_MODE:
        roi_img_ = roi_.returnROIResultMat(src_img_);
        if (basic_armor_.runBasicArmor(roi_img_, serial_.returnReceive())) {
          basic_armor_.fixFinalArmorCenter(0, roi_.returnRectTl());
            roi_.setLastRoiRect(basic_armor_.returnFinalArmorRotatedRect(0),
            basic_armor_.returnFinalArmorDistinguish(0));
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
            basic_armor_.returnFinalArmorDistinguish(0),
            basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(pnp_.returnYawAngle(),
            pnp_.returnPitchAngle(), pnp_.returnDepth(),
            basic_armor_.returnArmorNum(), 0);
        } else {
          serial_.updataWriteData(-pnp_.returnYawAngle(),
            pnp_.returnPitchAngle(), pnp_.returnDepth(),
            basic_armor_.returnLostCnt() > 0 ? 1 : 0, 0);
        }
        roi_.setLastRoiSuccess(basic_armor_.returnArmorNum());

        break;
      case uart::RECORD_MODE:
        record_.Rmode_current = 5;
        break;
      case uart::PLANE_MODE:

        break;
      case uart::SENTINEL_AUTONOMOUS_MODE:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          if (basic_armor_.returnFinalArmorDistinguish(0) == 1) {
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
              basic_armor_.returnFinalArmorDistinguish(0),
              basic_armor_.returnFinalArmorRotatedRect(0));
            serial_.updataWriteData(pnp_.returnYawAngle(),
              pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
          } else {
            for (int i = 0; i < basic_armor_.returnArmorNum(); i++) {
              if (model_.inferring(save_roi.cutRoIRotatedRect(src_img_,
                basic_armor_.returnFinalArmorRotatedRect(i)), 0, 0, src_img_) == 2) {
                if (basic_armor_.returnArmorNum() > 1) {
                  pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                    basic_armor_.returnFinalArmorDistinguish(i + 1),
                    basic_armor_.returnFinalArmorRotatedRect(i + 1));
                  serial_.updataWriteData(pnp_.returnYawAngle(),
                    pnp_.returnPitchAngle(), pnp_.returnDepth(),
                    basic_armor_.returnArmorNum(), 0);
                  break;
                } else {
                  serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), 0, 0);
                  break;
                }
              } else {
                pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                  basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
                serial_.updataWriteData(pnp_.returnYawAngle(),
                  pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
                break;
              }
            }
          }
        }
        break;
      case uart::RADAR_MODE:
        fmt::print("RADIA_MODE");
        cv::imshow("RECORD_IMG", src_img_);
        record_.RecordIng(src_img_);
        break;
      default:
        break;
      }
    }
    if (record_.Return_switch()) {
      record_.Vision_judge(src_img_, cv::waitKey(1), serial_.returnReceiveMode());
    }
    // 非击打哨兵模式时初始化
    if (serial_.returnReceiveMode() != uart::SENTRY_STRIKE_MODE) {
      basic_armor_.initializationSentryMode();
    }
    mv_capture_->cameraReleasebuff();
    basic_armor_.freeMemory();

#ifndef RELEASE
    cv::imshow("dafule", src_img_);
    if (cv::waitKey(1) == 'q') {
      return 0;
    }

#else
    usleep(1);
#endif
    global_fps_.calculateFPSGlobal();
    if (global_fps_.returnFps() > 500) {
      mv_capture_->~VideoCapture();
      while (!utils::resetMVCamera()) {
        usleep(100);
      }
      usleep(100);
      mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(
          0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

      static int counter { 30 };
      if (!--counter) {
        int i [[maybe_unused]] = std::system("echo 1 | sudo -S reboot");
      }
    }
  }
  return 0;
}
