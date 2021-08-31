/**
 * @file wolfvision.cpp
 * @author XX (2796393320@qq.com) 
 *         WCJ (1767851382@qq.com)
 *         SMS (2436210442@qq.com)
 *         SHL (2694359979@qq.com)
 * @brief 主函数
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
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

  RecordMode::Record record_ = RecordMode::Record(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/record/recordpath_save.yaml"),
        fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/record.avi"),
        cv::Size(1280, 800));  // 记得修改分辨率
  cv::VideoWriter vw_src;
  cv::FileStorage re_config_get(record_.video_save_path_, cv::FileStorage::READ);
  re_config_get["_PATH"] >> record_.path_;
  vw_src.open(CONFIG_FILE_PATH + '/record/' + record_.path_+ '.avi', cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 66, cv::Size(1280, 800), true);
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
      // 基础自瞄模式
      case uart::SUP_SHOOT:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
        }
        serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        break;
      // 能量机关
      case uart::ENERGY_AGENCY:
        serial_.writeData(basic_buff_.runTask(src_img_, serial_.returnReceive()));
        break;
      // 击打哨兵模式
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
      // 反小陀螺模式（暂未完善）
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
      // 录制视频
      case uart::RECORD_MODE:
        vw_src << src_img_;
        break;
      // 无人机模式（空缺）
      case uart::PLANE_MODE:
        break;
      // 哨兵模式（添加数字识别便于区分工程和其他车辆）
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
      // 雷达模式
      case uart::RADAR_MODE:
        break;
      // 默认进入基础自瞄
      default:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
        }
        serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        break;
      }
    }
    if (record_.last_mode_ != uart::RECORD_MODE && serial_.returnReceiveMode() == uart::RECORD_MODE) {
      vw_src.release();
    }
    record_.last_mode_ = serial_.returnReceiveMode();
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
    // 看门狗放置相机掉线
    global_fps_.calculateFPSGlobal();
    if (global_fps_.returnFps() > 500) {
      mv_capture_->~VideoCapture();
      static int counter_for_dev {100};
      static int counter_for_new {30};
      while (!utils::resetMVCamera()) {
        if (!--counter_for_dev) {
          int i [[maybe_unused]] = std::system("echo 1 | sudo -S reboot");
        }
        usleep(100);
      }
      usleep(100);
      mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(
          0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));
      if (!--counter_for_new) {
        int i [[maybe_unused]] = std::system("echo 1 | sudo -S reboot");
      }
    }
  }
  return 0;
}
