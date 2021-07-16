#include "wolfvision.hpp"

int main() {
  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);

  cv::Mat src_img_;

  mindvision::VideoCapture mv_capture_ = mindvision::VideoCapture(
                                      mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

  uart::SerialPort serial_ =
    uart::SerialPort(fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));

  cv::VideoCapture cap_ = cv::VideoCapture("/home/sms/VIDEO/camera_MaxBuff13.avi");

  basic_armor::Detector basic_armor_ = basic_armor::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));

  basic_buff::Detector basic_buff_ = basic_buff::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));

  basic_pnp::PnP pnp_   = basic_pnp::PnP(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_554.xml"),
    fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  onnx_inferring::model model_ = onnx_inferring::model(
      fmt::format("{}{}", SOURCE_PATH ,"/module/ml/mnist-8.onnx"));

  basic_roi::RoI save_roi;
  fps::FPS       global_fps_;

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
                              basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
        }

        serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.
                                returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        break;
      case uart::ENERGY_AGENCY:
        serial_.writeData(basic_buff_.runTask(src_img_, serial_.returnReceive()));
        break;
      case uart::SENTRY_MODE:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                              basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                          pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        } else {
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                         pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnLostCnt() > 0 ? 1 : 0, 0);
        }
        break;
      case uart::BASE_MODE:
        break;
      case uart::TOP_MODE:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                          basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(pnp_.returnYawAngle(),
                                        pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
        } else {
          serial_.updataWriteData(-pnp_.returnYawAngle(),
                                       pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnLostCnt() > 0 ? 1 : 0, 0);
        }
      case uart::PLANE_MODE:
        break;
      case uart::OCR_SENTRYSELF_MODE:
        if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
          if (basic_armor_.returnFinalArmorDistinguish(0) == 1) {  // if the firt type of the amror_list is large
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                                       basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
            serial_.updataWriteData(pnp_.returnYawAngle(),
                                       pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
          } else {  // another type
            int info_number;
            for (int i = 0; i < basic_armor_.returnArmorNum(); i++) {
              info_number = model_.inferring(save_roi.cutRoIRotatedRect(basic_armor_.ReturnFrame(),
                                                                            basic_armor_.returnFinalArmorRotatedRect(i)));
              if (info_number == 2) {
                serial_.updataWriteData(pnp_.returnYawAngle(),
                                                pnp_.returnPitchAngle(), pnp_.returnDepth(), 0, 0);
                break;
              } else {
                pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                                       basic_armor_.returnFinalArmorDistinguish(i + 1), basic_armor_.returnFinalArmorRotatedRect(i + 1));
                serial_.updataWriteData(pnp_.returnYawAngle(),
                                                pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
                break;
              }
            }
            if (model_.param_ocr.switch_number == 1) {
              cv::imshow("Dafult", save_roi.cutRoIRotatedRect(basic_armor_.ReturnFrame(),
                                                                      basic_armor_.returnFinalArmorRotatedRect(0)));
              cv::putText(basic_armor_.ReturnFrame(), std::to_string(info_number), cv::Point(100, 100), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
          }
        }
        // serial_.updataWriteData(pnp_.returnYawAngle(),
        //                         pnp_.returnPitchAngle(),
        //                         pnp_.returnDepth(),
        //                         basic_armor_.returnArmorNum(),
        //                         0);
        cv::putText(basic_armor_.ReturnFrame(), to_string(basic_armor_.returnArmorNum()), cv::Point(100, 200), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        cv::putText(basic_armor_.ReturnFrame(), to_string(serial_.returnReceiceColor()), cv::Point(100, 150), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        cv::imshow("frame", basic_armor_.ReturnFrame());
        break;
      default:
        break;
      }
      mv_capture_.cameraReleasebuff();
      basic_armor_.freeMemory();
      if (cv::waitKey(1) == 'q') {
        break;
      }
      global_fps_.calculateFPSGlobal();
    }
  }

  return 0;
}
