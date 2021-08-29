/**
 * @file mv_video_capture.hpp
 * @author RCX
 * @brief 相机读取类
 * @date 2021-08-29
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <CameraApi.h>

namespace mindvision {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "mv_video_capture");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "mv_video_capture");

enum EXPOSURETIME {
  // 相机曝光时间
  EXPOSURE_5000 = 5000,
  EXPOSURE_2500 = 2500,
  EXPOSURE_1200 = 1200,
  EXPOSURE_800 = 800,
  EXPOSURE_600 = 600,
  EXPOSURE_400 = 400,
};

enum RESOLUTION {
  // 相机分辨率
  RESOLUTION_1280_X_1024,
  RESOLUTION_1280_X_800,
  RESOLUTION_640_X_480,
};

struct Camera_Resolution {
  int cols;
  int rows;
  // 设置相机分辨率
  explicit Camera_Resolution(const mindvision::RESOLUTION _resolution) {
    switch (_resolution) {
      case mindvision::RESOLUTION::RESOLUTION_1280_X_1024:
        cols = 1280;
        rows = 1024;
        break;
      case mindvision::RESOLUTION::RESOLUTION_1280_X_800:
        cols = 1280;
        rows = 800;
        break;
      case mindvision::RESOLUTION::RESOLUTION_640_X_480:
        cols = 640;
        rows = 480;
        break;
      default:
        cols = 1280;
        rows = 800;
        break;
    }
  }
};

struct CameraParam {
  int camera_mode;
  int camera_exposuretime;

  mindvision::Camera_Resolution resolution;

  CameraParam(const int                      _camera_mode,
              const mindvision::RESOLUTION   _resolution,
              const mindvision::EXPOSURETIME _camera_exposuretime)
    : camera_mode(_camera_mode),
      camera_exposuretime(_camera_exposuretime),
      resolution(_resolution) {}
};

class VideoCapture {
 public:
  VideoCapture() = default;
  explicit VideoCapture(const mindvision::CameraParam &_camera_param);

  ~VideoCapture();
  /**
   * @brief 判断工业相机是否在线
   *
   * @return true   检测到工业相机
   * @return false  没检测到工业相机
   */
  bool isindustryimgInput();
  /**
   * @brief 清空相机内存（每次读取相机后进行清空）
   * 
   */
  void cameraReleasebuff();
  /**
   * @brief 设置相机参数
   * 
   * @param _CAMERA_RESOLUTION_COLS  设置相机宽度  
   * @param _CAMERA_RESOLUTION_ROWS  设置相机高度
   * @param _CAMERA_EXPOSURETIME     设置相机曝光
   * @return int 
   */
  int cameraInit(const int _CAMERA_RESOLUTION_COLS,
                 const int _CAMERA_RESOLUTION_ROWS,
                 const int _CAMERA_EXPOSURETIME);
  /**
   * @brief 返回相机读取图片
   * 
   * @return cv::Mat 
   */
  inline cv::Mat image() const { return cv::cvarrToMat(iplImage, true); }

 private:
  unsigned char* g_pRgbBuffer;

  int  iCameraCounts  = 1;
  int  iStatus        = -1;
  int  hCamera;
  int  channel        = 3;
  bool iscamera0_open = false;

  tSdkCameraDevInfo   tCameraEnumList;
  tSdkCameraCapbility tCapability;
  tSdkFrameHead       sFrameInfo;
  tSdkImageResolution pImageResolution;
  BYTE*               pbyBuffer;
  BOOL                AEstate  = FALSE;
  IplImage*           iplImage = nullptr;
};

}  // namespace mindvision
