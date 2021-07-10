#pragma once

#include <fmt/core.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <CameraApi.h>

namespace mindvision {

enum EXPOSURETIME {
  EXPOSURE_5000 = 5000,
  EXPOSURE_2500 = 2500,
  EXPOSURE_1200 = 1200,
  EXPOSURE_800  = 800,
  EXPOSURE_600  = 600,
  EXPOSURE_400  = 400,
};

enum RESOLUTION {
  RESOLUTION_1280_X_1024,
  RESOLUTION_1280_X_800,
  RESOLUTION_640_X_480,
};

struct Camera_Resolution {
  int cols;
  int rows;

  explicit Camera_Resolution(const mindvision::RESOLUTION _resolution) {
    switch (_resolution) {
      case mindvision::RESOLUTION::RESOLUTION_1280_X_1024: {
        cols = 1280;
        rows = 1024;
        break;
      }
      case mindvision::RESOLUTION::RESOLUTION_1280_X_800: {
        cols = 1280;
        rows = 800;
        break;
      }
      case mindvision::RESOLUTION::RESOLUTION_640_X_480: {
        cols = 640;
        rows = 480;
        break;
      }
      default: {
        cols = 1280;
        rows = 800;
        break;
      }
    }
  }
};

struct CameraParam {
  int camera_mode;
  int camera_exposuretime;

  mindvision::Camera_Resolution resolution;

  CameraParam(const int _camera_mode, const mindvision::RESOLUTION _resolution, const mindvision::EXPOSURETIME _camera_exposuretime)
    : camera_mode(_camera_mode), camera_exposuretime(_camera_exposuretime), resolution(_resolution){}
};

class VideoCapture {
 private:
  unsigned char *g_pRgbBuffer;

  int  iCameraCounts  = 1;
  int  iStatus        = -1;
  int  hCamera;
  int  channel        = 3;
  bool iscamera0_open = false;

  tSdkCameraDevInfo   tCameraEnumList;
  tSdkCameraCapbility tCapability;
  tSdkFrameHead       sFrameInfo;
  tSdkImageResolution pImageResolution;
  BYTE                *pbyBuffer;
  BOOL                AEstate   = FALSE;
  IplImage            *iplImage = nullptr;

 public:
  explicit VideoCapture(const mindvision::CameraParam &_camera_param);

  ~VideoCapture();

  bool isindustryimgInput();
  void cameraReleasebuff();

  int cameraInit(const int _CAMERA_RESOLUTION_COLS, const int _CAMERA_RESOLUTION_ROWS, const int _CAMERA_EXPOSURETIME);

  inline cv::Mat image() const { return cv::cvarrToMat(this->iplImage, true); }
};

}  // namespace mindvision
