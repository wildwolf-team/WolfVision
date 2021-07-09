#pragma once

#include <CameraApi.h>

#include <iostream>
/*---工业相机中使用到 opencv2.0 的 IplImage 需要包含此头文件 ---*/
#include <opencv2/imgproc/imgproc_c.h>
/*---工业相机中使用到 opencv2.0 的 IplImage 需要包含此头文件 ---*/
#include <opencv2/opencv.hpp>
namespace mindvision {
enum EXPOSURETIME {
  /**
   * @brief 预设值曝光时间
   *
   */
  EXPOSURE_5000 = 5000,
  EXPOSURE_2500 = 2500,
  EXPOSURE_1200 = 1200,
  EXPOSURE_800 = 800,
  EXPOSURE_600 = 600,
  EXPOSURE_400 = 400,
};

enum RESOLUTION {
  /**
   * @brief 预设置分辨率
   * @details 由于相机的硬件属性，预设分辨率需要满足
   *          宽为 16 的倍数，高为 2 的倍数
   */
  RESOLUTION_1280_X_1024,  // 默认 1280 x 1024
  RESOLUTION_1280_X_800,   // 1280 x 800
  RESOLUTION_640_X_480,    // 640 x 480
};

struct Camera_Resolution {
  int cols;
  int rows;

  /**
   * @param[in] _cols   相机分辨率 宽
   * @param[in] _rows   相机分辨率 高
   */
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

  /**
   * @brief Construct a new User Param object
   *
   * @param[in] _camera_mode              选择相机模式
   * @param[in] _resolution               选择相机分辨率
   * @param[in] _camera_exposuretime      设置相机曝光时间
   */
  CameraParam(const int _camera_mode, const mindvision::RESOLUTION _resolution,
              const mindvision::EXPOSURETIME _camera_exposuretime)
      : camera_mode(_camera_mode),
        resolution(_resolution),
        camera_exposuretime(_camera_exposuretime) {}
};

class RM_VideoCapture {
 private:
  unsigned char *g_pRgbBuffer;  // 处理后数据缓存区

  int iCameraCounts = 1;
  int iStatus = -1;
  tSdkCameraDevInfo tCameraEnumList;
  int hCamera;
  tSdkCameraCapbility tCapability;  // 设备描述信息
  tSdkFrameHead sFrameInfo;
  BYTE *pbyBuffer;
  IplImage *iplImage = nullptr;
  int channel = 3;
  BOOL AEstate = FALSE;
  tSdkImageResolution pImageResolution;  // 相机分辨率信息

  bool iscamera0_open = false;

 public:
  /**
   * @brief Construct a new RM_VideoCapture::RM_VideoCapture object
   * @param[in] _camera_param 相机参数结构体
   */
  explicit RM_VideoCapture(const mindvision::CameraParam &_camera_param);
  RM_VideoCapture();
  /**
   * @brief Destroy the rm videocapture::rm videocapture object
   */
  ~RM_VideoCapture();

  /**
   * @brief 工业相机的图像转到指针中 再通过指针转换变为 Mat
   * @return true 成功启用工业相机
   * @return false 不能启用工业相机
   */
  bool isindustryimgInput();

  /**
   * @brief 释放相机缓存数据
   */
  void cameraReleasebuff();

  int cameraInit(const int _CAMERA_RESOLUTION_COLS,
                 const int _CAMERA_RESOLUTION_ROWS,
                 const int _CAMERA_EXPOSURETIME);

  inline cv::Mat image() const { return cv::cvarrToMat(this->iplImage, true); }
};
}  // namespace mindvision
