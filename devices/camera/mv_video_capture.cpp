#include "mv_video_capture.hpp"

namespace mindvision {
RM_VideoCapture::RM_VideoCapture(const CameraParam &_camera_param) {
  if (_camera_param.camera_mode == 0) {
    cameraInit(_camera_param.resolution.cols, _camera_param.resolution.rows,
               _camera_param.camera_exposuretime);

    iscamera0_open = true;
    printf("Set camera Industrial camera 📷📷📷\n");
  } else {
    iscamera0_open = false;
    printf("set camera USB camera 🔌🔌🔌\n");
  }
}

RM_VideoCapture::~RM_VideoCapture() {
  if (iscamera0_open) {
    CameraUnInit(hCamera);
    // 注意，现反初始化后再free
    free(g_pRgbBuffer);
    printf("🔷 release Industry camera success...... 🔷 ");
  } else {
    printf("🔷 release USB camera success...... 🔷 ");
  }
}

bool RM_VideoCapture::isindustryimgInput() {
  bool isindustry_camera_open = false;
  if (iscamera0_open == 1) {
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) ==
        CAMERA_STATUS_SUCCESS) {
      // 读取原图
      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
      if (iplImage) {
        cvReleaseImageHeader(&iplImage);
      }
      iplImage = cvCreateImageHeader(
          cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
      // 此处只是设置指针，无图像块数据拷贝，不需担心转换效率
      cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel);
    }
    isindustry_camera_open = true;
  } else {
    isindustry_camera_open = false;
  }
  return isindustry_camera_open;
}

/**
 * @brief 工业相机初始化
 * @return int
 */
int RM_VideoCapture::cameraInit(const int _CAMERA_RESOLUTION_COLS,
                                const int _CAMERA_RESOLUTION_ROWS,
                                const int _CAMERA_EXPOSURETIME) {
  CameraSdkInit(1);
  // 枚举设备，并建立设备列表
  iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
  printf("state = %d\n", iStatus);
  printf("count = %d\n", iCameraCounts);
  // 没有连接设备
  if (iCameraCounts == 0) {
    printf("❌🔌 no camera device connected ❌🔌\n");
    return -1;
  }
  // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
  iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
  // 初始化失败
  printf("state = %d\n", iStatus);
  if (iStatus != CAMERA_STATUS_SUCCESS) {
    printf("❌📷 camera init failed ❌📷\n");
    return -1;
  }

  // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
  CameraGetCapability(hCamera, &tCapability);
  g_pRgbBuffer =
      (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax *
                              tCapability.sResolutionRange.iWidthMax * 3);
  /*--------设置分辨率---------*/
  CameraGetImageResolution(hCamera, &pImageResolution);
  pImageResolution.iIndex = 0xFF;
  pImageResolution.iWidthFOV = _CAMERA_RESOLUTION_COLS;
  pImageResolution.iHeightFOV = _CAMERA_RESOLUTION_ROWS;
  pImageResolution.iWidth = _CAMERA_RESOLUTION_COLS;
  pImageResolution.iHeight = _CAMERA_RESOLUTION_ROWS;
  pImageResolution.iHOffsetFOV =
      static_cast<int>((1280 - _CAMERA_RESOLUTION_COLS) * 0.5);
  pImageResolution.iVOffsetFOV =
      static_cast<int>((1024 - _CAMERA_RESOLUTION_ROWS) * 0.5);
  CameraSetImageResolution(hCamera, &pImageResolution);
  /*--------设置分辨率---------*/

  /*--------设置曝光时间---------*/
  std::cout << CameraGetAeState(hCamera, &AEstate);
  std::cout << CameraSetAeState(hCamera, FALSE);
  CameraSetExposureTime(hCamera, _CAMERA_EXPOSURETIME);
  /*--------设置曝光时间---------*/

  /*让SDK进入工作模式，开始接收来自相机发送的图像数据。
   *如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像*/
  CameraPlay(hCamera);
  CameraReleaseImageBuffer(hCamera, pbyBuffer);
  /*
  其他的相机参数设置
  例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
        CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
        CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
        更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
  */
  if (tCapability.sIspCapacity.bMonoSensor) {
    channel = 1;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
  } else {
    channel = 3;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
  }
  return 1;
}

void RM_VideoCapture::cameraReleasebuff() {
  if (iscamera0_open) {
    // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
    // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
  }
}
}  // namespace mindvision
