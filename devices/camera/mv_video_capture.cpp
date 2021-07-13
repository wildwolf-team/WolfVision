#include "mv_video_capture.hpp"

namespace mindvision {

VideoCapture::VideoCapture(const CameraParam &_camera_param) {
  if (_camera_param.camera_mode == 0) {
    cameraInit(_camera_param.resolution.cols,
               _camera_param.resolution.rows,
               _camera_param.camera_exposuretime);

      iscamera0_open = true;

      fmt::print("[{}] Using mindvision industrial camera: {}\n", idntifier_green, _camera_param.camera_mode);
  } else {
    iscamera0_open = false;

    fmt::print("[{}] Not using mindvision industrial camera: {}\n", idntifier_green, _camera_param.camera_mode);
  }
}

VideoCapture::~VideoCapture() {
  if (iscamera0_open) {
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    fmt::print("[{}] Released mindvision industrial camera: {}\n", idntifier_green, iStatus);
  }
}

bool VideoCapture::isindustryimgInput() {
  bool isindustry_camera_open = false;

  if (iscamera0_open == 1) {
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

      if (iplImage) {
        cvReleaseImageHeader(&iplImage);
      }

      iplImage =
        cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);

      cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel);
    }

    isindustry_camera_open = true;
  } else {
    isindustry_camera_open = false;
  }

  return isindustry_camera_open;
}

int VideoCapture::cameraInit(const int _CAMERA_RESOLUTION_COLS,
                             const int _CAMERA_RESOLUTION_ROWS,
                             const int _CAMERA_EXPOSURETIME) {
  CameraSdkInit(1);

  iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

  if (iCameraCounts == 0) {
    fmt::print("[{}] Error, no mindvision industrial camera detected: {}\n", idntifier_red, iCameraCounts);

    return -1;
  }

  iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

  if (iStatus != CAMERA_STATUS_SUCCESS) {
    fmt::print("[{}] Error, Init mindvision industrial camera failed: {}\n", idntifier_red, iStatus);

    return -1;
  }

  fmt::print("[{}] Info, Init mindvision industrial camera success: {}\n", idntifier_green, iStatus);

  CameraGetCapability(hCamera, &tCapability);

  g_pRgbBuffer =
    static_cast<unsigned char*>(
      malloc(tCapability.sResolutionRange.iHeightMax *
             tCapability.sResolutionRange.iWidthMax  * 3));

  CameraGetImageResolution(hCamera, &pImageResolution);

  pImageResolution.iIndex      = 0xFF;
  pImageResolution.iWidthFOV   = _CAMERA_RESOLUTION_COLS;
  pImageResolution.iHeightFOV  = _CAMERA_RESOLUTION_ROWS;
  pImageResolution.iWidth      = _CAMERA_RESOLUTION_COLS;
  pImageResolution.iHeight     = _CAMERA_RESOLUTION_ROWS;
  pImageResolution.iHOffsetFOV = static_cast<int>((1280 - _CAMERA_RESOLUTION_COLS) * 0.5);
  pImageResolution.iVOffsetFOV = static_cast<int>((1024 - _CAMERA_RESOLUTION_ROWS) * 0.5);

  CameraSetImageResolution(hCamera, &pImageResolution);

  fmt::print("[{}] Info, camera status: {}, {}\n", idntifier_green, CameraGetAeState(hCamera, &AEstate), CameraGetAeState(hCamera, FALSE));

  CameraSetExposureTime(hCamera, _CAMERA_EXPOSURETIME);
  CameraPlay(hCamera);
  CameraReleaseImageBuffer(hCamera, pbyBuffer);

  if (tCapability.sIspCapacity.bMonoSensor) {
    channel = 1;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
  } else {
    channel = 3;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
  }

  return 1;
}

void VideoCapture::cameraReleasebuff() {
  // After successfully calling CameraGetImageBuffer, CameraReleaseImageBuffer must be called to release the acquired buffer
  // Otherwise, when CameraGetImageBuffer is called again, the program will hang and block until CameraReleaseImageBuffer is called in another thread
  if (iscamera0_open) { CameraReleaseImageBuffer(hCamera, pbyBuffer); }
}

}  // namespace mindvision
