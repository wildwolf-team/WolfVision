/**
 * @file basic_pnp.cpp
 * @author XX (2796393320@qq.com)
 * @brief 角度结算
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#include "basic_pnp.hpp"

namespace basic_pnp {

PnP::PnP(std::string _camera_path,
         std::string _pnp_config_path) {
  cv::FileStorage fs_camera(_camera_path, cv::FileStorage::READ);

  fs_camera["camera-matrix"]      >> cameraMatrix_;
  fs_camera["distortion"]         >> distCoeffs_;

  cv::FileStorage fs_config(_pnp_config_path, cv::FileStorage::READ);

  fs_config["PTZ_CAMERA_X"]       >> pnp_config_.ptz_camera_x;
  fs_config["PTZ_CAMERA_Y"]       >> pnp_config_.ptz_camera_y;
  fs_config["PTZ_CAMERA_Z"]       >> pnp_config_.ptz_camera_z;

  fs_config["PTZ_BARREL_X"]       >> pnp_config_.barrel_ptz_offset_x;
  fs_config["PTZ_BARREL_Y"]       >> pnp_config_.barrel_ptz_offset_y;

  fs_config["OFFSET_ARMOR_YAW"]   >> pnp_config_.offset_armor_yaw;
  fs_config["OFFSET_ARMOR_PITCH"] >> pnp_config_.offset_armor_pitch;

  fs_config["COMPANY"]            >> pnp_config_.company;
  fs_config["BIG_ARMOR_WIDTH"]    >> pnp_config_.big_armor_width;
  fs_config["BIG_ARMOR_HEIGHT"]   >> pnp_config_.big_armor_height;

  fs_config["SMALL_ARMOR_WIDTH"]  >> pnp_config_.small_armor_width;
  fs_config["SMALL_ARMOR_HEIGHT"] >> pnp_config_.small_armor_height;

  fs_config["BUFF_ARMOR_WIDTH"]   >> pnp_config_.buff_armor_width;
  fs_config["BUFF_ARMOR_HEIGHT"]  >> pnp_config_.buff_armor_height;

  fmt::print("[{}] Info, ptz_camera_x,y,z: {}, {}, {}\n", idntifier_green,
             pnp_config_.ptz_camera_x,
             pnp_config_.ptz_camera_y,
             pnp_config_.ptz_camera_z);

  fmt::print("[{}] Info, ptz_barrel_x,y: {}, {}\n", idntifier_green,
             pnp_config_.barrel_ptz_offset_x,
             pnp_config_.barrel_ptz_offset_y);

  fmt::print("[{}] Info, offset_armor_yaw,pitch: {}, {}\n", idntifier_green,
             pnp_config_.offset_armor_yaw,
             pnp_config_.offset_armor_pitch);

  switch (pnp_config_.company) {
    case 1:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: millimeter\n", idntifier_green);
      break;
    case 10:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: centimeter\n", idntifier_green);
      break;
    case 100:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: decimeter\n", idntifier_green);
      break;
    case 1000:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: meter\n", idntifier_green);
      break;
    default:
      fmt::print("[{}] Error, Gravity compensation algorithm unit: unknown\n", idntifier_red);
      break;
  }

  fmt::print("[{}] Init PnP configuration finished\n", idntifier_green);
}

void PnP::solvePnP(int             _ballet_speed,
                   int             _armor_type,
                   cv::Mat&        _src_img,
                   cv::RotatedRect _rect) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  drawCoordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);
  // 固定偏移量
  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int      _ballet_speed,
                   int      _armor_type,
                   cv::Mat& _src_img,
                   cv::Rect _rect) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  drawCoordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int             _ballet_speed,
                   int             _width,
                   int             _height,
                   cv::Mat&        _src_img,
                   cv::RotatedRect _rect) {
  object_3d_ = initialize3DPoints(_width, _height);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  drawCoordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int      _ballet_speed,
                   int      _width,
                   int      _height,
                   cv::Mat& _src_img,
                   cv::Rect _rect) {
  object_3d_ = initialize3DPoints(_width, _height);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  drawCoordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int            _ballet_speed,
                   int            _armor_type,
                  cv::RotatedRect _rect) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int      _ballet_speed,
                   int      _armor_type,
                   cv::Rect _rect) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int             _ballet_speed,
                   int             _width,
                   int             _height,
                   cv::RotatedRect _rect) {
  object_3d_ = initialize3DPoints(_width, _height);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int      _ballet_speed,
                   int      _width,
                   int      _height,
                   cv::Rect _rect) {
  object_3d_ = initialize3DPoints(_width, _height);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int                      _ballet_speed,
                   int                      _armor_type,
                   std::vector<cv::Point2f> _target_2d) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int                      _ballet_speed,
                   int                      _armor_type,
                   cv::Mat&                 _src_img,
                   std::vector<cv::Point2f> _target_2d) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  drawCoordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int             _ballet_speed,
                   int             _armor_type,
                   cv::RotatedRect _rect,
                   int             _depth) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = initialize2DPoints(_rect);

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1, _depth);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

void PnP::solvePnP(int                      _ballet_speed,
                   int                      _armor_type,
                   std::vector<cv::Point2f> _target_2d,
                   int                      _depth) {
  object_3d_ = initialize3DPoints(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_,
               target_2d_,
               cameraMatrix_,
               distCoeffs_,
               rvec_,
               tvec_,
               false,
               cv::SOLVEPNP_ITERATIVE);

  cv::Mat     ptz   = cameraPtz(tvec_);
  cv::Point3f angle = getAngle(ptz, _ballet_speed, 1, _depth);

  pnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  pnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  pnp_info_.depth       = angle.z;

  object_3d_.clear();
  object_3d_.shrink_to_fit();
  target_2d_.clear();
  target_2d_.shrink_to_fit();
}

}  // namespace basic_pnp
