#include "basic_pnp.hpp"

namespace basic_pnp {

RM_Solvepnp::RM_Solvepnp(std::string _camera_path,
                         std::string _pnp_config_path) {
  cv::FileStorage fs_camera(_camera_path, cv::FileStorage::READ);

  fs_camera["camera-matrix"] >> cameraMatrix_;
  fs_camera["distortion"] >> distCoeffs_;

  cv::FileStorage fs_config(_pnp_config_path, cv::FileStorage::READ);

  fs_config["PTZ_CAMERA_X"] >> pnp_config_.ptz_camera_x;
  fs_config["PTZ_CAMERA_Y"] >> pnp_config_.ptz_camera_y;
  fs_config["PTZ_CAMERA_Z"] >> pnp_config_.ptz_camera_z;

  fs_config["PTZ_BARREL_X"] >> pnp_config_.barrel_ptz_offset_x;
  fs_config["PTZ_BARREL_Y"] >> pnp_config_.barrel_ptz_offset_y;

  fs_config["OFFSET_ARMOR_YAW"] >> pnp_config_.offset_armor_yaw;
  fs_config["OFFSET_ARMOR_PITCH"] >> pnp_config_.offset_armor_pitch;

  fs_config["COMPANY"] >> pnp_config_.company;
  fs_config["BIG_ARMOR_WIDTH"] >> pnp_config_.big_armor_width;
  fs_config["BIG_ARMOR_HEIGHT"] >> pnp_config_.big_armor_height;

  fs_config["SMALL_ARMOR_WIDTH"] >> pnp_config_.small_armor_width;
  fs_config["SMALL_ARMOR_HEIGHT"] >> pnp_config_.small_armor_height;

  fs_config["BUFF_ARMOR_WIDTH"] >> pnp_config_.buff_armor_width;
  fs_config["BUFF_ARMOR_HEIGHT"] >> pnp_config_.buff_armor_height;

  fmt::print("\n");

  std::cout << cameraMatrix_ << std::endl;
  std::cout << distCoeffs_ << std::endl;

  fmt::print("[{}] Info, ptz_camera_x: {}\n", idntifier_green,
             pnp_config_.ptz_camera_x);
  fmt::print("[{}] Info, ptz_camera_y: {}\n", idntifier_green,
             pnp_config_.ptz_camera_y);
  fmt::print("[{}] Info, ptz_camera_z: {}\n", idntifier_green,
             pnp_config_.ptz_camera_z);

  fmt::print("[{}] Info, ptz_barrel_x: {}\n", idntifier_green,
             pnp_config_.barrel_ptz_offset_x);
  fmt::print("[{}] Info, ptz_barrel_y: {}\n", idntifier_green,
             pnp_config_.barrel_ptz_offset_y);

  fmt::print("[{}] Info, offset_armor_yaw: {}\n", idntifier_green,
             pnp_config_.offset_armor_yaw);
  fmt::print("[{}] Info, offset_armor_pitch: {}\n", idntifier_green,
             pnp_config_.offset_armor_pitch);

  switch (pnp_config_.company) {
    case 1:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: "
                 "millimeter.\n", idntifier_green);
      break;
    case 10:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: "
                 "centimeter.\n", idntifier_green);
    case 100:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: "
                 "decimeter.\n", idntifier_green);
    case 1000:
      fmt::print("[{}] Info, Gravity compensation algorithm unit: "
                 "meter.\n", idntifier_green);
    default:
      break;
  }

  fmt::print("[{}] PNP initialize success.\n", idntifier_green);

  fmt::print("\n");
}

RM_Solvepnp::RM_Solvepnp() {}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::Mat &_src_img, cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::Mat &_src_img, cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _width, int _height,
                               cv::Mat &_src_img, cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _width, int _height,
                               cv::Mat &_src_img, cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;
  object_3d_.clear();

  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _width, int _height,
                               cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _width, int _height,
                               cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               std::vector<cv::Point2f> _target_2d) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::Mat &_src_img,
                               std::vector<cv::Point2f> _target_2d) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               cv::RotatedRect _rect, int _depth) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1, _depth);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}

void RM_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                               std::vector<cv::Point2f> _target_2d,
                               int _depth) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = _target_2d;

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz = this->camera_Ptz(tvec_);
  cv::Point3f angle = this->get_Angle(ptz, _ballet_speed, 1, _depth);

  solvepnp_info_.pitch_angle = angle.y + pnp_config_.offset_armor_pitch;
  solvepnp_info_.yaw_angle   = angle.x + pnp_config_.offset_armor_yaw;
  solvepnp_info_.depth       = angle.z;

  object_3d_.clear();
  std::vector<cv::Point3f>(object_3d_).swap(object_3d_);
  target_2d_.clear();
  std::vector<cv::Point2f>(target_2d_).swap(target_2d_);
}
RM_Solvepnp::~RM_Solvepnp() {}

}  // namespace basic_pnp
