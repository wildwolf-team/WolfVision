#include "basic_pnp.hpp"

namespace basic_pnp {
/**
 * @brief 初始化相机参数
 *
 * @param _camera_path 相机标定文件路径
 */
Rm_Solvepnp::Rm_Solvepnp(std::string _camera_path,
                         std::string _pnp_config_path) {
  //读取摄像头标定xml文件
  cv::FileStorage fs_camera(_camera_path, cv::FileStorage::READ);
  //读取相机内参和畸变矩阵
  fs_camera["camera-matrix"] >> cameraMatrix_;
  fs_camera["distortion"] >> distCoeffs_;
  //读取自定义xml文件
  cv::FileStorage fs_config(_pnp_config_path, cv::FileStorage::READ);
  //相机与云台偏移量
  fs_config["PTZ_CAMERA_X"] >> pnp_config_.ptz_camera_x;
  fs_config["PTZ_CAMERA_Y"] >> pnp_config_.ptz_camera_y;
  fs_config["PTZ_CAMERA_Z"] >> pnp_config_.ptz_camera_z;
  //相机枪管偏移量
  fs_config["PTZ_BARREL_X"] >> pnp_config_.barrel_ptz_offset_x;
  fs_config["PTZ_BARREL_Y"] >> pnp_config_.barrel_ptz_offset_y;
  //固定偏移量
  fs_config["OFFSET_ARMOR_YAW"] >> pnp_config_.offset_armor_yaw;
  fs_config["OFFSET_ARMOR_PITCH"] >> pnp_config_.offset_armor_pitch;
  //是否绘制坐标系
  fs_config["DRAW_XYZ"] >> pnp_config_.draw_xyz;
  //选择补偿算法单位
  fs_config["COMPANY"] >> pnp_config_.company;
  //打印
  std::cout << "------------------------------------------------------------"
            << std::endl;
  std::cout << cameraMatrix_ << std::endl;
  std::cout << distCoeffs_ << std::endl;

  std::cout << std::endl;

  std::cout << "ptz_camera_x = " << pnp_config_.ptz_camera_x << std::endl;
  std::cout << "ptz_camera_y = " << pnp_config_.ptz_camera_y << std::endl;
  std::cout << "ptz_camera_z = " << pnp_config_.ptz_camera_z << std::endl;

  std::cout << std::endl;

  std::cout << "ptz_barrel_x = " << pnp_config_.barrel_ptz_offset_x
            << std::endl;
  std::cout << "ptz_barrel_y = " << pnp_config_.barrel_ptz_offset_y
            << std::endl;

  std::cout << std::endl;

  std::cout << "offset_armor_yaw = " << pnp_config_.offset_armor_yaw
            << std::endl;
  std::cout << "offset_armor_pitch = " << pnp_config_.offset_armor_pitch
            << std::endl;

  std::cout << std::endl;

  if (pnp_config_.draw_xyz == 1) {
    std::cout << "打开坐标系绘制" << std::endl;
  }
  switch (pnp_config_.company) {
    case 1:
      std::cout << "重力补偿算法单位为mm" << std::endl;
      break;
    case 10:
      std::cout << "重力补偿算法单位为cm" << std::endl;
    case 100:
      std::cout << "重力补偿算法单位为dm" << std::endl;
    case 1000:
      std::cout << "重力补偿算法单位为m" << std::endl;
    default:
      break;
  }

  std::cout << std::endl;

  std::cout << "PNP初始化成功" << std::endl;
  std::cout << "💚💚💚💚💚💚💚💚💚💚💚💚" << std::endl;
  std::cout << "------------------------------------------------------------"
            << std::endl;
}

Rm_Solvepnp::Rm_Solvepnp() {}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _armor_type 装甲板类型
 * @param _src_img 原图像
 * @param _rect 目标旋转矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                                      cv::Mat &_src_img,
                                      cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _armor_type 装甲板类型
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                                      cv::Mat &_src_img, cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _width 目标实际宽度
 * @param _height 目标实际高度
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _width,
                                      int _height, cv::Mat &_src_img,
                                      cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _width 目标实际宽度
 * @param _height 目标实际高度
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _width,
                                      int _height, cv::Mat &_src_img,
                                      cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);

  this->draw_Coordinate(_src_img, rvec_, tvec_, cameraMatrix_, distCoeffs_);

  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _armor_type 装甲板类型
 * @param _src_img 原图像
 * @param _rect 目标旋转矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                                      cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _armor_type 装甲板类型
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _armor_type,
                                      cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_armor_type);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _width 目标实际宽度
 * @param _height 目标实际高度
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _width,
                                      int _height, cv::RotatedRect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

/**
 * @brief 计算目标偏移角度
 *
 * @param _ballet_speed 子弹速度
 * @param _width 目标实际宽度
 * @param _height 目标实际高度
 * @param _src_img 原图像
 * @param _rect 目标外接矩形
 * @return cv::Point3f yaw pitch偏移量 和 目标depth信息
 */
cv::Point3f Rm_Solvepnp::run_Solvepnp(int _ballet_speed, int _width,
                                      int _height, cv::Rect _rect) {
  object_3d_ = this->initialize_3d_Points(_width, _height);
  target_2d_ = this->initialize_2d_Points(_rect);

  cv::solvePnP(object_3d_, target_2d_, cameraMatrix_, distCoeffs_, rvec_, tvec_,
               false, cv::SOLVEPNP_ITERATIVE);
  cv::Mat ptz =
      this->camera_Ptz(tvec_, pnp_config_.ptz_camera_x,
                       pnp_config_.ptz_camera_y, pnp_config_.ptz_camera_z);
  cv::Point3f angle =
      this->get_Angle(ptz, _ballet_speed, 1, pnp_config_.barrel_ptz_offset_x,
                      pnp_config_.barrel_ptz_offset_y);

  angle.y += pnp_config_.offset_armor_pitch;
  angle.x += pnp_config_.offset_armor_yaw;

  return angle;
}

Rm_Solvepnp::~Rm_Solvepnp() {}
}  // namespace basic_pnp
