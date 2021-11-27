/**
 * @file abstract_pnp.hpp
 * @author XX (2796393320@qq.com)
 * @brief 角度结算基类
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace abstract_pnp {

struct PnP_Config {
  int    company      = 1;

  double ptz_camera_x = 0.0;
  double ptz_camera_y = 0.0;
  double ptz_camera_z = 0.0;

  float  barrel_ptz_offset_x = 0.0;
  float  barrel_ptz_offset_y = 0.0;
  float  offset_armor_pitch  = 0.0;
  float  offset_armor_yaw    = 0.0;
  // 各种装甲板模型
  int    small_armor_height  = 60,  small_armor_width = 140;
  int    big_armor_width     = 245, big_armor_height  = 60;
  int    light_size_width    = 10,  light_size_height = 55;
  int    buff_armor_width    = 250, buff_armor_height = 65;
};

class PnP {
 public:
  PnP_Config pnp_config_;

  PnP() {
    reference_Obj_.emplace_back(cv::Point3f(0.0, 0.0, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(100, 0.0, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(0.0, 100, 0.0));
    reference_Obj_.emplace_back(cv::Point3f(0.0, 0.0, 100));

    static double theta = 0;
    static double r_data[] = {1, 0,           0,
                              0, cos(theta),  sin(theta),
                              0, -sin(theta), cos(theta)};
    static double t_data[] = {static_cast<double>(pnp_config_.ptz_camera_x),
                              static_cast<double>(pnp_config_.ptz_camera_y),
                              static_cast<double>(pnp_config_.ptz_camera_z)};

    r_camera_ptz = cv::Mat(3, 3, CV_64FC1, r_data);
    t_camera_ptz = cv::Mat(3, 1, CV_64FC1, t_data);

    big_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.big_armor_width  * 0.5,
                  -pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.big_armor_width   * 0.5,
                  -pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.big_armor_width  * 0.5,
                  pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.big_armor_width * 0.5,
                  pnp_config_.big_armor_height * 0.5, 0));

    small_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.small_armor_width  * 0.5,
                  -pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.small_armor_width   * 0.5,
                  -pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.small_armor_width  * 0.5,
                  pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.small_armor_width * 0.5,
                  pnp_config_.small_armor_height * 0.5, 0));

    buff_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.buff_armor_width  * 0.5,
                  -pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.buff_armor_width   * 0.5,
                  -pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.emplace_back(
      cv::Point3f(pnp_config_.buff_armor_width  * 0.5,
                  pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.emplace_back(
      cv::Point3f(-pnp_config_.buff_armor_width * 0.5,
                  pnp_config_.buff_armor_height * 0.5, 0));
  }

  ~PnP() = default;
  /**
   * @brief 初始化目标 3d 点
   *
   * @param _armor_type               装甲板类型
   * @return std::vector<cv::Point3f> 返回目标 3d 点
   * @author XX
   */
  std::vector<cv::Point3f> initialize3DPoints(int _armor_type) {
    // 选择装甲板类型
    switch (_armor_type) {
      case 0:
        return small_object_3d_;
        break;
      case 1:
        return big_object_3d_;
        break;
      case 2:
        return buff_object_3d_;
        break;
      default:
        return small_object_3d_;
        break;
    }
  }
  /**
   * @brief 初始化目标 3d 点
   *
   * @param _width                    目标实际宽度
   * @param _heigth                   目标实际高度
   * @return std::vector<cv::Point3f> 返回目标 3d 点
   * @author XX
   */
  std::vector<cv::Point3f> initialize3DPoints(int _width, int _heigth) {
    float half_x = _width  * 0.5;
    float half_y = _heigth * 0.5;

    // 3d点模型赋值
    std::vector<cv::Point3f> object_3d = {
      cv::Point3f(-half_x, -half_y, 0),
      cv::Point3f(half_x,  -half_y, 0),
      cv::Point3f(half_x,  half_y,  0),
      cv::Point3f(-half_x, half_y,  0)
    };

    return object_3d;
  }
  /**
   * @brief 初始化目标 2d 点
   *
   * @param _rect                     目标旋转矩形
   * @return std::vector<cv::Point2f> 返回目标 2d 点
   * @author XX
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::RotatedRect _rect) {
    std::vector<cv::Point2f> target2d;

    static cv::Point2f vertex[4];
    static cv::Point2f lu, ld, ru, rd;

    _rect.points(vertex);
    // 排序 左上->右上->右下->左下
    std::sort(vertex, vertex + 4,
      [](const cv::Point2f& p1, const cv::Point2f& p2) {
          return p1.x < p2.x;
      });

    if (vertex[0].y < vertex[1].y) {
      lu = vertex[0];
      ld = vertex[1];
    } else {
      lu = vertex[1];
      ld = vertex[0];
    }
    if (vertex[2].y < vertex[3].y) {
      ru = vertex[2];
      rd = vertex[3];
    } else {
      ru = vertex[3];
      rd = vertex[2];
    }

    target2d.emplace_back(ld);
    target2d.emplace_back(rd);
    target2d.emplace_back(ru);
    target2d.emplace_back(lu);
    return target2d;
  }
  /**
   * @brief 初始化目标 2d 点
   *
   * @param _rect                     目标外接矩形
   * @return std::vector<cv::Point2f> 返回目标 2d 点
   * @author XX
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::Rect _rect) {
    cv::RotatedRect box = rectChangeRotatedrect(_rect);

    return initialize2DPoints(box);
  }
  /**
   * @brief 外接矩形转旋转矩形
   *
   * @param _rect            目标外接矩形
   * @return cv::RotatedRect 返回旋转矩形
   * @author XX
   */
  cv::RotatedRect rectChangeRotatedrect(cv::Rect _rect) {
    cv::RotatedRect box =
      cv::RotatedRect((_rect.tl() + _rect.br()) / 2,
                      cv::Size(_rect.width / 2, _rect.height / 2),
                      0);

    return box;
  }
  /**
   * @brief 转换坐标系
   *
   * @param _t       旋转向量
   * @return cv::Mat 返回转化后的旋转向量
   * @author XX
   */
  cv::Mat cameraPtz(cv::Mat& _t) {
    return r_camera_ptz * _t - t_camera_ptz;
  }
  /**
   * @brief 绘制坐标系
   *
   * @param _draw_img     画板
   * @param _rvec         旋转矩阵
   * @param _tvec         旋转向量
   * @param _cameraMatrix 相机内参
   * @param _distcoeffs   相机外参
   * @author XX
   */
  void drawCoordinate(cv::Mat& _draw_img,
                      cv::Mat& _rvec,         cv::Mat& _tvec,
                      cv::Mat& _cameraMatrix, cv::Mat& _distcoeffs) {
    std::vector<cv::Point2f> reference_Img;

    cv::projectPoints(reference_Obj_,
                      _rvec,         _tvec,
                      _cameraMatrix, _distcoeffs,
                      reference_Img);
    // 绘制坐标系
    cv::line(_draw_img, reference_Img[0], reference_Img[1],
             cv::Scalar(0, 0, 255), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[2],
             cv::Scalar(0, 255, 0), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[3],
             cv::Scalar(255, 0, 0), 2);

    cv::imshow("[abstract_pnp] drawCoordinate() -> _draw_img", _draw_img);
  }
  /**
   * @brief 计算子弹下坠
   *
   * @param _dist         目标深度
   * @param _tvec_y       目标高度
   * @param _ballet_speed 子弹速度
   * @param _company      计算单位
   * @return float        返回补偿角度
   * @author XX
   */
  float getPitch(float       _dist,
                 float       _tvec_y,
                 float       _ballet_speed,
                 const int   _company = 1) {
    // 选择计算单位
    _dist         /= _company;
    _tvec_y       /= _company;
    _ballet_speed /= _company;

    float       y_temp   = _tvec_y;
    float       y_actual = 0.f;
    float       dy       = 0.f;
    float       a        = 0.f;
    const float gravity  = 10000.f / _company;

    for (size_t i = 0; i != 20; ++i) {
      a = static_cast<float>(atan2(y_temp, _dist));
      // 子弹飞行时间
      float t =  (float((exp( 0.001 * _dist)-1) / ( 0.001 * _dist * 0.001 * _ballet_speed * cos(a))));
      // float t = _dist / _ballet_speed * cos(a);

      y_actual  = _ballet_speed * sin(a) * t - gravity * t * t / 2;
      dy        = _tvec_y - y_actual;
      y_temp   += dy;

      if (fabsf(dy) < 1e-2) { break; }
    }

    return a;
  }
  /**
   * @brief 计算云台偏差角度
   *
   * @param _pos_in_ptz   旋转向量
   * @param _bullet_speed 子弹速度
   * @param _company      计算子弹下坠单位
   * @return cv::Point3f  返回 Yaw Pitch 轴的偏移量和深度（mm）
   * @author XX
   */
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz,
                       const int      _bullet_speed,
                       const int      _company) {
    cv::Point3f angle;

    const double *_xyz  = reinterpret_cast<const double *>(_pos_in_ptz.data);
    double       down_t = 0.f;

    if (_bullet_speed > 1e-2) {
      down_t = _xyz[2] / (_bullet_speed * 1000);
    }

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    double xyz[3]         = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};

    if (pnp_config_.barrel_ptz_offset_y != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_y) /
             sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[1] < 0) {
        beta    = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(pnp_config_.barrel_ptz_offset_y)) {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }

    if (pnp_config_.barrel_ptz_offset_x != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_x) /
             sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[0] > 0) {
        beta    = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[0] < static_cast<double>(pnp_config_.barrel_ptz_offset_x)) {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(beta - alpha);  // camera coordinate
      }
    } else {
      angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }

    // 深度
    angle.z  = static_cast<float>(xyz[2]);
    // Yaw
    angle.x  = static_cast<float>(angle.x) * 180 / CV_PI;
    // Pitch
    angle.y  = static_cast<float>(angle.y) * 180 / CV_PI;
    angle.y -= getPitch(xyz[2], xyz[1], _bullet_speed * 1000, _company);

    return angle;
  }
  /**
   * @brief 计算云台偏差角度
   *
   * @param _pos_in_ptz   旋转向量
   * @param _bullet_speed 子弹速度
   * @param _company      子弹下坠单位
   * @param _depth        距目标的深度
   * @return cv::Point3f  返回 Yaw Pitch 轴的偏移量和深度（mm）
   * @author XX
   */
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz,
                       const int      _bullet_speed,
                       const int      _company,
                       const int      _depth) {
    cv::Point3f angle;

    const double *_xyz  = reinterpret_cast<const double *>(_pos_in_ptz.data);
    double       down_t = 0.f;

    if (_bullet_speed > 1e-2) {
      down_t = _xyz[2] / (_bullet_speed * 1000);
    }

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    double xyz[3]         = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};

    if (pnp_config_.barrel_ptz_offset_y != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_y) /
             sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[1] < 0) {
        beta    = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(pnp_config_.barrel_ptz_offset_y)) {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }

    if (pnp_config_.barrel_ptz_offset_x != 0.f) {
      double alpha =
        asin(static_cast<double>(pnp_config_.barrel_ptz_offset_x) /
             sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));
      double beta  = 0.f;

      if (xyz[0] > 0) {
        beta    = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + beta));  // camera coordinate
      } else if (xyz[0] <
                 static_cast<double>(pnp_config_.barrel_ptz_offset_x)) {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - beta));
      } else {
        beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(beta - alpha);  // camera coordinate
      }
    } else {
      angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }

    angle.z  = _depth;
    angle.x  = static_cast<float>(angle.x) * 180 / CV_PI;
    angle.y  = static_cast<float>(angle.y) * 180 / CV_PI;
    angle.y -= getPitch(_depth, xyz[1], _bullet_speed * 1000, _company);

    return angle;
  }

 private:
  cv::Mat pnp_config_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);

  std::vector<cv::Point3f> reference_Obj_;
  std::vector<cv::Point3f> big_object_3d_;
  std::vector<cv::Point3f> small_object_3d_;
  std::vector<cv::Point3f> buff_object_3d_;

  double theta = 0.0;
  double r_data[9];
  double t_data[3];

  cv::Mat r_camera_ptz;
  cv::Mat t_camera_ptz;
};

}  // namespace abstract_pnp
