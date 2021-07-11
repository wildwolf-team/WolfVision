#pragma once

#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace abstract_pnp {

struct PNP_Config {
  int    company      = 1;
  double ptz_camera_x = 0.0;
  double ptz_camera_y = 0.0;
  double ptz_camera_z = 0.0;
  float  barrel_ptz_offset_x = 0.0;
  float  barrel_ptz_offset_y = 0.0;
  float  offset_armor_pitch  = 0.0;
  float  offset_armor_yaw    = 0.0;
  int    small_armor_height  = 60,  small_armor_width = 140;
  int    big_armor_width     = 245, big_armor_height  = 60;
  int    light_size_width    = 10,  light_size_height = 55;
  int    buff_armor_width    = 250, buff_armor_height = 65;
};

class PNP {
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

 public:
  PNP_Config pnp_config_;

  PNP() {
    reference_Obj_.push_back(cv::Point3f(0.0, 0.0, 0.0));
    reference_Obj_.push_back(cv::Point3f(100, 0.0, 0.0));
    reference_Obj_.push_back(cv::Point3f(0.0, 100, 0.0));
    reference_Obj_.push_back(cv::Point3f(0.0, 0.0, 100));

    static double theta = 0;
    static double r_data[] = {1, 0,           0,
                              0, cos(theta),  sin(theta),
                              0, -sin(theta), cos(theta)};
    static double t_data[] = {static_cast<double>(pnp_config_.ptz_camera_x),
                              static_cast<double>(pnp_config_.ptz_camera_y),
                              static_cast<double>(pnp_config_.ptz_camera_z)};

    r_camera_ptz = cv::Mat(3, 3, CV_64FC1, r_data);
    t_camera_ptz = cv::Mat(3, 1, CV_64FC1, t_data);

    big_object_3d_.push_back(
      cv::Point3f(-pnp_config_.big_armor_width  * 0.5,
                  -pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.push_back(
      cv::Point3f(pnp_config_.big_armor_width   * 0.5,
                  -pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.push_back(
      cv::Point3f(pnp_config_.big_armor_width  * 0.5,
                  pnp_config_.big_armor_height * 0.5, 0));
    big_object_3d_.push_back(
      cv::Point3f(-pnp_config_.big_armor_width * 0.5,
                  pnp_config_.big_armor_height * 0.5, 0));

    small_object_3d_.push_back(
      cv::Point3f(-pnp_config_.small_armor_width  * 0.5,
                  -pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.push_back(
      cv::Point3f(pnp_config_.small_armor_width   * 0.5,
                  -pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.push_back(
      cv::Point3f(pnp_config_.small_armor_width  * 0.5,
                  pnp_config_.small_armor_height * 0.5, 0));
    small_object_3d_.push_back(
      cv::Point3f(-pnp_config_.small_armor_width * 0.5,
                  pnp_config_.small_armor_height * 0.5, 0));

    buff_object_3d_.push_back(
      cv::Point3f(-pnp_config_.buff_armor_width  * 0.5,
                  -pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.push_back(
      cv::Point3f(pnp_config_.buff_armor_width   * 0.5,
                  -pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.push_back(
      cv::Point3f(pnp_config_.buff_armor_width  * 0.5,
                  pnp_config_.buff_armor_height * 0.5, 0));
    buff_object_3d_.push_back(
      cv::Point3f(-pnp_config_.buff_armor_width * 0.5,
                  pnp_config_.buff_armor_height * 0.5, 0));
  }

  ~PNP() = default;

  std::vector<cv::Point3f> initialize3DPoints(int _armor_type) {
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

  std::vector<cv::Point3f> initialize3DPoints(int _width, int _heigth) {
    std::vector<cv::Point3f> object_3d;
    float half_x = _width  * 0.5;
    float half_y = _heigth * 0.5;

    object_3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x,  -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x,  half_y,  0));
    object_3d.push_back(cv::Point3f(-half_x, half_y,  0));

    return object_3d;
  }

  std::vector<cv::Point2f> initialize2DPoints(cv::RotatedRect _rect) {
    std::vector<cv::Point2f> target2d;
    static cv::Point2f vertex[4];
    static cv::Point2f lu, ld, ru, rd;

    _rect.points(vertex);
    std::sort(vertex, vertex + 4,
      [](const cv::Point2f &p1, const cv::Point2f &p2) {
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

    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);

    return target2d;
  }

  std::vector<cv::Point2f> initialize2DPoints(cv::Rect _rect) {
    cv::RotatedRect box = rectChangeRotatedrect(_rect);

    return this->initialize2DPoints(box);
  }

  cv::RotatedRect rectChangeRotatedrect(cv::Rect _rect) {
    cv::RotatedRect box =
      cv::RotatedRect((_rect.tl() + _rect.br()) / 2,
                      cv::Size(_rect.width / 2, _rect.height / 2),
                      0);

    return box;
  }

  cv::Mat cameraPtz(cv::Mat &_t) {
    cv::Mat position_in_ptz = r_camera_ptz * _t - t_camera_ptz;

    return position_in_ptz;
  }

  void drawCoordinate(cv::Mat &_draw_img,
                      cv::Mat &_rvec,         cv::Mat &_tvec,
                      cv::Mat &_cameraMatrix, cv::Mat &_distcoeffs) {
    std::vector<cv::Point2f> reference_Img;

    cv::projectPoints(reference_Obj_,
                      _rvec,         _tvec,
                      _cameraMatrix, _distcoeffs,
                      reference_Img);

    cv::line(_draw_img, reference_Img[0], reference_Img[1],
             cv::Scalar(0, 0, 255), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[2],
             cv::Scalar(0, 255, 0), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[3],
             cv::Scalar(255, 0, 0), 2);

    cv::imshow("[abstract_pnp] drawCoordinate", _draw_img);
  }

  float getPitch(float _dist, float _tvec_y, float _ballet_speed,
                 const int _company = 1) {
    _dist         /= _company;
    _tvec_y       /= _company;
    _ballet_speed /= _company;

    float y_temp, y_actual, dy;

    float       a       = 0.0;
    const float gravity = 10000.f / _company;

    y_temp = _tvec_y;

    for (size_t i = 0; i != 20; ++i) {
      a = static_cast<float>(atan2(y_temp, _dist));
      float t;
      t        = _dist / _ballet_speed * cos(a);
      y_actual = _ballet_speed * sin(a) * t - gravity * t * t / 2;
      dy       = _tvec_y - y_actual;
      y_temp   = y_temp + dy;

      if (fabsf(dy) < 0.01) { break; }
    }

    return a;
  }

  cv::Point3f getAngle(const cv::Mat &_pos_in_ptz,
                       const int     _bullet_speed,
                       const int     _company) {
    cv::Point3f angle;

    const double *_xyz  = reinterpret_cast<const double *>(_pos_in_ptz.data);
    double       down_t = 0.0;

    if (_bullet_speed > 10e-3) {
      down_t = _xyz[2] / (_bullet_speed * 1000);
    }

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    double xyz[3]         = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};

    if (pnp_config_.barrel_ptz_offset_y != 0.f) {
      double alpha = asin(static_cast<double>(pnp_config_.barrel_ptz_offset_y) /
                          sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2])),
             Beta  = 0.f;

      if (xyz[1] < 0) {
        Beta    = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(pnp_config_.barrel_ptz_offset_y)) {
        Beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - Beta));
      } else {
        Beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((Beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }

    if (pnp_config_.barrel_ptz_offset_x != 0.f) {
      double alpha = asin(static_cast<double>(pnp_config_.barrel_ptz_offset_x) /
                          sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])),
             Beta  = 0.f;

      if (xyz[0] > 0) {
        Beta    = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[0] < static_cast<double>(pnp_config_.barrel_ptz_offset_x)) {
        Beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - Beta));
      } else {
        Beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(Beta - alpha);  // camera coordinate
      }
    } else {
      angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }

    angle.z  = static_cast<float>(xyz[2]);
    angle.x  = static_cast<float>(angle.x) * 180 / CV_PI;
    angle.y  = static_cast<float>(angle.y) * 180 / CV_PI;
    angle.y -= getPitch(xyz[2], xyz[1], _bullet_speed * 1000, _company);

    return angle;
  }

  cv::Point3f getAngle(const cv::Mat &_pos_in_ptz,
                       const int     _bullet_speed,
                       const int     _company,
                       const int     _depth) {
    cv::Point3f angle;

    const double *_xyz  = reinterpret_cast<const double *>(_pos_in_ptz.data);
    double       down_t = 0.0;

    if (_bullet_speed > 10e-3) {
      down_t = _xyz[2] / (_bullet_speed * 1000);
    }

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    double xyz[3]         = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};

    if (pnp_config_.barrel_ptz_offset_y != 0.f) {
      double alpha = asin(static_cast<double>(pnp_config_.barrel_ptz_offset_y) /
                          sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2])),
             Beta = 0.f;

      if (xyz[1] < 0) {
        Beta    = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(pnp_config_.barrel_ptz_offset_y)) {
        Beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - Beta));
      } else {
        Beta    = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((Beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }

    if (pnp_config_.barrel_ptz_offset_x != 0.f) {
      double alpha = asin(static_cast<double>(pnp_config_.barrel_ptz_offset_x) /
                          sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])),
             Beta = 0.f;
      if (xyz[0] > 0) {
        Beta    = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[0] <
                 static_cast<double>(pnp_config_.barrel_ptz_offset_x)) {
        Beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - Beta));
      } else {
        Beta    = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(Beta - alpha);  // camera coordinate
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
};

}  // namespace abstract_pnp
