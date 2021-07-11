#pragma once

#include <fmt/core.h>
#include <fmt/color.h>

#include <string>
#include <vector>

#include "abstract_pnp.hpp"

namespace basic_pnp {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold,
                                   "basic_pnp");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold,
                                   "basic_pnp");

typedef struct Solvepnp_Information {
  float yaw_angle;
  float pitch_angle;
  int depth;
  Solvepnp_Information() {
    yaw_angle   = 0.f;
    pitch_angle = 0.f;
    depth       = 0;
  }
} Solvepnp_Info;

class RM_Solvepnp : public abstract_pnp::Abstract_Solvepnp {
 private:
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat rvec_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
  std::vector<cv::Point2f> target_2d_;
  std::vector<cv::Point3f> object_3d_;
  Solvepnp_Info solvepnp_info_;

 public:
  RM_Solvepnp(std::string _camera_path, std::string _pnp_config_path);
  RM_Solvepnp();
  ~RM_Solvepnp();

  inline float returnYawAngle() { return solvepnp_info_.yaw_angle; }
  inline float returnPitchAngle() { return solvepnp_info_.pitch_angle; }
  inline float returnDepth() { return solvepnp_info_.depth; }
  inline double returnTvecTx() { return tvec_.ptr<double>(0)[0]; }
  inline double returnTvecTy() { return tvec_.ptr<double>(0)[1]; }
  inline double returnTvecTz() { return tvec_.ptr<double>(0)[2]; }

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::Mat &_src_img,
                    cv::RotatedRect _rect);

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::Mat &_src_img,
                    cv::Rect _rect);

  void run_Solvepnp(int _ballet_speed, int _width, int _height,
                    cv::Mat &_src_img, cv::RotatedRect _rect);

  void run_Solvepnp(int _ballet_speed, int _width, int _height,
                    cv::Mat &_src_img, cv::Rect _rect);

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::RotatedRect _rect);

  void run_Solvepnp(int _ballet_speed, int _width, int _height,
                    cv::RotatedRect _rect);

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::Rect _rect);

  void run_Solvepnp(int _ballet_speed, int _width, int _height, cv::Rect _rect);

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::RotatedRect _rect,
                    int _depth);

  void run_Solvepnp(int _ballet_speed, int _armor_type,
                    std::vector<cv::Point2f> _target_2d, int _depth);

  void run_Solvepnp(int _ballet_speed, int _armor_type, cv::Mat &_src_img,
                    std::vector<cv::Point2f> _target_2d);

  void run_Solvepnp(int _ballet_speed, int _armor_type,
                    std::vector<cv::Point2f> _target_2d);
};
}  // namespace basic_pnp
