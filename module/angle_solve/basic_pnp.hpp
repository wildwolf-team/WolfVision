#pragma once

#include <string>
#include <vector>

#include <fmt/core.h>
#include <fmt/color.h>

#include "abstract_pnp.hpp"

namespace basic_pnp {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_pnp");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_pnp");

struct PnP_Information {
  float yaw_angle;
  float pitch_angle;
  int   depth;

  PnP_Information() {
    yaw_angle   = 0.f;
    pitch_angle = 0.f;
    depth       = 0;
  }
};

class PnP : public abstract_pnp::PnP {
 public:
  PnP() = default;
  explicit PnP(std::string _camera_path, std::string _pnp_config_path);

  ~PnP() = default;

  inline float  returnYawAngle()   { return pnp_info_.yaw_angle; }
  inline float  returnPitchAngle() { return pnp_info_.pitch_angle; }
  inline float  returnDepth()      { return pnp_info_.depth; }
  inline double returnTvecTx()     { return tvec_.ptr<double>(0)[0]; }
  inline double returnTvecTy()     { return tvec_.ptr<double>(0)[1]; }
  inline double returnTvecTz()     { return tvec_.ptr<double>(0)[2]; }

  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::Mat&        _src_img,
                cv::RotatedRect _rect);

  void solvePnP(int      _ballet_speed,
                int      _armor_type,
                cv::Mat& _src_img,
                cv::Rect _rect);

  void solvePnP(int             _ballet_speed,
                int             _width,
                int             _height,
                cv::Mat&        _src_img,
                cv::RotatedRect _rect);

  void solvePnP(int      _ballet_speed,
                int      _width,
                int      _height,
                cv::Mat& _src_img,
                cv::Rect _rect);

  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::RotatedRect _rect);

  void solvePnP(int             _ballet_speed,
                int             _width,
                int             _height,
                cv::RotatedRect _rect);

  void solvePnP(int      _ballet_speed,
                int      _armor_type,
                cv::Rect _rect);

  void solvePnP(int      _ballet_speed,
                int      _width,
                int      _height,
                cv::Rect _rect);

  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::RotatedRect _rect,
                int             _depth);

  void solvePnP(int                      _ballet_speed,
                int                      _armor_type,
                std::vector<cv::Point2f> _target_2d,
                int                      _depth);

  void solvePnP(int                      _ballet_speed,
                int                      _armor_type,
                cv::Mat&                 _src_img,
                std::vector<cv::Point2f> _target_2d);

  void solvePnP(int                      _ballet_speed,
                int                      _armor_type,
                std::vector<cv::Point2f> _target_2d);

 private:
  PnP_Information pnp_info_;

  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat rvec_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);

  std::vector<cv::Point2f> target_2d_;
  std::vector<cv::Point3f> object_3d_;
};

}  // namespace basic_pnp
