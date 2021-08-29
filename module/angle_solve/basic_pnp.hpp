/**
 * @file basic_pnp.hpp
 * @author XX (2796393320@qq.com)
 * @brief 角度结算
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
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
  /**
   * @brief 返回 Yaw 轴角度
   *
   * @return float
   * @author XX
   */
  inline float  returnYawAngle()   { return pnp_info_.yaw_angle; }
  /**
   * @brief 返回 Pitch 轴角度
   *
   * @return float
   * @author XX
   */
  inline float  returnPitchAngle() { return pnp_info_.pitch_angle; }
  /**
   * @brief 返回深度
   *
   * @return float
   * @author XX
   */
  inline float  returnDepth()      { return pnp_info_.depth; }
  /**
   * @brief 返回目标旋转向量
   *
   * @return double
   * @author XX
   */
  inline double returnTvecTx()     { return tvec_.ptr<double>(0)[0]; }
  inline double returnTvecTy()     { return tvec_.ptr<double>(0)[1]; }
  inline double returnTvecTz()     { return tvec_.ptr<double>(0)[2]; }
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标旋转矩形
   * @author XX
   */
  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::Mat&        _src_img,
                cv::RotatedRect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标外接矩形
   * @author XX
   */
  void solvePnP(int      _ballet_speed,
                int      _armor_type,
                cv::Mat& _src_img,
                cv::Rect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _width        目标实际宽度
   * @param _height       目标实际高度
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标旋转矩形
   * @author XX
   */
  void solvePnP(int             _ballet_speed,
                int             _width,
                int             _height,
                cv::Mat&        _src_img,
                cv::RotatedRect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _width        目标实际宽度
   * @param _height       目标实际高度
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _rect         目标外接矩形
   * @author XX
   */
  void solvePnP(int      _ballet_speed,
                int      _width,
                int      _height,
                cv::Mat& _src_img,
                cv::Rect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _rect         目标旋转矩形
   * @author XX
   */
  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::RotatedRect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _width        目标实际宽度
   * @param _height       目标实际高度
   * @param _rect         目标外接矩形
   * @author XX
   */
  void solvePnP(int             _ballet_speed,
                int             _width,
                int             _height,
                cv::RotatedRect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _rect         目标外接矩形
   * @author XX
   */
  void solvePnP(int      _ballet_speed,
                int      _armor_type,
                cv::Rect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _width        目标实际宽度
   * @param _height       目标实际高度
   * @param _rect         目标外接矩形
   * @author XX
   */
  void solvePnP(int      _ballet_speed,
                int      _width,
                int      _height,
                cv::Rect _rect);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _rect         目标旋转矩形
   * @param _depth        目标深度
   * @author XX
   */
  void solvePnP(int             _ballet_speed,
                int             _armor_type,
                cv::RotatedRect _rect,
                int             _depth);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _target_2d    目标 2d 点坐标
   * @param _depth        目标深度
   * @author XX
   */
  void solvePnP(int                      _ballet_speed,
                int                      _armor_type,
                std::vector<cv::Point2f> _target_2d,
                int                      _depth);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _src_img      原图（ CV_8UC3 ）
   * @param _target_2d    目标 2d 点坐标
   * @author XX
   */
  void solvePnP(int                      _ballet_speed,
                int                      _armor_type,
                cv::Mat&                 _src_img,
                std::vector<cv::Point2f> _target_2d);
  /**
   * @brief 角度结算
   *
   * @param _ballet_speed 子弹速度
   * @param _armor_type   装甲板类型
   * @param _target_2d    目标 2d 点坐标
   * @author XX
   */
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
