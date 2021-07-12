#pragma once

#include <vector>

#include "abstract_blade.hpp"
#include "fan_armor.hpp"

namespace abstract_target {

class Target {
 public:
  Target() {
    fan_blade_          = abstract_blade::FanBlade();
    fan_armor_          = fan_armor::Detector();
    type_               = abstract_object::UNKNOW;
    angle_              = 0.f;
    diff_angle_         = 0.f;
    area_ratio_         = 0.f;
    delta_height_point_ = cv::Point2f(0.f, 0.f);

    points_2d_.reserve(4);
  }

  ~Target() = default;

  void inputParams(const abstract_blade::FanBlade& _fan_blade,
                   const fan_armor::Detector&      _fan_armor) {
    fan_armor_  = _fan_armor;
    fan_blade_  = _fan_blade;
    diff_angle_ = fabsf(fan_armor_.getAngle() - fan_blade_.getAngle());
    angle_      = atan2((fan_armor_.getRect().center.y - fan_blade_.getRect().center.y),
                        (fan_armor_.getRect().center.x - fan_blade_.getRect().center.x)) *
                  180                                                                    /
                  static_cast<float>(CV_PI);
    type_       = abstract_object::UNKNOW;

    if (angle_ < 0.f) { angle_ += 360.f; }

    area_ratio_ = fan_armor_.getRect().size.area() / fan_blade_.getRect().size.area();
  }

  void updateVertex(cv::Mat& _img) {
    points_2d_.clear();

    cv::Point2f point_up_center   = (fan_armor_.getVertex(0) + fan_armor_.getVertex(1)) * 0.5;
    cv::Point2f point_down_center = (fan_armor_.getVertex(2) + fan_armor_.getVertex(3)) * 0.5;

    float up_distance   =
        abstract_object::centerDistance(point_up_center, fan_blade_.getRect().center);
    float down_distance =
        abstract_object::centerDistance(point_down_center, fan_blade_.getRect().center);

    if (up_distance > down_distance) {
      points_2d_.emplace_back(fan_armor_.getVertex(0));
      points_2d_.emplace_back(fan_armor_.getVertex(1));
      points_2d_.emplace_back(fan_armor_.getVertex(2));
      points_2d_.emplace_back(fan_armor_.getVertex(3));
    } else {
      points_2d_.emplace_back(fan_armor_.getVertex(2));
      points_2d_.emplace_back(fan_armor_.getVertex(3));
      points_2d_.emplace_back(fan_armor_.getVertex(0));
      points_2d_.emplace_back(fan_armor_.getVertex(1));
    }

    cv::circle(_img, points_2d_[0], 5, cv::Scalar(0, 0, 255),   -1);
    cv::circle(_img, points_2d_[1], 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(_img, points_2d_[2], 5, cv::Scalar(255, 155, 0), -1);
    cv::circle(_img, points_2d_[3], 5, cv::Scalar(0, 255, 0),   -1);
  }

  void setType(const abstract_object::ObjectType& _type) { type_ = _type; }

  void setType(cv::Mat& _bin_img) {
    cv::Point2f point_up_center   = (fan_armor_.getVertex(0) + fan_armor_.getVertex(1)) * 0.5;
    cv::Point2f point_down_center = (fan_armor_.getVertex(2) + fan_armor_.getVertex(3)) * 0.5;

    float up_distance   =
        abstract_object::centerDistance(point_up_center, fan_blade_.getRect().center);
    float down_distance =
        abstract_object::centerDistance(point_down_center, fan_blade_.getRect().center);

    cv::Point2f vector_height;

    if (up_distance > down_distance) {
      vector_height = fan_armor_.getVertex(0) - fan_armor_.getVertex(3);
    } else {
      vector_height = fan_armor_.getVertex(3) - fan_armor_.getVertex(0);
    }

    cv::Point left_center  = vector2DPoint(3) - vector_height;
    cv::Point right_center = vector2DPoint(2) - vector_height;

    int width  = 10;
    int height = 5;

    cv::Point left1  = cv::Point(left_center.x - width, left_center.y - height);
    cv::Point left2  = cv::Point(left_center.x + width, left_center.y + height);

    cv::Point right1 = cv::Point(right_center.x - width, right_center.y - height);
    cv::Point right2 = cv::Point(right_center.x + width, right_center.y + height);

    cv::Rect left_rect(left1, left2);
    cv::Rect right_rect(right1, right2);

    int left_intensity  = abstract_object::getRectIntensity(_bin_img, left_rect);
    int right_intensity = abstract_object::getRectIntensity(_bin_img, right_rect);

    if (left_intensity <= 15 && right_intensity <= 15) {
      type_ = abstract_object::INACTION;
    } else {
      type_ = abstract_object::ACTION;
    }
  }

  void displayInactionTarget(cv::Mat& _img) {
    if (type_ == abstract_object::INACTION) {
      for (size_t i = 0; i != 4; ++i) {
        cv::line(_img, fan_armor_.getVertex(i), fan_armor_.getVertex((i + 1) % 4),
                cv::Scalar(0, 255, 0),   2);
        cv::line(_img, fan_blade_.getVertex(i), fan_blade_.getVertex((i + 1) % 4),
                cv::Scalar(0, 255, 255), 2);
      }
    }
  }

  abstract_blade::FanBlade    getBlade()      { return fan_blade_; }
  fan_armor::Detector         getArmor()      { return fan_armor_; }

  abstract_object::ObjectType getType()       { return type_; }

  float                       getAngle()      { return angle_; }
  float                       diffAngle()     { return diff_angle_; }
  double                      areaRatio()     { return area_ratio_; }

  std::vector<cv::Point2f>    vector2DPoint()              { return points_2d_; }

  cv::Point2f                 vector2DPoint(const int& _i) { return points_2d_[_i]; }
  cv::Point2f                 deltaPoint()                 { return delta_height_point_; }

 private:
  abstract_blade::FanBlade    fan_blade_;
  fan_armor::Detector         fan_armor_;

  abstract_object::ObjectType type_;

  float                       angle_;
  float                       diff_angle_;
  double                      area_ratio_;

  std::vector<cv::Point2f>    points_2d_;
  cv::Point2f                 delta_height_point_;
};

}  // namespace abstract_target
