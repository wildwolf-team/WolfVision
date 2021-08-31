/**
 * @file abstract_target.hpp
 * @author WCJ (1767851382@qq.com)
 * @brief 目标类
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#pragma once

#include <vector>

#include "abstract_blade.hpp"
#include "fan_armor.hpp"

namespace abstract_target {
class Target {
 public:
  // 构造函数 析构函数
  Target();
  ~Target();

  /**
   * @brief 输入参数
   * @param[in]  _fan_blade       外轮廓（扇叶）
   * @param[in]  _fan_armor       内轮廓（装甲板）
   */
  void inputParams(const abstract_blade::FanBlade& _fan_blade, const fan_armor::Detector& _fan_armor);

  /**
   * @brief 更新装甲板的四个顶点编号
   * @param[in]  _img             输入绘制图像
   * @note 将各个顶点绘制在输入图像上
   */
  void updateVertex(cv::Mat& _img);

  /**
   * @brief 设置运转类型
   * @details 强制
   * @param[in]  _type            类型
   */
  void setType(const abstract_object::ObjectType& _type);

  /**
   * @brief 设置运转类型
   * @param[in]  _bin_img         输入二值图
   * @param[in]  _img             输入绘制图像
   * @todo 待添加新版roi
   */
  void setType(cv::Mat& _bin_img, cv::Mat& _img);

  /**
   * @brief 显示可打击的目标
   * @param[in]  _img             输入绘制图像
   * @note 装甲板为绿框，扇叶为黄框
   */
  void displayInactionTarget(cv::Mat& _img);

  /**
   * @brief 返回扇叶大轮廓
   * @return abstract_blade::FanBlade
   */
  abstract_blade::FanBlade Blade();

  /**
   * @brief 返回装甲板小轮廓
   * @return fan_armor::Detector
   */
  fan_armor::Detector Armor();

  /**
   * @brief 返回目标角度
   * @details 水平线右上为360°，右下为0°，顺时针增长
   * @return float
   */
  float Angle();

  /**
   * @brief 返回角度差
   * @return float
   */
  float diffAngle();

  /**
   * @brief 返回当前目标运转类型
   * @return abstract_object::ObjectType
   */
  abstract_object::ObjectType Type();

  /**
   * @brief 返回面积比例
   * @return double
   */
  double areaRatio();

  /**
   * @brief 返回装甲板排序后的顶点
   * @return vector<Point2f>
   * @todo 留做返回给pnp的接口，或者是留做返回给基类的接口
   */
  std::vector<cv::Point2f> vector2DPoint();

  /**
   * @brief 返回装甲板排序具体顶点
   * @param  _i               顶点编号
   * @return Point2f
   */
  cv::Point2f vector2DPoint(const int& _i);

  /**
   * @brief 返回装甲板高度点差
   * @return Point2f
   * @note 由于是点之间的差值，所以相减的顺序会影响变化的方向，要留意
   */
  cv::Point2f deltaPoint();

 private:
  // 大轮廓
  abstract_blade::FanBlade fan_blade_;
  // 小轮廓
  fan_armor::Detector fan_armor_;

  // 角度（可定坐标系）
  float angle_;
  // 大小轮廓角度差
  float diff_angle_;
  // 运转类型
  abstract_object::ObjectType type_;
  // 面积比例:小/大
  double area_ratio_;
  // 顶点容器
  std::vector<cv::Point2f> points_2d_;
  // 装甲板高度点差
  cv::Point2f delta_height_point_;
};

Target::Target() {
  fan_blade_        = abstract_blade::FanBlade();
  fan_armor_        = fan_armor::Detector();
  angle_      = 0.f;
  diff_angle_ = 0.f;
  type_       = abstract_object::UNKNOW;
  area_ratio_ = 0.f;
  points_2d_.reserve(4);
  delta_height_point_ = cv::Point2f(0.f, 0.f);
}

Target::~Target() {}

void Target::inputParams(const abstract_blade::FanBlade& _fan_blade, const fan_armor::Detector& _fan_armor) {
  // 内轮廓
  fan_armor_ = _fan_armor;

  // 外轮廓
  fan_blade_ = _fan_blade;

  // 内外轮廓角度差
  diff_angle_ = fabsf(fan_armor_.Angle() - fan_blade_.Angle());

  // 扇叶角度= 扇叶和装甲板的连线 TODO:未使用
  angle_ = atan2((fan_armor_.Rect().center.y - fan_blade_.Rect().center.y), (fan_armor_.Rect().center.x - fan_blade_.Rect().center.x)) * 180 / static_cast<float>(CV_PI);
  // 过零处理
  if (angle_ < 0.f) {
    angle_ += 360.f;
  }

  // 设置默认运行模式：未知
  type_ = abstract_object::UNKNOW;

  // 面积比例(方案一)
  area_ratio_ = fan_armor_.Rect().size.area() / fan_blade_.Rect().size.area();

  // 面积比例(方案二)
  // area_ratio_ = fan_armor_.Area() / fan_blade_.Area();
}

void Target::updateVertex(cv::Mat& _img) {
  points_2d_.clear();

  cv::Point2f point_up_center   = (fan_armor_.Vertex(0) + fan_armor_.Vertex(1)) * 0.5;
  cv::Point2f point_down_center = (fan_armor_.Vertex(2) + fan_armor_.Vertex(3)) * 0.5;

  float up_distance   = abstract_object::centerDistance(point_up_center, fan_blade_.Rect().center);
  float down_distance = abstract_object::centerDistance(point_down_center, fan_blade_.Rect().center);

  if (up_distance > down_distance) {
    // 0 1
    // 3 2
    points_2d_.emplace_back(fan_armor_.Vertex(0));
    points_2d_.emplace_back(fan_armor_.Vertex(1));
    points_2d_.emplace_back(fan_armor_.Vertex(2));
    points_2d_.emplace_back(fan_armor_.Vertex(3));
  } else {
    // 2 3
    // 1 0
    points_2d_.emplace_back(fan_armor_.Vertex(2));
    points_2d_.emplace_back(fan_armor_.Vertex(3));
    points_2d_.emplace_back(fan_armor_.Vertex(0));
    points_2d_.emplace_back(fan_armor_.Vertex(1));
  }

  // 画出各个顶点 红黄蓝绿
  cv::circle(_img, points_2d_[0], 5, cv::Scalar(0, 0, 255), -1);
  cv::circle(_img, points_2d_[1], 5, cv::Scalar(0, 255, 255), -1);
  cv::circle(_img, points_2d_[2], 5, cv::Scalar(255, 155, 0), -1);
  cv::circle(_img, points_2d_[3], 5, cv::Scalar(0, 255, 0), -1);
}

void Target::setType(const abstract_object::ObjectType& _type) { type_ = _type; }

void Target::setType(cv::Mat& _bin_img, cv::Mat& _img) {
  // 上层中心点 和 下层中心点
  cv::Point2f point_up_center   = (fan_armor_.Vertex(0) + fan_armor_.Vertex(1)) * 0.5;
  cv::Point2f point_down_center = (fan_armor_.Vertex(2) + fan_armor_.Vertex(3)) * 0.5;

  // 上层离外轮廓中点距离
  // 下册离外轮廓中点距离
  // TODO(fqjun) ：待修改为优化方案，不用重复判断方向
  float up_distance   = abstract_object::centerDistance(point_up_center, fan_blade_.Rect().center);
  float down_distance = abstract_object::centerDistance(point_down_center, fan_blade_.Rect().center);

  // 计算装甲板的高度差，且方向朝向圆心
  cv::Point2f vector_height;
  if (up_distance > down_distance) {
    vector_height = fan_armor_.Vertex(0) - fan_armor_.Vertex(3);
  } else {
    vector_height = fan_armor_.Vertex(3) - fan_armor_.Vertex(0);
  }

  // 这里记得分清楚应该是在哪个点往哪个方向移动多少距离进行框取小ROI
  // 计算两个小roi的中心点
  cv::Point left_center  = vector2DPoint(3) - vector_height;
  cv::Point right_center = vector2DPoint(2) - vector_height;

  // 创建roi并绘制
  int width  = 10;
  int height = 5;

  cv::Point left1 = cv::Point(left_center.x - width, left_center.y - height);
  cv::Point left2 = cv::Point(left_center.x + width, left_center.y + height);

  cv::Point right1 = cv::Point(right_center.x - width, right_center.y - height);
  cv::Point right2 = cv::Point(right_center.x + width, right_center.y + height);

  cv::Rect left_rect(left1, left2);     // 画出左边小roi
  cv::Rect right_rect(right1, right2);  // 画出右边小roi

  // 计算光线强度
  int left_intensity  = abstract_object::getRectIntensity(_bin_img, left_rect);
  int right_intensity = abstract_object::getRectIntensity(_bin_img, right_rect);

  if (left_intensity <= 15 && right_intensity <= 15) {
    type_ = abstract_object::INACTION;
  } else {
    type_ = abstract_object::ACTION;
  }
}

void Target::displayInactionTarget(cv::Mat& _img) {
  if (type_ == abstract_object::INACTION) {
    for (int k = 0; k < 4; ++k) {
      cv::line(_img, fan_armor_.Vertex(k), fan_armor_.Vertex((k + 1) % 4), cv::Scalar(0, 255, 0), 2);
      cv::line(_img, fan_blade_.Vertex(k), fan_blade_.Vertex((k + 1) % 4), cv::Scalar(0, 255, 255), 2);
    }
  }
}

inline abstract_blade::FanBlade Target::Blade() { return fan_blade_; }

inline fan_armor::Detector Target::Armor() { return fan_armor_; }

inline float Target::Angle() { return angle_; }

inline float Target::diffAngle() { return diff_angle_; }

inline abstract_object::ObjectType Target::Type() { return type_; }

inline double Target::areaRatio() { return area_ratio_; }

inline std::vector<cv::Point2f> Target::vector2DPoint() { return points_2d_; }

inline cv::Point2f Target::vector2DPoint(const int& _i) { return points_2d_[_i]; }

inline cv::Point2f Target::deltaPoint() { return points_2d_[0] - points_2d_[3]; }

}  // namespace abstract_target
