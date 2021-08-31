/**
 * @file abstract_object.hpp
 * @author WCJ (1767851382@qq.com)
 * @brief 目标基类
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <vector>

namespace abstract_object {
// 扇叶状态
enum ObjectType { UNKNOW, INACTION, ACTION };

// 目标类（基类）
class Object {
 public:
  /**
   * @brief Construct a new Object object
   */
  Object() {
    rect_         = cv::RotatedRect();
    width_        = 0.f;
    height_       = 0.f;
    index_        = 0;
    angle_        = 0.f;
    aspect_ratio_ = 0.f;
    area_         = 0.f;
    rect_.points(vertex_);
  }

  /**
   * @brief Destroy the Object object
   */
  ~Object() = default;

  /**
   * @brief 返回矩形
   * @return RotatedRect
   */
  cv::RotatedRect Rect();

  /**
   * @brief 返回宽度
   * @return float
   */
  float Width();

  /**
   * @brief 返回高度
   * @return float
   */
  float Height();

  /**
   * @brief 返回索引号
   * @return size_t
   */
  size_t Index();

  /**
   * @brief 返回角度
   * @return float
   */
  float Angle();

  /**
   * @brief 返回宽高比
   * @return float
   */
  float aspectRatio();

  /**
   * @brief 返回轮廓面积
   * @return float
   */
  float Area();

  /**
   * @brief 返回顶点点集
   * @return Point2f
   */
  cv::Point2f* Vertex();

  /**
   * @brief 返回特定顶点
   * @param  _i               顶点编号
   * @return Point2f
   */
  cv::Point2f Vertex(const int& _i);

 protected:
  /**
   * @brief 输入参数
   * @param[in]  _contours        轮廓点集
   */
  void inputParams(const std::vector<cv::Point>& _contours);

 protected:
  // 旋转矩形
  cv::RotatedRect rect_;
  // 宽
  float width_;
  // 高
  float height_;
  // 索引号
  size_t index_;
  // 角度
  float angle_;
  // 宽高比
  float aspect_ratio_;
  // 轮廓面积
  float area_;
  // 顶点点集
  cv::Point2f vertex_[4];
};

void Object::inputParams(const std::vector<cv::Point>& _contours) {
  // 矩形
  this->rect_ = cv::fitEllipse(_contours);
  // 长宽
  if (this->rect_.size.width > this->rect_.size.height) {
    this->width_  = this->rect_.size.width;
    this->height_ = this->rect_.size.height;
  } else {
    this->width_  = this->rect_.size.height;
    this->height_ = this->rect_.size.width;
  }
  // 长宽比
  if (this->height_ != 0) {
    this->aspect_ratio_ = this->width_ / this->height_;
  } else {
    this->aspect_ratio_ = 0.f;
  }
  // 顶点
  this->rect_.points(this->vertex_);
  // 角度
  this->angle_ = this->rect_.angle;
  // 面积
  this->area_ = cv::contourArea(_contours);
}

inline cv::RotatedRect Object::Rect() { return rect_; }

inline float Object::Width() { return width_; }

inline float Object::Height() { return height_; }

inline size_t Object::Index() { return index_; }

inline float Object::Angle() { return angle_; }

inline float Object::aspectRatio() { return aspect_ratio_; }

inline float Object::Area() { return area_; }

inline cv::Point2f* Object::Vertex() { return vertex_; }

inline cv::Point2f Object::Vertex(const int& _i) { return vertex_[_i]; }

float centerDistance(const cv::Point& p1, const cv::Point& p2) {
  float D = static_cast<float>(sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y))));
  return D;
}

int getRectIntensity(const cv::Mat& frame, cv::Rect rect) {
  if (rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1 || rect.width + rect.x > frame.cols || rect.height + rect.y > frame.rows) return 255;
  cv::Mat roi = frame(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
  //    imshow("roi ", roi);
  int average_intensity = static_cast<int>(cv::mean(roi).val[0]);
  roi.release();
  return average_intensity;
}

}  // namespace abstract_object
