#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace abstract_object {

enum ObjectType { UNKNOW, INACTION, ACTION };

class Object {
 public:
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

  ~Object() = default;

  cv::RotatedRect getRect() { return rect_; }
  cv::Point2f*    getVertex() { return vertex_; }
  cv::Point2f     getVertex(const int& _i) { return vertex_[_i]; }

  size_t getIndex() { return index_; }

  float getWidth() { return width_; }
  float getHeight() { return height_; }
  float getAngle() { return angle_; }
  float aspectRatio() { return aspect_ratio_; }
  float getArea() { return area_; }

 protected:
  /**
   * @brief 输入参数
   * @param[in]  _contours        轮廓点集
   */
  void inputParams(const std::vector<cv::Point>& _contours) {
    // 矩形
    rect_ = cv::fitEllipse(_contours);

    // 长宽
    if (rect_.size.width > rect_.size.height) {
      width_  = rect_.size.width;
      height_ = rect_.size.height;
    } else {
      width_  = rect_.size.height;
      height_ = rect_.size.width;
    }

    // 长宽比
    if (height_ != 0) {
      aspect_ratio_ = width_ / height_;
    } else {
      aspect_ratio_ = 0.f;
    }

    // 顶点
    rect_.points(vertex_);

    // 角度
    angle_ = rect_.angle;

    // 面积
    area_ = cv::contourArea(_contours);
  }

 protected:
  cv::RotatedRect rect_;       // 旋转矩形
  cv::Point2f     vertex_[4];  // 顶点点集

  size_t index_;  // 索引号

  float width_;         // 宽
  float height_;        // 高
  float angle_;         // 角度
  float aspect_ratio_;  // 宽高比
  float area_;          // 轮廓面积
};

float centerDistance(const cv::Point& p1, const cv::Point& p2) {
  float D = static_cast<float>(sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y))));

  return D;
}

int getRectIntensity(const cv::Mat& frame, cv::Rect rect) {
  if (rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1 || rect.width + rect.x > frame.cols || rect.height + rect.y > frame.rows) {
    return 255;
  }

  cv::Mat roi               = frame(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
  int     average_intensity = static_cast<int>(cv::mean(roi).val[0]);

  roi.release();

  return average_intensity;
}

}  // namespace abstract_object
