#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

namespace abstract_object {

enum ObjectType {
  UNKNOW,
  INACTION,
  ACTION
};

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

  cv::RotatedRect getRect()                { return rect_; }
  cv::Point2f*    getVertex()              { return vertex_; }
  cv::Point2f     getVertex(const int& _i) { return vertex_[_i]; }

  size_t          getIndex()               { return index_; }

  float           getWidth()               { return width_; }
  float           getHeight()              { return height_; }
  float           getAngle()               { return angle_; }
  float           aspectRatio()            { return aspect_ratio_; }
  float           getArea()                { return area_; }

 protected:
  void inputParams(const std::vector<cv::Point>& _contours) {
    rect_ = cv::fitEllipse(_contours);

    if (rect_.size.width > rect_.size.height) {
      width_  = rect_.size.width;
      height_ = rect_.size.height;
    } else {
      width_  = rect_.size.height;
      height_ = rect_.size.width;
    }

    if (height_ != 0) {
      aspect_ratio_ = width_ / height_;
    } else {
      aspect_ratio_ = 0.f;
    }

    rect_.points(vertex_);

    angle_ = rect_.angle;
    area_  = cv::contourArea(_contours);
}

 protected:
  cv::RotatedRect rect_;
  cv::Point2f     vertex_[4];

  size_t index_;

  float  width_;
  float  height_;
  float  angle_;
  float  aspect_ratio_;
  float  area_;
};

float centerDistance(const cv::Point& p1, const cv::Point& p2) {
  float D =
      static_cast<float>(sqrt(((p1.x - p2.x) * (p1.x - p2.x)) +
                              ((p1.y - p2.y) * (p1.y - p2.y))));

  return D;
}

int getRectIntensity(const cv::Mat& frame, cv::Rect rect) {
  if (rect.width  < 1                   ||
      rect.height < 1                   ||
      rect.x < 1                        ||
      rect.y < 1                        ||
      rect.width  + rect.x > frame.cols ||
      rect.height + rect.y > frame.rows) {
    return 255;
  }

  cv::Mat roi           = frame(cv::Range(rect.y, rect.y + rect.height),
                                cv::Range(rect.x, rect.x + rect.width));
  int average_intensity = static_cast<int>(cv::mean(roi).val[0]);

  roi.release();

  return average_intensity;
}

}  // namespace abstract_object
