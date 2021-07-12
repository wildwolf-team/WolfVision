#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "abstract_object.hpp"

namespace abstract_blade {

class FanBlade : public abstract_object::Object {
 public:
  FanBlade() { length_ = 0; }

  ~FanBlade() = default;

  void inputParams(const std::vector<cv::Point>& _contours) {
    abstract_object::Object::inputParams(_contours);
  }

  void displayFanBlade(cv::Mat& _img) {
    cv::Scalar color         = cv::Scalar(0, 255, 255);
    cv::Point  put_fan_blade = cv::Point(_img.cols - 100, 30);

    cv::putText(_img, " FanBlade ", put_fan_blade, cv::FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);

    for (size_t i = 0; i != 4; ++i) {
      cv::line(_img, vertex_[i], vertex_[(i + 1) % 4], color, 4);
    }
  }

  inline int getLength() { return length_; }

 private:
  int length_;
};

}  // namespace abstract_blade
