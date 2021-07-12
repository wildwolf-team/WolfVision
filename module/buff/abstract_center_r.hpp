#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "abstract_object.hpp"

namespace abstract_center_r {

class Center_R : public abstract_object::Object {
 public:
  void inputParams(const std::vector<cv::Point>& _contours, const cv::Mat& _roi_img) {
    abstract_object::Object::inputParams(_contours);

    center_distance = abstract_object::centerDistance(
      rect_.center, cv::Point(_roi_img.cols * 0.5, _roi_img.rows * 0.5));
  }

  float centerDist() { return center_distance; }

 private:
  float center_distance;
};

}  // namespace abstract_center_r
