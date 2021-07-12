#pragma once

#include <vector>

#include "module/buff/abstract_object.hpp"

namespace fan_armor {

class Detector : public abstract_object::Object {
 public:
  Detector() = default;

  ~Detector() = default;

  void inputParams(const std::vector<cv::Point>& _contours);

  void displayFanArmor(cv::Mat& _img);
};

}  // namespace fan_armor
