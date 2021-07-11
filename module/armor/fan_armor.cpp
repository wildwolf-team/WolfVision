#include "fan_armor.hpp"

#include <vector>

namespace fan_armor {
FanArmor::FanArmor() {}

FanArmor::~FanArmor() {}

void FanArmor::inputParams(const std::vector<cv::Point>& _contours) {
  abstract_object::Object::inputParams(_contours);
}

void FanArmor::displayFanArmor(cv::Mat& _img) {
  cv::Scalar color = cv::Scalar(0, 255, 0);

  cv::Point put_fan_armor = cv::Point(_img.cols - 100, 10);
  cv::putText(_img, " Fan Armor ", put_fan_armor, cv::FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);

  for (int k = 0; k < 4; ++k) {
    cv::line(_img, this->vertex_[k], this->vertex_[(k + 1) % 4], color, 2);
  }
}
}  // namespace fan_armor
