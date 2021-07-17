#include "fan_armor.hpp"

namespace fan_armor {

void Detector::inputParams(const std::vector<cv::Point>& _contours) { abstract_object::Object::inputParams(_contours); }

void Detector::displayFanArmor(cv::Mat& _img) {
  cv::Scalar color         = cv::Scalar(0, 255, 0);
  cv::Point  put_fan_armor = cv::Point(_img.cols - 100, 10);

  cv::putText(_img, " FanArmor ", put_fan_armor, cv::FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);

  for (size_t i = 0; i != 4; ++i) {
    cv::line(_img, vertex_[i], vertex_[(i + 1) % 4], color, 2);
  }
}

}  // namespace fan_armor
