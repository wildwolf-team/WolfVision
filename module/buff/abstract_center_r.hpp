#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "abstract_object.hpp"

namespace abstract_center_r {

class Center_R : public abstract_object::Object {
 public:
  Center_R()  = default;
  ~Center_R() = default;
  /**
   * @brief 输入参数
   * @param[in]  _contours        轮廓点集
   * @param[in]  _roi_img         圆形区域ROI
   */
  void inputParams(const std::vector<cv::Point>& _contours, const cv::Mat& _roi_img) {
    abstract_object::Object::inputParams(_contours);

    center_distance = abstract_object::centerDistance(rect_.center, cv::Point(_roi_img.cols * 0.5, _roi_img.rows * 0.5));
  }

  float centerDist() { return center_distance; }

 private:
  float center_distance;  // 中心距离
};

}  // namespace abstract_center_r
