/**
 * @file abstract_center_r.hpp
 * @author WCJ (1767851382@qq.com)
 * @brief 圆心 R 类
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "abstract_object.hpp"

namespace abstract_center_r {
class Center_R : public abstract_object::Object {
 public:
  /**
   * @brief 输入参数
   * @param[in]  _contours        轮廓点集
   * @param[in]  _roi_img         圆形区域ROI
   */
  void inputParams(const std::vector<cv::Point>& _contours, const cv::Mat& _roi_img);

  /**
   * @brief 返回目标距离ROI图像中心点的距离
   * @return float
   */
  float centerDist();

 private:
  // 中心距离
  float center_distance;
};

void Center_R::inputParams(const std::vector<cv::Point>& _contours, const cv::Mat& _roi_img) {
  abstract_object::Object::inputParams(_contours);

  // 中心距离
  this->center_distance = abstract_object::centerDistance(this->rect_.center, cv::Point(_roi_img.cols * 0.5, _roi_img.rows * 0.5));
}

inline float Center_R::centerDist() { return this->center_distance; }

}  // namespace abstract_center_r
