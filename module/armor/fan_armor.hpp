#pragma once

#include <vector>

#include "module/buff/abstract_object.hpp"

namespace fan_armor {

class Detector : public abstract_object::Object {
 public:
  Detector() = default;

  ~Detector() = default;

  /**
   * @brief 输入参数
   * @param[in]  _contours        输入轮廓点集
   */
  void inputParams(const std::vector<cv::Point>& _contours);

  /**
   * @brief 显示内轮廓
   * @param[out]  _img     输出图像
   * @note 绿色
   * @note 图例显示在右侧
   */
  void displayFanArmor(cv::Mat& _img);
};

}  // namespace fan_armor
