#pragma once

#include <vector>

#include "abstract_armor.hpp"
#include "module/buff/abstract_object.hpp"

namespace fan_armor {
class FanArmor : public abstract_object::Object {
 public:
  FanArmor();
  ~FanArmor();

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

 private:
  /* anything else */
};
}  // namespace fan_armor
