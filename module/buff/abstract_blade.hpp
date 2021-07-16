#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "abstract_object.hpp"

namespace abstract_blade {

class FanBlade : public abstract_object::Object {
 public:
  FanBlade() { length_ = 0; }

  ~FanBlade() = default;

  /**
   * @brief 输入外轮廓参数
   * @param[in]  _contours        外轮廓点集
   */
  void inputParams(const std::vector<cv::Point>& _contours) { abstract_object::Object::inputParams(_contours); }

  /**
   * @brief 显示外轮廓
   * @param[out]  _img         输出图像
   * @note 黄色
   * @note 图例显示在右侧
   */
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
  int length_;  // 周长
};

}  // namespace abstract_blade
