/**
 * @file abstract_blade.hpp
 * @author WCJ (1767851382@qq.com)
 * @brief 扇叶目标类
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "abstract_object.hpp"
namespace abstract_blade {

class FanBlade : public abstract_object::Object {
 public:
  FanBlade();
  ~FanBlade();

  /**
   * @brief 输入外轮廓参数
   * @param[in]  _contours        外轮廓点集
   */
  void inputParams(const std::vector<cv::Point>& _contours);

  /**
   * @brief 显示外轮廓
   * @param[out]  _img         输出图像
   * @note 黄色
   * @note 图例显示在右侧
   */
  void displayFanBlade(cv::Mat& _img);

  /**
   * @brief 返回周长
   * @return int
   */
  int Length();

 private:
  // 周长
  int length_;
};

void FanBlade::inputParams(const std::vector<cv::Point>& _contours) { abstract_object::Object::inputParams(_contours); }

FanBlade::FanBlade() { length_ = 0; }

FanBlade::~FanBlade() {}

void FanBlade::displayFanBlade(cv::Mat& _img) {
  cv::Scalar color = cv::Scalar(0, 255, 255);

  cv::Point put_fan_blade = cv::Point(_img.cols - 100, 30);
  cv::putText(_img, " Fan Blade ", put_fan_blade, cv::FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);

  for (int k = 0; k < 4; ++k) {
    cv::line(_img, vertex_[k], vertex_[(k + 1) % 4], color, 4);
  }
}

inline int FanBlade::Length() { return length_; }

}  // namespace abstract_blade
