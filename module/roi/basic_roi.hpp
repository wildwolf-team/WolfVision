#pragma once

#include "abstract_roi.hpp"

namespace basic_roi {
class ImageRoi : public abstract_roi::Abstract_ImageRoi {
 public:
  ImageRoi() {}
  ~ImageRoi() {}

  cv::Rect makeRectSafe_Fixed(const cv::Mat &_input_img,
                              const cv::RotatedRect &_r_rect);

  cv::Rect makeRectSafe_Tailor(const cv::Mat &_input_img,
                               const cv::RotatedRect &_r_rect);

  cv::Rect makeRectSafe_Tailor(const cv::Mat &_input_img,
                               const cv::Rect &_r_rect);
};

}  // namespace basic_roi
