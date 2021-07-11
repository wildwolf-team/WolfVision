#pragma once

#include "abstract_roi.hpp"

namespace basic_roi {

class ROI : public abstract_roi::ROI {
 public:
  ROI() = default;

  ~ROI() = default;

  cv::Rect makeRectSafeFixed(const cv::Mat&         _input_img,
                             const cv::RotatedRect& _r_rect);

  cv::Rect makeRectSafeTailor(const cv::Mat&         _input_img,
                              const cv::RotatedRect& _r_rect);

  cv::Rect makeRectSafeTailor(const cv::Mat&  _input_img,
                              const cv::Rect& _r_rect);
};

}  // namespace basic_roi
