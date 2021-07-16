#pragma once

#include "abstract_roi.hpp"

namespace basic_roi {

struct Roi_Armor {
  cv::RotatedRect last_roi_armor_rect;
  bool last_armor_success = false;
  cv::Rect last_rect_ = cv::Rect(0, 0, 0, 0);
};

class RoI : public abstract_roi::RoI {
 private:
  Roi_Armor roi_armor_data_;

 public:
  inline cv::Point returnRectTl() { return roi_armor_data_.last_rect_.tl(); }
  inline void setLastRoiSuccess(int _num) {
    roi_armor_data_.last_armor_success = false;
    if (_num != 0) {
      roi_armor_data_.last_armor_success = true;
    }
  }
  inline void setLastRoiRect(cv::RotatedRect _rect) {
    roi_armor_data_.last_roi_armor_rect.center = _rect.center;
    roi_armor_data_.last_roi_armor_rect.size.width = _rect.size.width * 2;
    roi_armor_data_.last_roi_armor_rect.size.height = _rect.size.height * 2;
    roi_armor_data_.last_roi_armor_rect.angle = _rect.angle;
  }

  RoI() = default;

  ~RoI() = default;

  cv::Rect makeRectSafeFixed(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);

  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::RotatedRect& _r_rect);

  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::Rect& _r_rect);

  cv::Rect makeRectSafeThird(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);

  cv::Mat returnROIResultMat(const cv::Mat& _input_img);
};

}  // namespace basic_roi
