#pragma once

#include "abstract_roi.hpp"

namespace basic_roi {

struct Roi_Armor {
  cv::RotatedRect last_roi_armor_rect;
  bool last_armor_success = false;
  cv::Rect last_rect = cv::Rect(0, 0, 0, 0);
  int lost_count = 0;
};

class RoI : public abstract_roi::RoI {
 private:
  Roi_Armor roi_armor_data_;
  int first = 0;
  int width_big_num = 18;
  int height_big_num = 12;

 public:
  inline cv::Point returnRectTl() { return roi_armor_data_.last_rect.tl(); }
  inline void setLastRoiSuccess(int _num) {
    roi_armor_data_.last_armor_success = false;
    if (_num != 0) {
      roi_armor_data_.last_armor_success = true;
    }
  }
  inline void setLastRoiRect(cv::RotatedRect _rect, int _ArmorDistinguish) {
    if (_ArmorDistinguish == 0) {
      roi_armor_data_.last_roi_armor_rect.center = _rect.center;
      roi_armor_data_.last_roi_armor_rect.size.width = _rect.size.width * 4;
      roi_armor_data_.last_roi_armor_rect.size.height = _rect.size.height * 4;
      roi_armor_data_.last_roi_armor_rect.angle = _rect.angle;
    } else {
      roi_armor_data_.last_roi_armor_rect.center = _rect.center;
      roi_armor_data_.last_roi_armor_rect.size.width = _rect.size.width * 2;
      roi_armor_data_.last_roi_armor_rect.size.height = _rect.size.height * 4;
      roi_armor_data_.last_roi_armor_rect.angle = _rect.angle;
    }
  }
  inline void BigLastRoiRect(cv::RotatedRect _rect, float width_big_num,
                             float height_big_num) {
    roi_armor_data_.last_roi_armor_rect.size.width =
        _rect.size.width * width_big_num;
    roi_armor_data_.last_roi_armor_rect.size.height =
        _rect.size.height * height_big_num;
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
