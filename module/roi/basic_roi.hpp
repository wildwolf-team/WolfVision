/**
 * @file basic_roi.hpp
 * @author RCXXX
 * @brief ROI
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
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
  /**
   * @brief 返回ROI的tl点
   *
   * @return cv::Point ROI的tl点
   */
  inline cv::Point returnRectTl() { return roi_armor_data_.last_rect.tl(); }
  /**
   * @brief 设置是否进行ROI
   * 
   * @param _num 
   */
  inline void setLastRoiSuccess(int _num) {
    roi_armor_data_.last_armor_success = false;
    // 装甲板数量不为0
    if (_num != 0) {
      roi_armor_data_.last_armor_success = true;
    }
  }
  /**
   * @brief 设置装甲板ROI基础范围
   * 
   * @param _rect 装甲板旋转矩形
   * @param _ArmorDistinguish 装甲板类型
   */
  inline void setLastRoiRect(cv::RotatedRect _rect, int _ArmorDistinguish) {
    if (_ArmorDistinguish == 0) {
      roi_armor_data_.last_roi_armor_rect.center = _rect.center;
      roi_armor_data_.last_roi_armor_rect.size.width = _rect.size.width * 8;
      roi_armor_data_.last_roi_armor_rect.size.height = _rect.size.height * 4;
      roi_armor_data_.last_roi_armor_rect.angle = _rect.angle;
    } else {
      roi_armor_data_.last_roi_armor_rect.center = _rect.center;
      roi_armor_data_.last_roi_armor_rect.size.width = _rect.size.width * 4;
      roi_armor_data_.last_roi_armor_rect.size.height = _rect.size.height * 4;
      roi_armor_data_.last_roi_armor_rect.angle = _rect.angle;
    }
  }
  /**
   * @brief ROI扩大倍数
   * 
   * @param _rect ROI的旋转矩形
   * @param width_big_num 宽度增加的倍数
   * @param height_big_num 高度增加的倍数
   */
  inline void BigLastRoiRect(cv::RotatedRect _rect, float width_big_num,
                             float height_big_num) {
    roi_armor_data_.last_roi_armor_rect.size.width =
        _rect.size.width * width_big_num;
    roi_armor_data_.last_roi_armor_rect.size.height =
        _rect.size.height * height_big_num;
  }

  RoI() = default;

  ~RoI() = default;
  /**
   * @brief 限制ROI范围
   * 
   * @param _input_img 图像
   * @param _r_rect ROI区域
   * @return cv::Rect 
   */
  cv::Rect makeRectSafeFixed(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);
  /**
   * @brief 限制ROI范围
   *
   * @param _input_img 图像
   * @param _r_rect ROI区域
   * @return cv::Rect
   */
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::RotatedRect& _r_rect);
  /**
   * @brief 限制ROI范围
   *
   * @param _input_img 图像
   * @param _r_rect ROI区域
   * @return cv::Rect
   */
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::Rect& _r_rect);
  /**
   * @brief 限制ROI范围
   *
   * @param _input_img 图像
   * @param _r_rect ROI区域
   * @return cv::Rect
   */
  cv::Rect makeRectSafeThird(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);

  /**
   * @brief 逐级扩大ROI范围 
   * 
   * @param _input_img 图像
   * @return cv::Mat ROI图像
   */
  cv::Mat returnROIResultMat(const cv::Mat& _input_img);
};

}  // namespace basic_roi
