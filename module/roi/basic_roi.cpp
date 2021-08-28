/**
 * @file basic_roi.cpp
 * @author RCXXX
 * @brief ROI
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#include "basic_roi.hpp"

namespace basic_roi {

cv::Rect RoI::makeRectSafeFixed(const cv::Mat&         _input_img,
                                const cv::RotatedRect& _r_rect) {
  int width  = _r_rect.boundingRect().width;
  int height = _r_rect.boundingRect().height;

  cv::Point tl = cv::Point(_r_rect.center.x - (width * 0.5f),
                           _r_rect.center.y - (height * 0.5f));
  // 限制 ROI 出界条件
  if (tl.x < 0) {
    tl.x = 0;
  }
  if (tl.y < 0) {
    tl.y = 0;
  }

  if (tl.x + width > _input_img.cols) {
    tl.x -= tl.x + width - _input_img.cols;
  }
  if (tl.y + height > _input_img.rows) {
    tl.y -= tl.y + height - _input_img.rows;
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

cv::Rect RoI::makeRectSafeTailor(const cv::Mat&         _input_img,
                                 const cv::RotatedRect& _r_rect) {
  int width  = _r_rect.boundingRect().width;
  int height = _r_rect.boundingRect().height;

  cv::Point tl = cv::Point(_r_rect.center.x - (width * 0.5f),
                           _r_rect.center.y - (height * 0.5f));
  // 限制 ROI 出界条件
  if (tl.x < 0) {
    width -= 0 - tl.x;
    tl.x   = 0;
  }
  if (tl.y < 0) {
    height -= 0 - tl.y;
    tl.y    = 0;
  }

  if (tl.x + width > _input_img.cols) {
    width -= tl.x + width - _input_img.cols;
  }
  if (tl.y + height > _input_img.rows) {
    height -= tl.y + height - _input_img.rows;
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

cv::Rect RoI::makeRectSafeTailor(const cv::Mat&  _input_img,
                                 const cv::Rect& _r_rect) {
  int width  = _r_rect.width;
  int height = _r_rect.height;

  cv::Point tl = _r_rect.tl();
  // 限制 ROI 出界条件
  if (tl.x < 0) {
    width -= 0 - tl.x;
    tl.x   = 0;
  }
  if (tl.y < 0) {
    height -= 0 - tl.y;
    tl.y    = 0;
  }

  if (tl.x + width > _input_img.cols) {
    width -= (tl.x + width - _input_img.cols);
  }
  if (tl.y + height > _input_img.rows) {
    height -= (tl.y + height - _input_img.rows);
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

cv::Rect RoI::makeRectSafeThird(const cv::Mat&         _input_img,
                                const cv::RotatedRect& _r_rect) {
  int width = _r_rect.boundingRect().width;
  int height = _r_rect.boundingRect().height;

  cv::Point tl = cv::Point(_r_rect.center.x - (width * 0.5f),
                           _r_rect.center.y - (height * 0.5f));
  // 限制 ROI 出界条件
  if (tl.x < 0) {
    tl.x = 0;
  }
  if (tl.y < 0) {
    tl.y = 0;
  }

  if (tl.x + width > _input_img.cols) {
    width = _input_img.cols - tl.x;
  }
  if (tl.y + height > _input_img.rows) {
    height = _input_img.rows - tl.y;
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

cv::Mat RoI::returnROIResultMat(const cv::Mat& _input_img) {
  if (!roi_armor_data_.last_armor_success) {
    roi_armor_data_.lost_count++;
    // 第一帧不进入
    if (first == 0) {
      first++;
      roi_armor_data_.last_rect = cv::Rect(0, 0, 0, 0);
      return _input_img;
    }
    // 根据设置次数调整 ROI 范围 （逐级扩大）
    if (roi_armor_data_.lost_count < 10) {
      BigLastRoiRect(roi_armor_data_.last_roi_armor_rect, width_big_num * 0.1,
                     height_big_num * 0.1);
      roi_armor_data_.last_rect =
          makeRectSafeThird(_input_img, roi_armor_data_.last_roi_armor_rect);
      return _input_img(roi_armor_data_.last_rect);
    } else if (roi_armor_data_.lost_count < 15) {
      BigLastRoiRect(roi_armor_data_.last_roi_armor_rect, width_big_num * 0.1,
                     height_big_num * 0.1);
      roi_armor_data_.last_rect =
          makeRectSafeThird(_input_img, roi_armor_data_.last_roi_armor_rect);
      return _input_img(roi_armor_data_.last_rect);
    } else if (roi_armor_data_.lost_count < 20) {
      BigLastRoiRect(roi_armor_data_.last_roi_armor_rect, width_big_num * 0.1,
                     height_big_num * 0.1);
      roi_armor_data_.last_rect =
          makeRectSafeThird(_input_img, roi_armor_data_.last_roi_armor_rect);
      return _input_img(roi_armor_data_.last_rect);
    } else if (roi_armor_data_.lost_count < 25) {
      BigLastRoiRect(roi_armor_data_.last_roi_armor_rect, width_big_num * 0.1,
                     height_big_num * 0.1);
      roi_armor_data_.last_rect =
          makeRectSafeThird(_input_img, roi_armor_data_.last_roi_armor_rect);
      return _input_img(roi_armor_data_.last_rect);
    } else {
      roi_armor_data_.last_rect = cv::Rect(0, 0, 0, 0);
      return _input_img;
    }
  } else {
    // 丢失清零
    roi_armor_data_.lost_count = 0;
    // 更新上一帧 ROI 参数
    roi_armor_data_.last_rect =
        makeRectSafeThird(_input_img, roi_armor_data_.last_roi_armor_rect);
    return _input_img(roi_armor_data_.last_rect);
  }
}
}  // namespace basic_roi
