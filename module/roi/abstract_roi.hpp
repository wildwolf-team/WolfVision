/**
 * @file abstract_roi.hpp
 * @author RCXXX
 * @brief ROI基类
 * @date 2021-08-27
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace abstract_roi {

class RoI {
 public:
  RoI() : tl_(cv::Point2d(0, 0)) {}

  virtual ~RoI() {}
  /**
   * @brief 截取 ROI 图像
   *
   * @param _input_img 需要截取的图像
   * @param _rect      需要截取的位置(外接矩形)
   * @return cv::Mat   返回截取后的图片
   * @author RCX
   */
  cv::Mat cutRoIRect(const cv::Mat&   _input_img,
                     const cv::Rect&  _rect) {
    tl_ = _rect.tl();
    _input_img(_rect).copyTo(roi_img_);

    return roi_img_;
  }
  /**
   * @brief 截取 ROI 图像
   *
   * @param _input_img 需要截取的图像
   * @param _rect      需要截取的位置 (旋转矩形)
   * @return cv::Mat   返回截取后的图片
   * @author RCX
   */
  cv::Mat cutRoIRotatedRect(const cv::Mat&         _input_img,
                            const cv::RotatedRect& _rect) {
    int roi_w = MAX(_rect.size.width, _rect.size.height);
    int roi_h = MIN(_rect.size.width, _rect.size.height);
    cv::RotatedRect r_rect =
        cv::RotatedRect(_rect.center, cv::Size(roi_w, roi_h), _rect.angle);

    cv::Point2f verices[4];
    r_rect.points(verices);

    cv::Point2f verdst[4];
    verdst[0] = cv::Point2f(0, roi_h);
    verdst[1] = cv::Point2f(0, 0);
    verdst[2] = cv::Point2f(roi_w, 0);
    verdst[3] = cv::Point2f(roi_w, roi_h);

    cv::Mat roi_img_r_rect = cv::Mat(roi_h, roi_w, CV_8UC1);
    cv::Mat warpMatrix     = cv::getPerspectiveTransform(verices, verdst);

    cv::warpPerspective(_input_img,
                        roi_img_r_rect,
                        warpMatrix,
                        roi_img_r_rect.size(),
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT);

    return roi_img_r_rect;
  }
  /**
   * @brief 原点转换
   *
   * @param _input_point ROI 图像的点
   * @return cv::Point2d 返回原图的点
   * @author RCX
   */
  inline cv::Point2d coordMapping(const cv::Point& _input_point) {
    return cv::Point(_input_point.x + tl_.x,
                     _input_point.y + tl_.y);
  }
  /**
   * @brief Rect 坐标转换
   *
   * @param _input_rect ROI 图像的值
   * @return cv::Rect   返回原图的值
   * @author RCX
   */
  inline cv::Rect rectMapping(const cv::Rect& _input_rect) {
    cv::Point convert_tl = coordMapping(_input_rect.tl());

    return cv::Rect(convert_tl.x,
                    convert_tl.y,
                    _input_rect.size().width,
                    _input_rect.size().height);
  }
  /**
   * @brief RotateRect 坐标转换
   *
   * @param _input_r_rect    ROI 图像的值
   * @return cv::RotatedRect 返回原图的值
   * @author RCX
   */
  inline cv::RotatedRect rotatedRectMapping(const cv::RotatedRect& _input_r_rect) {
    return cv::RotatedRect(coordMapping(_input_r_rect.center),
                           _input_r_rect.size,
                           _input_r_rect.angle);
  }

 protected:
  cv::Mat     roi_img_;
  cv::Point2d tl_;
};

}  // namespace abstract_roi
