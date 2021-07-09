#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace abstract_roi {
class Abstract_ImageRoi {
 protected:
  cv::Mat roi_img_;  // 截取后 ROI 区域图像
  cv::Point2d tl_;   // ROI 区域左上角的顶点

 public:
  Abstract_ImageRoi() : tl_(cv::Point2d(0, 0)) { /* code */
  }

  virtual ~Abstract_ImageRoi() {}

  /**
   * @brief 通过矩形 cv::Rect 截取 ROI
   * @details description
   * @param[in] _input_img  输入图像
   * @param[in] _rect       输入图像的 ROI 区域
   * @return cv::Mat
   */
  cv::Mat cutRoi_Rect(const cv::Mat &_input_img, const cv::Rect &_rect) {
    this->tl_ = _rect.tl();
    _input_img(_rect).copyTo(roi_img_);

    return roi_img_;
  }

  /**
   * @brief 通过旋转矩形 cv::RotatedRect 截取 ROI
   *
   * @param[in] _input_img  输入图像
   * @param[in] _r_rect     输入图像的 Rotate ROI 区域
   * @return cv::Mat
   */
  cv::Mat cutRoi_RotatedRect(const cv::Mat &_input_img,
                             const cv::RotatedRect &_rect) {
    int roi_w = MAX(_rect.size.width, _rect.size.height);
    int roi_h = MIN(_rect.size.width, _rect.size.height);
    cv::RotatedRect r_rect =
        cv::RotatedRect(_rect.center, cv::Size(roi_w, roi_h), _rect.angle);

    cv::Point2f verices[4];
    r_rect.points(verices);
    // for (int j = 0; j < 4; j++){
    //   line(_input_img, verices[j], verices[(j + 1) % 4], cv::Scalar(150, 50,
    //   100),2,8,0);
    // }
    cv::Point2f verdst[4];
    verdst[0] = cv::Point2f(0, roi_h);
    verdst[1] = cv::Point2f(0, 0);
    verdst[2] = cv::Point2f(roi_w, 0);
    verdst[3] = cv::Point2f(roi_w, roi_h);

    cv::Mat roi_img_r_rect = cv::Mat(roi_h, roi_w, CV_8UC1);
    cv::Mat warpMatrix = getPerspectiveTransform(verices, verdst);
    warpPerspective(_input_img, roi_img_r_rect, warpMatrix,
                    roi_img_r_rect.size(), cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT);

    return roi_img_r_rect;
  }

  /**
   * @brief 坐标点映射接口
   * @details 将 roi_img 中的坐标点转换为输入输入图像中的坐标点
   *          (x += tl.x, y += tl.y)
   * @param[in] _input_point
   * @return cv::Point2d
   */
  inline cv::Point2d coordMapping(const cv::Point &_input_point) {
    return cv::Point(_input_point.x + this->tl_.x,
                     _input_point.y + this->tl_.y);
  }

  /**
   * @brief 矩形映射接口
   * @details 通过 coordMapping 映射矩形左上角的顶点 tl 到原图中
   * @param[in] _input_rect
   * @return cv::Rect
   */
  inline cv::Rect rectMapping(const cv::Rect &_input_rect) {
    cv::Point convert_tl = coordMapping(_input_rect.tl());
    return cv::Rect(convert_tl.x, convert_tl.y, _input_rect.size().width,
                    _input_rect.size().height);
  }

  /**
   * @brief 旋转矩形映射接口
   * @details 通过 coordMapping 映射旋转矩形的中点 center 到原图中
   * @param[in] _input_r_rect
   * @return cv::RotatedRect
   */
  inline cv::RotatedRect r_rectMapping(const cv::RotatedRect &_input_r_rect) {
    return cv::RotatedRect(coordMapping(_input_r_rect.center),
                           _input_r_rect.size, _input_r_rect.angle);
  }
};
}  // namespace abstract_roi
