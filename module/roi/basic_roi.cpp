#include "basic_roi.hpp"

namespace basic_roi {
cv::Rect ImageRoi::makeRectSafe_Fixed(const cv::Mat &_input_img,
                                      const cv::RotatedRect &_r_rect) {
  int width = _r_rect.boundingRect().width;
  int height = _r_rect.boundingRect().height;

  cv::Point tl =
      cv::Point(_r_rect.center.x - width * 0.5f, _r_rect.center.y * 0.5f);
  if (tl.x < 0) {
    tl.x = 0;
  }
  if (tl.y < 0) {
    tl.y = 0;
  }

  if (tl.x + width > _input_img.cols) {
    tl.x -= (tl.x + width - _input_img.cols);
  }

  if (tl.y + height > _input_img.rows) {
    tl.y -= (tl.y + height - _input_img.rows);
  }

  return cv::Rect(tl.x, tl.y, width, height);
} 

cv::Rect ImageRoi::makeRectSafe_Tailor(const cv::Mat &_input_img,
                                       const cv::RotatedRect &_r_rect) {
  int width = _r_rect.boundingRect().width;
  int height = _r_rect.boundingRect().height;

  cv::Point tl =
      cv::Point(_r_rect.center.x - width * 0.5f, _r_rect.center.y * 0.5f);

  if (tl.x < 0) {
    width -= (0 - tl.x);
    tl.x = 0;
  }
  if (tl.y < 0) {
    height -= (0 - tl.y);
    tl.y = 0;
  }

  if (tl.x + width > _input_img.cols) {
    width -= (tl.x + width - _input_img.cols);
  }
  if (tl.y + height > _input_img.rows) {
    height -= (tl.y + height - _input_img.rows);
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

cv::Rect ImageRoi::makeRectSafe_Tailor(const cv::Mat &_input_img,
                                       const cv::Rect &_r_rect) {
  int width = _r_rect.width;
  int height = _r_rect.height;

  cv::Point tl = _r_rect.tl();

  if (tl.x < 0) {
    width -= (0 - tl.x);
    tl.x = 0;
  }
  if (tl.y < 0) {
    height -= (0 - tl.y);
    tl.y = 0;
  }

  if (tl.x + width > _input_img.cols) {
    width -= (tl.x + width - _input_img.cols);
  }
  if (tl.y + height > _input_img.rows) {
    height -= (tl.y + height - _input_img.rows);
  }

  return cv::Rect(tl.x, tl.y, width, height);
}

}  // namespace basic_roi
