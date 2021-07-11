#include "basic_armor.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace basic_armor {

RM_ArmorDetector::RM_ArmorDetector(std::string _armor_config) {
  cv::FileStorage fs_armor(_armor_config, cv::FileStorage::READ);

  fs_armor["GRAY_EDIT"]          >> image_config_.gray_edit;
  fs_armor["COLOR_EDIT"]         >> image_config_.color_edit;
  fs_armor["METHOD"]             >> image_config_.method;
  fs_armor["BLUE_ARMOR_GRAY_TH"] >> image_config_.blue_armor_gray_th;
  fs_armor["RED_ARMOR_GRAY_TH"]  >> image_config_.red_armor_gray_th;

  if (image_config_.method == 0) {
    fs_armor["RED_ARMOR_COLOR_TH"]  >> image_config_.red_armor_color_th;
    fs_armor["BLUE_ARMOR_COLOR_TH"] >> image_config_.blue_armor_color_th;
  } else {
    fs_armor["H_RED_MIN"] >> image_config_.h_red_min;
    fs_armor["H_RED_MAX"] >> image_config_.h_red_max;
    fs_armor["S_RED_MIN"] >> image_config_.s_red_min;
    fs_armor["S_RED_MAX"] >> image_config_.s_red_max;
    fs_armor["V_RED_MIN"] >> image_config_.v_red_min;
    fs_armor["V_RED_MAX"] >> image_config_.v_red_max;

    fs_armor["H_BLUE_MIN"] >> image_config_.h_blue_min;
    fs_armor["H_BLUE_MAX"] >> image_config_.h_blue_max;
    fs_armor["S_BLUE_MIN"] >> image_config_.s_blue_min;
    fs_armor["S_BLUE_MAX"] >> image_config_.s_blue_max;
    fs_armor["V_BLUE_MIN"] >> image_config_.v_blue_min;
    fs_armor["V_BLUE_MAX"] >> image_config_.v_blue_max;
  }
  fs_armor["LIGHT_DRAW"]             >> light_config_.light_draw;
  fs_armor["LIGHT_EDTI"]             >> light_config_.light_edit;

  fs_armor["LIGHT_RATIO_W_H_MIN"]    >> light_config_.ratio_w_h_min;
  fs_armor["LIGHT_RATIO_W_H_MAX"]    >> light_config_.ratio_w_h_max;

  fs_armor["LIGHT_ANGLE_MIN"]        >> light_config_.angle_min;
  fs_armor["LIGHT_ANGLE_MAX"]        >> light_config_.angle_max;

  fs_armor["LIGHT_PERIMETER_MIN"]    >> light_config_.perimeter_min;
  fs_armor["LIGHT_PERIMETER_MAX"]    >> light_config_.perimeter_max;

  fs_armor["ARMOR_EDIT"]             >> armor_config_.armor_edit;
  fs_armor["ARMOR_DRAW"]             >> armor_config_.armor_draw;
  fs_armor["ARMOR_HEIGHT_RATIO_MIN"] >> armor_config_.light_height_ratio_min;
  fs_armor["ARMOR_HEIGHT_RATIO_MAX"] >> armor_config_.light_height_ratio_max;

  fs_armor["ARMOR_WIDTH_RATIO_MIN"]  >> armor_config_.light_width_ratio_min;
  fs_armor["ARMOR_WIDTH_RATIO_MAX"]  >> armor_config_.light_width_ratio_max;

  fs_armor["ARMOR_Y_DIFFERENT"]      >> armor_config_.light_y_different;
  fs_armor["ARMOR_HEIGHT_DIFFERENT"] >> armor_config_.light_height_different;
  fs_armor["ARMOR_ANGLE_DIFFERENT"]  >> armor_config_.armor_angle_different;

  fs_armor["ARMOR_SMALL_ASPECT_MIN"] >> armor_config_.small_armor_aspect_min;
  fs_armor["ARMOR_TYPE_TH"]          >> armor_config_.armor_type_th;
  fs_armor["ARMOR_BIG_ASPECT_MAX"]   >> armor_config_.big_armor_aspect_max;

  fmt::print("[{}] Info, Armor configuration initial success.\n", idntifier_green);
}

float Distance(cv::Point a,
               cv::Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void RM_ArmorDetector::free_Memory() {
  lost_armor_success = armor_success;
  armor_success      = false;

  if (light_.size() > 0) {
    light_.clear();
    std::vector<cv::RotatedRect>(light_).swap(light_);
    if (armor_.size() > 0) {
      armor_.clear();
      std::vector<Armor_Data>(armor_).swap(armor_);
    }
  }
}

bool RM_ArmorDetector::find_Light() {
  int perimeter = 0;
  cv::RotatedRect box;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin_color_img,
                   contours,
                   cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);

  if (contours.size() < 2) {
    fmt::print("[{}] Info, quantity of contours less than two.\n", idntifier_green);
    return false;
  }

  if (light_config_.light_edit == 1) {
    cv::namedWindow("light_trackbar");

    cv::createTrackbar("angle_min", "light_trackbar",
                       &light_config_.angle_min, 1800, NULL);
    cv::createTrackbar("angle_max", "light_trackbar",
                       &light_config_.angle_max, 1800, NULL);

    cv::createTrackbar("perimeter_min", "light_trackbar",
                       &light_config_.perimeter_min, 100000, NULL);
    cv::createTrackbar("perimeter_max", "light_trackbar",
                       &light_config_.perimeter_max, 100000, NULL);

    cv::createTrackbar("ratio_w_h_min", "light_trackbar",
                       &light_config_.ratio_w_h_min, 1000, NULL);
    cv::createTrackbar("ratio_w_h_max", "light_trackbar",
                       &light_config_.ratio_w_h_max, 1000, NULL);

    cv::imshow("light_trackbar", light_trackbar_);
  }

  for (size_t i = 0; i < contours.size(); i++) {
    perimeter = arcLength(contours[i], true);

    if (perimeter < light_config_.perimeter_min ||
        perimeter > light_config_.perimeter_max ||
        contours[i].size() < 5) {
      continue;
    }

    box = cv::fitEllipse(cv::Mat(contours[i]));

    if (box.angle > 90.0f) {
      box.angle = box.angle - 180.0f;
    }

    static float light_w_h;
    static float _h = MAX(box.size.width, box.size.height);
    static float _w = MIN(box.size.width, box.size.height);
    light_w_h       = _h / _w;

    if (box.angle < light_config_.angle_max &&
        box.angle > -light_config_.angle_min &&
        light_w_h < light_config_.ratio_w_h_max &&
        light_w_h > light_config_.ratio_w_h_min) {
      this->light_.push_back(box);

      if (light_config_.light_draw == 1) {
        cv::Point2f vertex[4];
        box.points(vertex);
        for (int l = 0; l < 4; l++) {
          line(draw_img_, vertex[l], vertex[(l + 1) % 4],
               cv::Scalar(0, 255, 255), 3, 8);
        }
      }
    }
  }

  if (this->light_.size() < 2) {
    fmt::print("[{}] Info, quantity of light bar less than two.\n", idntifier_green);
    return false;
  }
  return true;
}

uart::Write_Data RM_ArmorDetector::run_Armor(cv::Mat            &_src_img,
                                             uart::Receive_Data _receive_data) {
  run_Image(_src_img, _receive_data.my_color);
  draw_img_ = _src_img;
  if (find_Light()) {
    if (fitting_Armor()) {
      final_Armor();
      if (armor_config_.armor_draw == 1 || light_config_.light_draw == 1 ||
          armor_config_.armor_edit == 1 || light_config_.light_edit == 1) {
        imshow("armor_draw_img", draw_img_);
        draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
      }
      pnp_.solvePnP(_receive_data.bullet_velocity,
                        armor_[0].distinguish,
                        armor_[0].armor_rect);
    }
  }
  if (armor_config_.armor_draw == 1 || light_config_.light_draw == 1 ||
      armor_config_.armor_edit == 1 || light_config_.light_edit == 1) {
    imshow("armor_draw_img", draw_img_);
    draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
  }
  return serial_.gainWriteData(pnp_.returnYawAngle(),
                               pnp_.returnPitchAngle(),
                               pnp_.returnDepth(),
                               armor_.size(),
                               0);
}

bool comparison(Armor_Data _a, Armor_Data _b) {
  return _a.distance_center < _b.distance_center;
}

void RM_ArmorDetector::final_Armor() {
  armor_success = true;
  if (armor_.size() == 1) {
    fmt::print("[{}] Info, only one armor.\n", idntifier_green);
  } else {
    fmt::print("[{}] Info, multiple armors.\n", idntifier_green);
    std::sort(armor_.begin(), armor_.end(), comparison);
    if (armor_config_.armor_draw == 1 || armor_config_.armor_edit == 1) {
      cv::rectangle(draw_img_, armor_[0].armor_rect.boundingRect(),
                    cv::Scalar(0, 255, 0), 3, 8);
    }
  }
}

bool RM_ArmorDetector::fitting_Armor() {
  if (armor_config_.armor_edit == 1) {
    cv::namedWindow("armor_trackbar");
    cv::createTrackbar("light_height_aspect_min", "armor_trackbar",
                       &armor_config_.light_height_ratio_min, 100, NULL);
    cv::createTrackbar("light_height_aspect_max", "armor_trackbar",
                       &armor_config_.light_height_ratio_max, 100, NULL);
    cv::createTrackbar("light_width_aspect_min", "armor_trackbar",
                       &armor_config_.light_width_ratio_min, 100, NULL);
    cv::createTrackbar("light_width_ratio_max", "armor_trackbar",
                       &armor_config_.light_width_ratio_max, 100, NULL);
    cv::createTrackbar("light_y_different", "armor_trackbar",
                       &armor_config_.light_y_different, 100, NULL);
    cv::createTrackbar("light_height_different", "armor_trackbar",
                       &armor_config_.light_height_different, 100, NULL);
    cv::createTrackbar("armor_angle_different", "armor_trackbar",
                       &armor_config_.armor_angle_different, 100, NULL);
    cv::createTrackbar("small_armor_aspect_min", "armor_trackbar",
                       &armor_config_.small_armor_aspect_min, 100, NULL);
    cv::createTrackbar("armor_type_th", "armor_trackbar",
                       &armor_config_.armor_type_th, 100, NULL);
    cv::createTrackbar("big_armor_aspect_max", "armor_trackbar",
                       &armor_config_.big_armor_aspect_max, 100, NULL);

    cv::imshow("armor_trackbar", armor_trackbar_);
  }

  for (size_t i = 0; i < this->light_.size(); i++) {
    for (size_t j = i + 1; j < this->light_.size(); j++) {
      int light_left = 0, light_right = 0;
      if (this->light_[i].center.x > this->light_[j].center.x) {
        light_left  = j;
        light_right = i;
      } else {
        light_left  = i;
        light_right = j;
      }

      armor_data_.left_light  = this->light_[light_left];
      armor_data_.right_light = this->light_[light_right];

      float error_angle =
        atan((light_[light_right].center.y - light_[light_left].center.y) /
             (light_[light_right].center.x - light_[light_left].center.x));

      if (error_angle < 10.0f) {
        armor_data_.tan_angle = atan(error_angle) * 180 / CV_PI;
        if (this->light_Judge(light_left, light_right)) {
          if (this->average_Color() < 100) {
            armor_.push_back(armor_data_);
            if (armor_config_.armor_draw == 1 ||
                armor_config_.armor_edit == 1) {
              rectangle(draw_img_, armor_data_.armor_rect.boundingRect(),
                        cv::Scalar(255, 255, 0), 5, 8);
            }
          }
        }
      }
    }
  }
  if (this->armor_.size() < 1) {
    fmt::print("[{}] Info, armor not found.\n", idntifier_green);
    return false;
  }
  return true;
}

bool RM_ArmorDetector::light_Judge(int i, int j) {
  armor_data_.left_light_height   =
      MAX(light_[i].size.height, light_[i].size.width);
  armor_data_.left_light_width    =
      MIN(light_[i].size.height, light_[i].size.width);
  armor_data_.right_light_height  =
      MAX(light_[j].size.height, light_[j].size.width);
  armor_data_.right_light_width   =
      MIN(light_[j].size.height, light_[j].size.width);
  armor_data_.light_height_aspect =
      armor_data_.left_light_height / armor_data_.right_light_height;
  armor_data_.light_width_aspect  =
      armor_data_.left_light_width / armor_data_.right_light_width;

  if (armor_data_.light_height_aspect <
          (armor_config_.light_height_ratio_max * 0.1) &&
      armor_data_.light_height_aspect >
          (armor_config_.light_height_ratio_min * 0.1) &&
      armor_data_.light_width_aspect <
          (armor_config_.light_width_ratio_max * 0.1) &&
      armor_data_.light_width_aspect >
          (armor_config_.light_height_ratio_min * 0.1)) {
    armor_data_.height = (armor_data_.left_light.size.height +
                          armor_data_.right_light.size.height) / 2;

    if (fabs(armor_data_.left_light.center.y -
        armor_data_.right_light.center.y) <
        (armor_data_.height * armor_config_.light_y_different * 0.1)) {
      if (fabs(armor_data_.left_light.size.height -
          armor_data_.right_light.size.height) <
          (armor_data_.height * armor_config_.light_height_different * 0.1)) {
            armor_data_.width = Distance(armor_data_.left_light.center,
                                     armor_data_.right_light.center);

        armor_data_.aspect_ratio = armor_data_.width / armor_data_.height;

        if (fabs(armor_data_.left_light.angle - armor_data_.right_light.angle) <
          (armor_config_.armor_angle_different * 0.1)) {
            cv::RotatedRect rects = cv::RotatedRect(
              (armor_data_.left_light.center + armor_data_.right_light.center) / 2,
              cv::Size(armor_data_.width, armor_data_.height),
              armor_data_.tan_angle);

          armor_data_.armor_rect      = rects;
          armor_data_.distance_center =
            Distance(armor_data_.armor_rect.center,
                     cv::Point(this->draw_img_.cols, this->draw_img_.rows));
          if (armor_data_.aspect_ratio >
                (armor_config_.small_armor_aspect_min * 0.1) &&
              (armor_data_.aspect_ratio < armor_config_.armor_type_th * 0.1)) {
            armor_data_.distinguish = 0;
            return true;
          } else if (armor_data_.aspect_ratio >
                      (armor_config_.armor_type_th * 0.1) &&
                     armor_data_.aspect_ratio <
                      (armor_config_.big_armor_aspect_max * 0.1)) {
            armor_data_.distinguish = 1;
            return true;
          }
        }
      }
    }
  }
  return false;
}

int RM_ArmorDetector::average_Color() {
  armor_data_.left_light_height  =
      MAX(armor_data_.left_light_height, armor_data_.left_light_width);
  armor_data_.left_light_width   =
      MIN(armor_data_.left_light_height, armor_data_.left_light_width);
  armor_data_.right_light_height =
      MAX(armor_data_.right_light_height, armor_data_.right_light_width);
  armor_data_.right_light_width  =
      MIN(armor_data_.right_light_height, armor_data_.right_light_width);
  cv::RotatedRect rects = cv::RotatedRect(
      (armor_data_.left_light.center + armor_data_.right_light.center) / 2,
      cv::Size(
          armor_data_.width -
              (armor_data_.left_light_width + armor_data_.right_light_width),
          (armor_data_.left_light_height + armor_data_.right_light_height) / 2),
      armor_data_.tan_angle);
  cv::rectangle(draw_img_, rects.boundingRect(), cv::Scalar(0, 0, 255), 3, 8);
  armor_data_.armor_rect = rects;
  cv::Rect _rect         = rects.boundingRect();
  if (_rect.x <= 0) {
    _rect.x = 0;
  }
  if (_rect.y <= 0) {
    _rect.y = 0;
  }
  if (_rect.y + _rect.height >= bin_gray_img.rows) {
    _rect.height = bin_gray_img.rows - _rect.y;
  }
  if (_rect.x + _rect.width >= bin_gray_img.cols) {
    _rect.width = bin_gray_img.cols - _rect.x;
  }
  static cv::Mat roi    = bin_gray_img(_rect);
  int average_intensity = static_cast<int>(mean(roi).val[0]);
  return average_intensity;
}

void RM_ArmorDetector::run_Image(cv::Mat  &_src_img,
                                 const int _my_color) {
  switch (image_config_.method) {
    case 0:
      bin_color_img = this->fuse_Image(this->gray_Pretreat(_src_img, _my_color),
                                       this->bgr_Pretreat(_src_img, _my_color));
      break;
    default:
      bin_color_img = this->fuse_Image(this->gray_Pretreat(_src_img, _my_color),
                                       this->hsv_Pretreat(_src_img, _my_color));
      break;
  }
}

cv::Mat RM_ArmorDetector::fuse_Image(cv::Mat _bin_gray_img,
                                     cv::Mat _bin_color_img) {
  cv::bitwise_and(_bin_color_img, _bin_gray_img, _bin_color_img);
  morphologyEx(_bin_color_img, _bin_color_img, cv::MORPH_DILATE, this->ele_);
  return _bin_color_img;
}

cv::Mat RM_ArmorDetector::gray_Pretreat(cv::Mat  &_src_img,
                                        const int _my_color) {
  cv::cvtColor(_src_img, gray_img_, cv::COLOR_BGR2GRAY);
  switch (_my_color) {
    case 0:
      if (image_config_.gray_edit) {
        cv::namedWindow("gray_trackbar");
        cv::createTrackbar("gray_th", "gray_trackbar",
                           &image_config_.blue_armor_gray_th, 255, NULL);
        cv::imshow("gray_trackbar", this->gray_trackbar_);
      }
      cv::threshold(gray_img_, bin_gray_img, image_config_.blue_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
    default:
      if (image_config_.gray_edit) {
        cv::namedWindow("gray_trackbar");
        cv::createTrackbar("gray_th", "gray_trackbar",
                           &image_config_.red_armor_gray_th, 255, NULL);
        cv::imshow("gray_trackbar", this->gray_trackbar_);
      }
      cv::threshold(gray_img_, bin_gray_img, image_config_.red_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
  }
  if (image_config_.gray_edit) {
    cv::imshow("gray_trackbar", bin_gray_img);
  }
  return bin_gray_img;
}

cv::Mat RM_ArmorDetector::bgr_Pretreat(cv::Mat  &_src_img,
                                       const int _my_color) {
  static std::vector<cv::Mat> _split;
  static cv::Mat bin_color_img;
  cv::split(_src_img, _split);
  switch (_my_color) {
    case 0:
      cv::subtract(_split[0], _split[2], bin_color_img);

      if (image_config_.color_edit) {
        cv::namedWindow("color_trackbar");
        cv::createTrackbar("blue_color_th", "color_trackbar",
                           &image_config_.blue_armor_color_th, 255, NULL);
        cv::imshow("color_trackbar", this->bgr_trackbar_);
      }

      cv::threshold(bin_color_img, bin_color_img,
                    image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
      break;
    default:
      cv::subtract(_split[2], _split[0], bin_color_img);

      if (image_config_.color_edit) {
        cv::namedWindow("color_trackbar");
        cv::createTrackbar("red_color_th", "color_trackbar",
                           &image_config_.red_armor_color_th, 255, NULL);
        cv::imshow("color_trackbar", this->bgr_trackbar_);
      }

      cv::threshold(bin_color_img, bin_color_img,
                    image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
      break;
  }
  if (image_config_.gray_edit) {
    cv::imshow("color_trackbar", bin_color_img);
  }
  return bin_color_img;
}

cv::Mat RM_ArmorDetector::hsv_Pretreat(cv::Mat  &_src_img,
                                       const int _my_color) {
  cv::cvtColor(_src_img, hsv_img, cv::COLOR_BGR2HSV_FULL);
  switch (_my_color) {
    case 0:
      if (image_config_.color_edit) {
        cv::namedWindow("hsv_trackbar");
        cv::createTrackbar("blue_h_min:", "hsv_trackbar",
                           &image_config_.h_blue_min, 255, NULL);
        cv::createTrackbar("blue_h_max:", "hsv_trackbar",
                           &image_config_.h_blue_max, 255, NULL);
        cv::createTrackbar("blue_s_min:", "hsv_trackbar",
                           &image_config_.s_blue_min, 255, NULL);
        cv::createTrackbar("blue_s_max:", "hsv_trackbar",
                           &image_config_.s_blue_max, 255, NULL);
        cv::createTrackbar("blue_v_min:", "hsv_trackbar",
                           &image_config_.v_blue_min, 255, NULL);
        cv::createTrackbar("blue_v_max:", "hsv_trackbar",
                           &image_config_.v_red_max, 255, NULL);
        cv::imshow("hsv_trackbar", this->hsv_trackbar_);
      }
      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_blue_min, image_config_.s_blue_min,
                             image_config_.v_blue_min),
                  cv::Scalar(image_config_.h_blue_max, image_config_.s_blue_max,
                             image_config_.v_blue_max),
                  bin_color_img);
      break;
    default:
      if (image_config_.color_edit) {
        cv::namedWindow("hsv_trackbar");
        cv::createTrackbar("red_h_min:", "hsv_trackbar",
                           &image_config_.h_red_min, 255, NULL);
        cv::createTrackbar("red_h_max:", "hsv_trackbar",
                           &image_config_.h_red_max, 255, NULL);
        cv::createTrackbar("red_s_min:", "hsv_trackbar",
                           &image_config_.s_red_min, 255, NULL);
        cv::createTrackbar("red_s_max:", "hsv_trackbar",
                           &image_config_.s_red_max, 255, NULL);
        cv::createTrackbar("red_v_min:", "hsv_trackbar",
                           &image_config_.v_red_min, 255, NULL);
        cv::createTrackbar("red_v_max:", "hsv_trackbar",
                           &image_config_.v_red_max, 255, NULL);
        cv::imshow("hsv_trackbar", this->hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_red_min,
                             image_config_.s_red_min,
                             image_config_.v_red_min),
                  cv::Scalar(image_config_.h_red_max,
                             image_config_.s_red_max,
                             image_config_.v_red_max),
                  bin_color_img);

      break;
  }

  if (image_config_.gray_edit) {
    cv::imshow("hsv_trackbar", bin_color_img);
  }

  return bin_color_img;
}
}  // namespace basic_armor
