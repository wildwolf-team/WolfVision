#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "devices/serial/uart_serial.hpp"

namespace basic_armor {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_armor");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_armor");

struct Armor_Data {
  float width        = 0;
  float height       = 0;
  float aspect_ratio = 0;
  float tan_angle    = 0;

  cv::RotatedRect armor_rect;
  cv::RotatedRect left_light;
  cv::RotatedRect right_light;

  int distance_center = 0;
  int distinguish     = 0;

  float left_light_width   = 0;
  float right_light_width  = 0;
  float left_light_height  = 0;
  float right_light_height = 0;

  float light_height_aspect = 0;
  float light_width_aspect  = 0;
};

struct Armor_Config {
  int armor_edit;
  int armor_draw;
  int light_height_ratio_min;
  int light_height_ratio_max;
  int light_width_ratio_min;
  int light_width_ratio_max;

  int light_y_different;
  int light_height_different;
  int armor_angle_different;

  int small_armor_aspect_min;
  int armor_type_th;
  int big_armor_aspect_max;
};

struct Light_Config {
  int light_draw;
  int light_edit;

  int ratio_w_h_min;
  int ratio_w_h_max;

  int angle_min;
  int angle_max;

  int perimeter_max;
  int perimeter_min;
};

struct Image_Config {
  int red_armor_gray_th;
  int red_armor_color_th;
  int blue_armor_gray_th;
  int blue_armor_color_th;

  int h_red_max;
  int h_red_min;
  int s_red_max;
  int s_red_min;
  int v_red_max;
  int v_red_min;

  int h_blue_max;
  int h_blue_min;
  int s_blue_max;
  int s_blue_min;
  int v_blue_max;
  int v_blue_min;

  int gray_edit  = 0;
  int color_edit = 0;
  int method     = 0;
};

class Detector {
 public:
  explicit Detector(const std::string _armor_config);
  ~Detector() = default;

  bool runBasicArmor(const cv::Mat&           _src_img,
                     const uart::Receive_Data _receive_data);
  float getDistance(const cv::Point a, const cv::Point b);
  bool  lightJudge(const int i, const int j);
  bool  fittingArmor();
  bool  findLight();
  void  finalArmor();
  void  freeMemory();
  int   averageColor();
  int   motionDirection();

  inline int             returnLostCnt()                                  { return lost_cnt_--; }
  inline int             returnArmorNum()                                 { return armor_.size(); }
  inline bool            returnSuccessArmor()                             { return armor_success; }
  inline Armor_Data      returnFinalArmor(const int _num)                 { return armor_[_num]; }
  inline int             returnFinalArmorDistinguish(const int _num)      { return armor_[_num].distinguish; }
  inline cv::RotatedRect returnFinalArmorRotatedRect(const int _num)      { return armor_[_num].armor_rect; }
  inline void fixFinalArmorCenter(const int _num, const cv::Point2f _tl) {
    armor_[_num].armor_rect.center += _tl;
  }

  void runImage(const cv::Mat &_src_img, const int _my_color);

  cv::Mat bgrPretreat(const cv::Mat &_src_img, const int _my_color);
  cv::Mat hsvPretreat(const cv::Mat &_src_img, const int _my_color);
  cv::Mat grayPretreat(const cv::Mat &_src_img, const int _my_color);

  cv::Mat fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img);


 private:
  Armor_Config armor_config_;
  Image_Config image_config_;
  Light_Config light_config_;
  Armor_Data   armor_data_;

  cv::Mat frame;
  cv::Mat draw_img_;
  cv::Mat gray_img_;
  cv::Mat hsv_img;
  cv::Mat bin_gray_img;
  cv::Mat bin_red_gray_img;
  cv::Mat bin_blue_gray_img;
  cv::Mat bin_color_img;
  cv::Mat bin_red_color_img;
  cv::Mat bin_blue_color_img;
  const cv::Mat light_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat armor_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat gray_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat bgr_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat ele_ = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

  cv::Rect armor_roi;

  cv::Point lost_armor_center;
  cv::Point armor_center;

  std::vector<Armor_Data>      armor_;
  std::vector<cv::RotatedRect> light_;

  bool lost_armor_success = false;
  bool armor_success      = false;
  bool switch_armor       = false;

  int lost_cnt_           = 10;
  int lost_distance_armor = 0;
  int amplitude           = 0;
  int optimal_armor       = 0;
  int armor_position      = 0;
  int armor_direction     = 0;
  int num                 = 0;
};

}  // namespace basic_armor
