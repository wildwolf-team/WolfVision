#pragma once

#include <fmt/core.h>
#include <fmt/color.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "basic_pnp.hpp"
#include "uart_serial.hpp"
#include "abstract_armor.hpp"

namespace basic_armor {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_armor");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_armor");

struct Armor_Data {
  cv::RotatedRect armor_rect;
  float width        = 0;
  float height       = 0;
  float aspect_ratio = 0;
  float tan_angle    = 0;
  cv::RotatedRect left_light;
  cv::RotatedRect right_light;

  int distance_center = 0;

  float left_light_width   = 0;
  float right_light_width  = 0;
  float left_light_height  = 0;
  float right_light_height = 0;

  float light_height_aspect = 0;
  float light_width_aspect  = 0;

  int distinguish = 0;
};

struct Armor_Cfg {
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

struct Light_Cfg {
  int light_draw;
  int light_edit;

  int ratio_w_h_min;
  int ratio_w_h_max;

  int angle_min;
  int angle_max;

  int perimeter_max;
  int perimeter_min;
};

struct Image_Cfg {
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

  int gray_edit = 0;
  int color_edit = 0;
  int method = 0;
};

class RM_ArmorDetector {
 public:
  uart::Write_Data run_Armor(cv::Mat           &_src_img,
                             uart::Receive_Data _receive_data);
  bool light_Judge(int i, int j);
  int  average_Color();
  bool fitting_Armor();
  bool find_Light();
  void final_Armor();
  void free_Memory();
  int  motion_Direction();
  inline Armor_Data returnFinalArmor(int _num) { return armor_[_num]; }

  inline cv::RotatedRect returnFinalArmorRotatedRect(int _num) {
    return armor_[_num].armor_rect;
  }
  inline int returnFinalArmorDistinguish(int _num) {
    return armor_[_num].distinguish;
  }

  RM_ArmorDetector() {}
  explicit RM_ArmorDetector(const std::string _armor_config);
  ~RM_ArmorDetector() {}

  inline bool returnSuccessArmor() { return armor_success; }

 private:
  Armor_Cfg armor_config_;
  Image_Cfg image_config_;
  Light_Cfg light_config_;

  cv::Mat frame;
  cv::Mat draw_img_;
  cv::Mat gray_img_;
  cv::Mat hsv_img;
  cv::Mat bin_gray_img;
  cv::Mat bin_color_img;
  cv::Mat light_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat armor_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);

  basic_pnp::PnP pnp_ =
      basic_pnp::PnP("configs/camera/cameraParams/cameraParams_407.xml",
                             "configs/angle_solve/basic_pnp_config.xml");
  uart::SerialPort serial_ =
      uart::SerialPort("configs/serial/uart_serial_config.xml");
  Armor_Data armor_data_;

  std::vector<Armor_Data>      armor_;
  std::vector<cv::RotatedRect> light_;

  cv::Rect armor_roi;

  cv::Point lost_armor_center;
  cv::Point armor_center;

  bool lost_armor_success = false;
  bool armor_success      = false;
  bool switch_armor       = false;

  int lost_distance_armor = 0;
  int amplitude           = 0;
  int optimal_armor       = 0;
  int armor_position      = 0;
  int armor_direction     = 0;
  int num                 = 0;

  // 图像
 private:
  cv::Mat gray_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat bgr_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat hsv_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat ele_ = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

 public:
  void set_Image_Config(Image_Cfg _Image_Config);

  void run_Image(cv::Mat &_src_img, const int _my_color);

  cv::Mat bgr_Pretreat(cv::Mat &_src_img, const int _my_color);

  cv::Mat hsv_Pretreat(cv::Mat &_src_img, const int _my_color);

  cv::Mat gray_Pretreat(cv::Mat &_src_img, const int _my_color);

  cv::Mat fuse_Image(cv::Mat _bin_gray_img, cv::Mat _bin_color_img);
};
}  // namespace basic_armor
