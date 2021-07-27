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
#include "module/filter/basic_kalman.hpp"

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
  int armor_forecast;
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
  bool sentryMode(const cv::Mat&           _src_img,
                  const uart::Receive_Data _receive_data);
  void initialPredictionData(const float   _gyro_speed,
                             const int     _bullet_velocity,
                             const float   _yaw_angle);
  inline cv::RotatedRect returnLastFinalArmorRotatefRect()                { return last_armor_rect_; }
  inline int             returnSentryDepth()                              { return actual_depth_; }
  inline void fixFinalArmorCenter(const int _num, const cv::Point2f _tl) {
    armor_[_num].armor_rect.center += _tl;
  }
  inline void initializationSentryMode() {
    sentry_cnt_        = 5;
    initial_gyroscope_ = 0.f;
    deviation_angle_   = 0.f;
    actual_z_          = 0;
    actual_depth_      = 0;
    forecast_angle_    = 0.f;
    forecast_pixels_   = 0;
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

  basic_kalman::Kalman1 kalman_ = basic_kalman::Kalman1();

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
  const cv::Mat light_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat armor_trackbar_  = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat sentry_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat gray_trackbar_   = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat bgr_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat hsv_trackbar_    = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat ele_ = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

  cv::Rect armor_roi;
  cv::RotatedRect last_armor_rect_;
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
  int Q_                  = 10;
  int R_                  = 5;
  int num                 = 0;

  int num_cnt_                   = 0;
  // 哨兵模型初始化计数器
  int sentry_cnt_                = 15;
  // 哨兵装甲板到枪管高度
  const int sentry_height_       = 350;
  // 哨兵到环形高度水平面垂直深度
  const int sentry_dist_         = 3380;
  // 哨兵到初始化位置实际水平深度
  int actual_z_                  = 0;
  // 哨兵到枪管实际深度
  int actual_depth_              = 0;
  // 预测像素点数量
  int forecast_pixels_           = 0;
  // 上一帧预测像素点数量
  int last_forecast_pixels_      = 0;
  // 上上帧预测像素点数量
  int last_last_forecast_pixels_ = 0;
  // 相机焦距
  const int camera_focal_        = 8;
  // 延时滤波占比
  int proportion_direction_      = 15;
  // 预测效果大小 * 0.1
  int forecast_size_             = 3000;
  // 预测最大效果 * 装甲板宽度 * 0.1
  int forecast_max_size_         = 15;
  // 判断正负范围
  int judge_direction_           = 10;
  // 像素点宽度 1080p 1mm 4个像素点
  const int pixel_width_         = 4;
  // 最大突变量
  int abrupt_variable_           = 10;
  // 射击延迟
  float firing_delay_            = 0.3;
  // 实际运动方向
  float filter_direction_        = 0.f;
  // 上一帧运动方向
  float last_direction_          = 0.f;
  // 这一帧运动方向
  float current_direction_       = 0.f;
  // 预测角度
  float forecast_angle_          = 0.f;
  // 哨兵到初始化位置的偏差角度
  float deviation_angle_         = 0.f;
  // 上一次哨兵到初始化位置的偏差角度
  float last_deviation_angle_    = 0.f;
  // 初始化陀螺仪Yaw轴位置
  float initial_gyroscope_       = 0.f;
};

}  // namespace basic_armor
