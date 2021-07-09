#pragma once

#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "abstract_armor.hpp"
#include "basic_pnp.hpp"
#include "uart_serial.hpp"
namespace basic_armor {
struct Armor_Data {
  cv::RotatedRect armor_rect;
  float width = 0;              // 装甲板宽度
  float height = 0;             // 装甲板高度
  float aspect_ratio = 0;       // 装甲板宽高比
  float tan_angle = 0;          // tan 角度
  cv::RotatedRect left_light;   // 左灯条旋转矩形
  cv::RotatedRect right_light;  // 右灯条旋转矩形

  int distance_center = 0;  // 距离图像中心点距离

  float left_light_width = 0;    // 左灯条宽度
  float right_light_width = 0;   // 右灯条高度
  float left_light_height = 0;   // 左灯条高度
  float right_light_height = 0;  // 右灯条高度

  float light_height_aspect = 0;  // 左灯条高和右灯条高比值
  float light_width_aspect = 0;   // 左灯条宽和右灯条宽比值

  int distinguish = 0;  // 大小装甲板 小0 大1
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
  // 灯条宽高比范围
  int ratio_w_h_min;
  int ratio_w_h_max;
  // 灯条角度范围
  int angle_min;
  int angle_max;
  // 灯条长度范围
  int perimeter_max;
  int perimeter_min;
};

struct Image_Cfg {
  // BGR
  int red_armor_gray_th;
  int red_armor_color_th;
  int blue_armor_gray_th;
  int blue_armor_color_th;
  // HSV-red
  int h_red_max;
  int h_red_min;
  int s_red_max;
  int s_red_min;
  int v_red_max;
  int v_red_min;
  // HSV-blue
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
/**
 * @brief 图像处理
 *
 */
class RM_ArmorDetector {
  // 装甲板
 public:
  uart::Write_Data run_Armor(cv::Mat &_src_img,
                             uart::Receive_Data _receive_data);
  bool light_Judge(int i, int j);  // 判断左右灯条能否组成装甲板
  int average_Color();             // 计算图像颜色平均值
  bool fitting_Armor();            // 拟合装甲板
  bool find_Light();               // 寻找灯条
  void final_Armor();              // 最优装甲板
  void free_Memory();              // 释放内存
  int motion_Direction();          // 判断装甲板运动方向
  inline Armor_Data returnFinalArmor(int _num) { return armor_[_num]; }

  /**
   * @brief 指定返回已排序后第几个装甲板的参数
   *
   * @return Armor_Data
   */
  inline cv::RotatedRect returnFinalArmorRotatedRect(int _num) {
    return armor_[_num].armor_rect;
  }
  inline int returnFinalArmorDistinguish(int _num) {
    return armor_[_num].distinguish;
  }

  // inline size_t returnArmornum() { return armor_.size(); };

  RM_ArmorDetector() {}
  explicit RM_ArmorDetector(const std::string _armor_config);
  ~RM_ArmorDetector() {}

  inline bool returnSuccessArmor() { return armor_success; }

 private:
  Armor_Cfg armor_config_;
  Image_Cfg image_config_;
  Light_Cfg light_config_;

  cv::Mat frame;      // 原图
  cv::Mat draw_img_;  // 画板
  cv::Mat gray_img_;
  cv::Mat hsv_img;
  cv::Mat bin_gray_img;
  cv::Mat bin_color_img;
  cv::Mat light_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat armor_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);

  basic_pnp::RM_Solvepnp pnp_ =
      basic_pnp::RM_Solvepnp("configs/camera/cameraParams/cameraParams_407.xml",
                             "configs/angle_solve/basic_pnp_config.xml");

  Armor_Data armor_data_;

  std::vector<Armor_Data> armor_;
  std::vector<cv::RotatedRect> light_;

  cv::Rect armor_roi;

  cv::Point lost_armor_center;
  cv::Point armor_center;  // 装甲板中心点

  bool lost_armor_success = false;
  bool armor_success = false;
  bool switch_armor = false;  // 切换装甲板

  int lost_distance_armor = 0;  // 两帧装甲板之间的距离
  int amplitude = 0;            // 幅度
  int optimal_armor = 0;        // 最优装甲板
  int armor_position = 0;       // 装甲板在车的位置
  int armor_direction = 0;      // 1向右 -1 向左
  int num = 0;                  // 运行次数

  // 图像
 private:
  cv::Mat gray_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat bgr_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  cv::Mat hsv_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
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
