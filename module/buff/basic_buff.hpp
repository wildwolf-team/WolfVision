#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <fmt/color.h>

#include "abstract_center_r.hpp"
#include "abstract_target.hpp"

#include "devices/serial/uart_serial.hpp"
#include "module/angle_solve/basic_pnp.hpp"
#include "roi/basic_roi.hpp"
#include "utils/fps.hpp"

namespace basic_buff {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_buff");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "basic_buff");

struct Buff_Param {
  int RED_BUFF_GRAY_TH;
  int RED_BUFF_COLOR_TH;
  int BLUE_BUFF_GRAY_TH;
  int BLUE_BUFF_COLOR_TH;

  int H_RED_MAX;
  int H_RED_MIN;
  int S_RED_MAX;
  int S_RED_MIN;
  int V_RED_MAX;
  int V_RED_MIN;

  int H_BLUE_MAX;
  int H_BLUE_MIN;
  int S_BLUE_MAX;
  int S_BLUE_MIN;
  int V_BLUE_MAX;
  int V_BLUE_MIN;

  int SMALL_TARGET_AREA_MAX;
  int SMALL_TARGET_AREA_MIN;
  int BIG_TARGET_AREA_MAX;
  int BIG_TARGET_AREA_MIN;

  int SMALL_TARGET_Length_MIN;
  int SMALL_TARGET_Length_MAX;
  int BIG_TARGET_Length_MIN;
  int BIG_TARGET_Length_MAX;

  int DIFF_ANGLE_MAX;
  int DIFF_ANGLE_MIN;

  float SMALL_TARGET_ASPECT_RATIO_MAX;
  float SMALL_TARGET_ASPECT_RATIO_MIN;

  float AREA_RATIO_MAX;
  float AREA_RATIO_MIN;

  float BIG_LENTH_R;

  int CENTER_R_ROI_SIZE;

  float FILTER_COEFFICIENT;

  float BUFF_H;
  float BUFF_RADIUS;
  float PLATFORM_H;
  float BARREL_ROBOT_H;
  float TARGET_X;

  float OFFSET_FIXED_RADIAN;
};

struct Buff_Ctrl {
  int IS_PARAM_ADJUSTMENT;
  int IS_SHOW_BIN_IMG;
  int PROCESSING_MODE;
};

struct Buff_Config {
  Buff_Param param;
  Buff_Ctrl  ctrl;
};

class Detector {
 public:
  Detector() = default;
  explicit Detector(const std::string& _buff_config_path);

  ~Detector() = default;

  void             runTask(cv::Mat&            _input_img,
                           uart::Receive_Data& _receive_info,
                           uart::Write_Data&   _send_info);
  uart::Write_Data runTask(cv::Mat&            _input_img,
                           uart::Receive_Data& _receive_info);

 private:
  void readBuffConfig(const cv::FileStorage& _fs);

  void getInput(cv::Mat& _input_img, const int& _my_color);

  void displayDst();

  cv::Mat                  src_img_;          // 输入原图
  cv::Mat                  dst_img_;          // 图像效果展示图
  cv::RotatedRect          target_rect_;      // 目标矩形

  std::vector<cv::Point2f> target_2d_point_;  // 目标二维点集

  float                    final_target_z_;   // 最终打击目标的深度信息 (预测点)
  int                      my_color_;         // 当前自己的颜色

  Buff_Config              buff_config_;      // 参数结构体
  abstract_target::Target  last_target_;      // 上一个打击目标

  bool is_find_last_target_;
  bool is_find_target_;

 private:
  enum Processing_Mode {
    BGR_MODE,
    HSV_MODE,
  };

  void imageProcessing(cv::Mat&                 _input_img,
                       const int&               _my_color,
                       const Processing_Mode&   _process_moudle);

  void bgrProcessing(const int& _my_color);
  void hsvProcessing(const int& _my_color);

  cv::Mat gray_img_;        // 灰度图
  cv::Mat bin_img_;         // 最终二值图
  cv::Mat bin_img_color_;   // 颜色二值图
  cv::Mat bin_img_color1_;  // 颜色二值图一
  cv::Mat bin_img_color2_;  // 颜色二值图二
  cv::Mat bin_img_gray_;    // 灰度二值图

  cv::Mat ele_ =
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

  // BGR
  std::vector<cv::Mat> split_img_;
  int                  average_th_;

  // HSV
  cv::Mat hsv_img_;
  cv::Mat trackbar_img_ = cv::Mat::zeros(1, 300, CV_8UC1);

 private:
  void findTarget(cv::Mat&                              _input_dst_img,
                  cv::Mat&                              _input_bin_img,
                  std::vector<abstract_target::Target>& _target_box);

  fan_armor::Detector      small_target_;             // 内轮廓
  abstract_blade::FanBlade big_target_;               // 外轮廓
  abstract_target::Target  candidated_target_;        // 当前检测得到的打击目标 (遍历使用)
  abstract_target::Target  current_target_;           // 当前检测打击目标 (现在)

  std::vector<abstract_target::Target> target_box_;   // 打击目标队列

  int action_cnt_;    // 已打击扇叶数
  int inaction_cnt_;  // 未击打扇叶数

  // 轮廓点集关系
  std::vector<std::vector<cv::Point>> contours_;
  std::vector<cv::Vec4i>              hierarchy_;

  // 小轮廓条件 (area and length)
  float small_rect_area_;
  float small_rect_length_;

  // 大轮廓条件 (area and length)
  float big_rect_area_;
  float big_rect_length_;

  cv::Mat target_trackbar_img_ = cv::Mat::zeros(1, 300, CV_8UC1);

  int small_target_aspect_ratio_max_int_;
  int small_target_aspect_ratio_min_int_;
  int area_ratio_max_int_;
  int area_ratio_min_int_;

 private:
  bool isFindTarget(cv::Mat&                             _input_img,
                   std::vector<abstract_target::Target>& _target_box);

 private:
  cv::Point2f findCircleR(cv::Mat&    _input_src_img,
                          cv::Mat&    _input_bin_img,
                          cv::Mat&    _dst_img,
                          const bool& _is_find_target);

  bool           is_circle_;           // 是否找到圆心
  cv::Point2f    delta_height_point_;  // 获取装甲板的高度点差
  cv::Point2f    roi_global_center_;   // RoI 圆心中点位置 (在原图中)
  cv::Mat        result_img_;          // 二值图
  cv::Mat        roi_img_;             // 截取原图
  basic_roi::RoI roi_tool_;            // RoI 截取工具

  abstract_center_r::Center_R              center_r_;          // 候选圆心 R
  std::vector<std::vector<cv::Point>>      contours_r_;        // 中心R的遍历点集
  cv::Point2f                              roi_local_center_;  // 截取 RoI 的图像中心点
  std::vector<abstract_center_r::Center_R> center_r_box_;      // 第一次筛选之后得到的待选中心R
  cv::Point2f                              final_center_r_;    // 最终圆心（假定/真实）

 private:
  void judgeCondition(const bool& _is_find_target);

  void calAngle();
  void calDirection();
  void calVelocity();

  int  getState();

  float current_angle_;         // 当前目标角度
  float last_angle_;            // 上一次目标角度
  float diff_angle_;            // 两帧之间角度差
  float last_diff_angle_;       // 上一帧扇叶移动的角度
  bool  is_change_blade_;       // 判断是否切换扇叶

  float filter_direction_;      // 第二次滤波方向
  int   final_direction_;       // 最终的方向
  int   last_final_direction_;  // 上一次最终的方向
  float current_direction_;     // 当前方向
  float last_direction_;        // 上一次方向
  int   find_cnt_;              // 发现目标次数
  float d_angle_;               // 滤波器系数
  int   confirm_cnt_;           // 记录达到条件次数
  bool  is_confirm_;            // 判断是否达到条件

  float  current_speed_;        // 当前转速
  double last_time_;            // 上一帧的时间

 private:
  float doPredict(const float& _bullet_velocity,
                  const bool&  _is_find_target);

  float fixedPredict(const float& _bullet_velocity);

  float current_radian_;         // 当前弧度
  float barrel_buff_botton_h_;   // 枪口到扇叶底部高度
  float target_buff_h_;          // 目标在扇叶上的高度
  float target_y_;               // 当前扇叶目标高度
  float target_x_;               // 当前扇叶目标到高台的直线距离
  float target_z_;               // 当前扇叶目标直线距离

  // 手动补偿值
  int   offset_angle_int_;
  float offset_angle_float_;

  float bullet_tof_;               // 子弹飞行时间
  float fixed_forecast_quantity_;  // 固定预测量 (扇叶的单帧移动量)
  float final_forecast_quantity_;  // 最终合成的预测量

 private:
  void calculateTargetPointSet(const float&              _predict_quantity,
                               const cv::Point2f&        _final_center_r,
                               std::vector<cv::Point2f>& _target_2d_point,
                               cv::Mat&                  _input_dst_img,
                               const bool&               _is_find_target);

  double theta_;             // 特殊的弧度
  float  final_angle_;       // 最终角度
  float  final_radian_;      // 最终弧度
  float  sin_calcu_;         // 计算的sin值
  float  cos_calcu_;         // 计算的cos值
  float  radio_;             // 轨迹圆半径
  cv::Point2f pre_center_;   // 最终预测点

 private:
  basic_pnp::PnP buff_pnp_ = basic_pnp::PnP(fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_994.xml"),
    fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  void updateLastData(const bool& _is_find_target);

 private:
  fps::FPS buff_fps_1_{"Part 1"};
  fps::FPS buff_fps_2_{"Part 2"};
  fps::FPS buff_fps_;
};

}  // namespace basic_buff
