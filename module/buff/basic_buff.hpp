/**
 * @file basic_buff.hpp
 * @author WCJ (1767851382@qq.com)
 * @brief 能量机关
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <algorithm>
#include <string>
#include <vector>

#include "abstract_center_r.hpp"
#include "abstract_target.hpp"
#include "devices/serial/uart_serial.hpp"
#include "filter/basic_kalman.hpp"
#include "module/angle_solve/basic_pnp.hpp"
#include "roi/basic_roi.hpp"
#include "utils/fps.hpp"

double fps::FPS::last_time;

// #define DEBUG_KALMAN
#define DEBUG_MANUAL
// #define DEBUG_STATIC
// #define DEBUG_BARREL_OFFSET

namespace basic_buff {

auto idntifier_green     = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "basic_buff");
auto idntifier_red       = fmt::format(fg(fmt::color::red) | fmt::emphasis::bold, "basic_buff");
auto idntifier_yellow    = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "basic_buff");
auto process_yellow      = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "processing");
auto target_yellow       = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "find_target");
auto center_yellow       = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "center_r");
auto judgement_yellow    = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "judgement");
auto predict_yellow      = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "predict");
auto final_target_yellow = fmt::format(fg(fmt::color::yellow) | fmt::emphasis::bold, "final_target");

struct Buff_Param {
  // BGR
  int RED_BUFF_GRAY_TH;
  int RED_BUFF_COLOR_TH;
  int BLUE_BUFF_GRAY_TH;
  int BLUE_BUFF_COLOR_TH;

  // HSV-red
  int H_RED_MAX;
  int H_RED_MIN;
  int S_RED_MAX;
  int S_RED_MIN;
  int V_RED_MAX;
  int V_RED_MIN;

  // HSV-blue
  int H_BLUE_MAX;
  int H_BLUE_MIN;
  int S_BLUE_MAX;
  int S_BLUE_MIN;
  int V_BLUE_MAX;
  int V_BLUE_MIN;

  // 筛选条件
  // 面积 MAX MIN 大 小
  int SMALL_TARGET_AREA_MAX;
  int SMALL_TARGET_AREA_MIN;
  int BIG_TARGET_AREA_MAX;
  int BIG_TARGET_AREA_MIN;
  // 周长 MAX MIN
  int SMALL_TARGET_Length_MIN;
  int SMALL_TARGET_Length_MAX;
  int BIG_TARGET_Length_MIN;
  int BIG_TARGET_Length_MAX;
  // 角度差 MAX MIN
  int DIFF_ANGLE_MAX;
  int DIFF_ANGLE_MIN;
  // 长宽比 MAX MIN
  float SMALL_TARGET_ASPECT_RATIO_MAX;
  float SMALL_TARGET_ASPECT_RATIO_MIN;
  // 面积比 MAX MIN
  float AREA_RATIO_MAX;
  float AREA_RATIO_MIN;
  // 圆心R距离小轮廓中心的距离系数
  float BIG_LENTH_R;

  // 圆心限制条件
  // 圆心roi矩形大小
  int CENTER_R_ROI_SIZE;

  // 圆心矩形比例
  float CENTER_R_ASPECT_RATIO_MIN;
  float CENTER_R_ASPECT_RATIO_MAX;

  // 圆心矩形面积
  int CENTER_R_AREA_MIN;
  int CENTER_R_AREA_MAX;

  // 滤波器系数
  float FILTER_COEFFICIENT;

  // 能量机关打击模型参数，详情见 buff_config.xml
  float BUFF_H;
  float BUFF_RADIUS;
  float PLATFORM_H;
  float BARREL_ROBOT_H;
  float TARGET_X;

  // 预测量的补偿（弧度）
  float OFFSET_FIXED_RADIAN;

  // 模型深度补偿（左半边比右半边距离要远）
  float OFFSET_TARGET_Z;

  // yaw 和 pitch 轴弹道补偿
  float OFFSET_ARMOR_YAW;
  float OFFSET_ARMOR_PITCH;

  // 手算pitch 轴弹道补偿
  float OFFSET_MANUAL_ARMOR_PITCH;
};

struct Buff_Ctrl {
  int IS_PARAM_ADJUSTMENT;
  int IS_SHOW_BIN_IMG;
  int PROCESSING_MODE;
};

struct Buff_Cfg {
  Buff_Param param;
  Buff_Ctrl  ctrl;
};

// 能量机关类 继承抽象类
class Detector {
 public:
  Detector() = default;
  explicit Detector(const std::string& _buff_config_path);

  ~Detector() = default;

  /**
   * @brief 总执行函数（接口）
   * @param  _input_img       输入图像
   * @param  _receive_info    串口接收结构体
   * @param  _send_info       串口发送结构体
   * @author WCJ
   */
  void runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info, uart::Write_Data& _send_info);

  /**
   * @brief 总执行函数（接口）
   * @param  _input_img       输入图像
   * @param  _receive_info    串口接收结构体
   * @return uart::Write_Data 串口发送结构体
   * @author WCJ
   */
  uart::Write_Data runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info);

 private:
  /**
   * @brief 获取参数更新结构体
   * @param[in]  _fs               文件对象
   * @author WCJ
   */
  void readBuffConfig(const cv::FileStorage& _fs);

  /**
   * @brief 获取基本信息
   * @details 图像和颜色
   * @param[in]  _input_img       输入图像
   * @param[in]  _my_color        己方颜色
   * @author WCJ
   */
  void getInput(cv::Mat& _input_img, const int& _my_color);

  /**
   * @brief 显示最终图像
   */
  void displayDst();

  // 类中全局变量
  cv::Mat                  src_img_;          // 输入原图
  cv::Mat                  dst_img_;          // 图像效果展示图
  std::vector<cv::Point2f> target_2d_point_;  // 目标二维点集
  float                    final_target_z_;   // 最终打击目标的深度信息（预测点）
  cv::RotatedRect          target_rect_;      // 目标矩形
  int                      my_color_;         // 当前自己的颜色
  Buff_Cfg                 buff_config_;      // 参数结构体

  bool is_find_last_target_;  // 上一帧是否发现目标 true：发现 false：未发现
  bool is_find_target_;       // 是否发现目标 true：发现 false：未发现

  abstract_target::Target last_target_;  //  上一个打击目标

 private:
  // 预处理
  /**
   * @brief 预处理的模式
   */
  enum Processing_Moudle {
    BGR_MODE,
    HSV_MODE,
  };

  /**
   * @brief 预处理执行函数
   * @param[in]  _input_img       输入图像（src）
   * @param[out] _output_img      输出图像（bin）
   * @param[in]  _my_color        颜色参数
   * @param[in]  _process_moudle  预处理模式
   * @author WCJ HZH RCX
   */
  void imageProcessing(cv::Mat& _input_img, cv::Mat& _output_img, const int& _my_color, const Processing_Moudle& _process_moudle);

  /**
   * @brief BGR颜色空间预处理
   * @param  _my_color        颜色参数
   * @author WCJ
   */
  void bgrProcessing(const int& _my_color);

  /**
   * @brief HSV颜色空间预处理
   * @param  _my_color        颜色参数
   * @author WCJ
   */
  void hsvProcessing(const int& _my_color);

  cv::Mat gray_img_;        // 灰度图
  cv::Mat bin_img_;         // 最终二值图
  cv::Mat bin_img_color_;   // 颜色二值图
  cv::Mat bin_img_color1_;  // 颜色二值图一
  cv::Mat bin_img_color2_;  // 颜色二值图二
  cv::Mat bin_img_gray_;    // 灰度二值图

  cv::Mat ele_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));  // 椭圆内核

// BGR
#ifdef DEBUG_STATIC
  static std::vector<cv::Mat> split_img_;  // 分通道图
#else
  std::vector<cv::Mat> split_img_;  // 分通道图
#endif              // DEBUG_STATIC
  int average_th_;  // 平均阈值

  // HSV
  cv::Mat hsv_img_;  // hsv预处理输入图

#ifndef RELEASE
  // 滑动条窗口
  cv::Mat trackbar_img_ = cv::Mat::zeros(1, 300, CV_8UC1);  // 预处理
#endif                                                      // !RELEASE

 private:
  // 查找目标
  /**
   * @brief 查找目标
   * @param  _input_dst_img   输入图像
   * @param  _input_bin_img   输入二值图
   * @param  _target_box      输出目标容器
   * @author WCJ HZH
   */
  void findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box);

  fan_armor::Detector      small_target_;  // 内轮廓
  abstract_blade::FanBlade big_target_;    // 外轮廓

  abstract_target::Target              candidated_target_;  // 当前检测得到的打击目标（遍历使用）
  abstract_target::Target              current_target_;     // 当前检测打击目标（现在）
  std::vector<abstract_target::Target> target_box_;         // 打击目标队列

  int action_cnt_;    // 已打击扇叶数
  int inaction_cnt_;  // 未击打扇叶数

  // 轮廓点集 轮廓关系
  std::vector<std::vector<cv::Point>> contours_;
  std::vector<cv::Vec4i>              hierarchy_;

  // 小轮廓条件(area and length)
  float small_rect_area_;
  float small_rect_length_;

  // 大轮廓条件(area and length)
  float big_rect_area_;
  float big_rect_length_;

#ifndef RELEASE
  // 滑动条窗口
  cv::Mat target_trackbar_img_ = cv::Mat::zeros(1, 300, CV_8UC1);

  int small_target_aspect_ratio_max_int_;
  int small_target_aspect_ratio_min_int_;
  int area_ratio_max_int_;
  int area_ratio_min_int_;
#endif  // !RELEASE

 private:
  /**
   * @brief 判断是否有目标
   * @param[in]  _input_img       绘制未激活的目标
   * @param[in]  _target_box      扇叶目标容器
   * @return true                 发现目标
   * @return false                丢失目标
   * @author WCJ
   */
  bool isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box);

 private:
  /**
   * @brief  查找圆心
   * @param  _input_src_img   输入src原图
   * @param  _input_bin_img   输入bin二值图
   * @param  _dst_img         输入dst画板图
   * @param  _is_find_target  是否发现扇叶目标
   * @return cv::Point2f          返回圆心R中点坐标
   * @author WCJ HZH
   */
  cv::Point2f findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target);

  bool           is_circle_;           // 是否找到圆心
  cv::Point2f    delta_height_point_;  // 获取装甲板的高度点差
  cv::Point2f    roi_global_center_;   // roi 圆心中点位置(在原图中)
  cv::Mat        result_img_;          // 二值图
  cv::Mat        roi_img_;             // 截取原图
  basic_roi::RoI roi_tool_;            // roi截取工具

  abstract_center_r::Center_R              center_r_;          // 候选圆心R
  std::vector<std::vector<cv::Point>>      contours_r_;        // 中心R的遍历点集
  cv::Point2f                              roi_local_center_;  // 截取roi的图像中心点(在roi_img中)
  std::vector<abstract_center_r::Center_R> center_r_box_;      // 第一次筛选之后得到的待选中心R
  cv::Point2f                              final_center_r_;    // 最终圆心（假定/真实）

 private:
  // 计算运转状态值：速度、方向、角度
  /**
   * @brief 计算运转状态值：速度、方向、角度
   * @param  _is_find_target  是否发现目标
   * @author WCJ
   */
  void judgeCondition(const bool& _is_find_target);

  /**
   * @brief 计算角度和角度差
   * @author WCJ
   */
  void calAngle();

  /**
   * @brief 计算转动方向
   * @details 1：顺时针 -1：逆时针 0：不转动
   * @author WCJ HZH
   */
  void calDirection();

  /**
   * @brief 获取风车转向
   * @return int
   * @author WCJ
   */
  int getState();

  /**
   * @brief 计算当前扇叶转动速度
   * @author WCJ
   */
  void calVelocity();

  // 角度
  float current_angle_;         // 当前目标角度
  float last_angle_;            // 上一次目标角度
  float diff_angle_;            // 两帧之间角度差
  float last_diff_angle_;       // 上一帧扇叶移动的角度
  float last_last_diff_angle_;  // 上上帧扇叶移动的角度
  bool  is_change_blade_;       // 判断是否切换扇叶

  // 方向
  float filter_direction_;      // 第二次滤波方向
  int   final_direction_;       // 最终的方向
  int   last_final_direction_;  // 上一次最终的方向
  float current_direction_;     // 当前方向
  float last_direction_;        // 上一次方向
  int   find_cnt_;              // 发现目标次数
  float d_angle_;               // 滤波器系数
  int   confirm_cnt_;           // 记录达到条件次数
  bool  is_confirm_;            // 判断是否达到条件

  // 速度
  float  current_speed_;   // 当前转速
  double last_time_;       // 上一帧的时间
  double last_last_time_;  // 上上帧的时间

 private:
  // 计算预测量

  /**
   * @brief 计算预测量
   * @param[in]  _bullet_velocity 子弹速度
   * @param[in]  _is_find_target  是否发现目标
   * @return float 预测量
   * @author WCJ
   */
  float doPredict(const float& _bullet_velocity, const bool& _is_find_target);

  /**
   * @brief 计算固定预测量
   * @param[in]  _bullet_velocity 子弹速度
   * @return float  预测量
   * @author WCJ
   */
  float fixedPredict(const float& _bullet_velocity);

  // 变化预测量 TODO(fqjun)
  void mutativePredict(const float& _input_predict_quantity, float& _output_predict_quantity);

  float current_radian_;        // 当前弧度
  float barrel_buff_botton_h_;  // 枪口到扇叶底部高度
  float target_buff_h_;         // 目标在扇叶上的高度
  float target_y_;              // 当前扇叶目标高度
  float target_x_;              // 当前扇叶目标到高台的直线距离
  float target_z_;              // 当前扇叶目标直线距离

  float bullet_tof_;               // 子弹飞行时间
  float fixed_forecast_quantity_;  // 固定预测量（扇叶的单帧移动量）
  float final_forecast_quantity_;  // 最终合成的预测量

#ifdef DEBUG_KALMAN

 private:
  // 卡尔曼滤波器
  // basic_kalman::firstKalman buff_filter_ = basic_kalman::firstKalman(0.01f, 0.03f, 1.f, 0.f, 0.f);
  basic_kalman::firstKalman buff_filter_ = basic_kalman::firstKalman(0.02f, 0.03f, 1.f, 0.f, 0.f);

  int   Q                        = 3;
  int   R                        = 2;
  float current_predict_quantity = 0.f;

  cv::Mat kalman_trackbar_img_ = cv::Mat::zeros(1, 300, CV_8UC1);  // 预处理
  float   last_final_radian_   = 0.f;
#endif  // DEBUG_KALMAN

 private:
  // 计算获取最终目标（矩形、顶点）

  /**
   * @brief 计算最终目标矩形顶点点集
   * @param  _predict_quantity 预测量
   * @param  _final_center_r   圆心坐标（src）
   * @param  _target_2d_point  目标矩形顶点容器
   * @param  _input_dst_img    输入画板
   * @param  _is_find_target   是否有目标
   * @author WCJ
   */
  void calculateTargetPointSet(const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target);

  double      theta_;         // 特殊的弧度
  float       final_angle_;   // 最终角度
  float       final_radian_;  // 最终弧度
  float       sin_calcu_;     // 计算的sin值
  float       cos_calcu_;     // 计算的cos值
  cv::Point2f pre_center_;    // 最终预测点
  float       radio_;         // 轨迹圆半径

 private:
  // 计算云台角度

  basic_pnp::PnP buff_pnp_ = basic_pnp::PnP(fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_407.xml"), fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  // 手动计算云台角度
  /**
   * @brief  手动计算云台角度
   * @param  _target_center   目标中心点
   * @param  _unit_pixel_length 成像像素元件长度 mm
   * @param  _image_size      图像大小
   * @param  _focal_length    相机焦距 mm
   * @author WCJ
   * @return cv::Point2f 
   */
  cv::Point2f angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length);

 private:
  // 自动控制

 private:
  // 更新上一帧数据
  /**
   * @brief 更新上一帧数据
   * @param  _is_find_target  是否有目标
   * @author WCJ
   */
  void updateLastData(const bool& _is_find_target);

 private:
  // 帧率测试
  fps::FPS buff_fps_1_{"Part 1"};
  fps::FPS buff_fps_2_{"Part 2"};
  fps::FPS buff_fps_;
};

}  // namespace basic_buff
