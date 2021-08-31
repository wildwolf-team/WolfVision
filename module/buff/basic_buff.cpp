/**
 * @file basic_buff.cpp
 * @author WCJ (1767851382@qq.com)
 * @brief 能量机关
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */

#include "basic_buff.hpp"

namespace basic_buff {
#ifdef DEBUG_STATIC
std::vector<cv::Mat> Detector::split_img_;
#endif

Detector::Detector(const std::string& _buff_config_address) {
  // 读取buff配置文件
  cv::FileStorage buff_config_fs(_buff_config_address, cv::FileStorage::READ);
  readBuffConfig(buff_config_fs);
  buff_config_fs.release();

  target_2d_point_.reserve(4);
  final_target_z_ = 0.f;
  target_rect_    = cv::RotatedRect();
  my_color_       = 0;

  // 预处理
  split_img_.reserve(3);
  average_th_ = 0;

  // 查找目标
  action_cnt_        = 0;
  small_rect_area_   = 0.f;
  small_rect_length_ = 0.f;
  big_rect_area_     = 0.f;
  big_rect_length_   = 0.f;

#ifndef RELEASE
  small_target_aspect_ratio_max_int_ = buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MAX * 10;
  small_target_aspect_ratio_min_int_ = buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MIN * 10;
  area_ratio_max_int_                = buff_config_.param.AREA_RATIO_MAX * 100;
  area_ratio_min_int_                = buff_config_.param.AREA_RATIO_MIN * 100;
#endif  // !RELEASE

  // 判断目标是否为空
  is_find_target_ = false;

  // 计算运转状态值：速度、方向、角度
  current_angle_        = 0.f;
  last_angle_           = 0.f;
  diff_angle_           = 0.f;
  last_diff_angle_      = 0.f;
  last_last_diff_angle_ = 0.f;
  is_change_blade_      = false;

  current_direction_    = 0.f;
  last_direction_       = 0.f;
  last_final_direction_ = 0;
  find_cnt_             = 0;
  d_angle_              = 1.f;
  confirm_cnt_          = 0;
  is_confirm_           = false;

  current_speed_  = 0.f;
  last_time_      = 0.0;
  last_last_time_ = 0.0;

  // 计算预测量
  barrel_buff_botton_h_ = (buff_config_.param.BUFF_H - buff_config_.param.BUFF_RADIUS) - (buff_config_.param.PLATFORM_H + buff_config_.param.BARREL_ROBOT_H);
  current_radian_       = 0.f;
  target_buff_h_        = 0.f;
  target_y_             = 0.f;
  target_x_             = 0.f;

  target_z_                = 0.f;
  bullet_tof_              = 0.f;
  fixed_forecast_quantity_ = 0.f;
  final_forecast_quantity_ = 0.f;

  // 计算获取最终目标（矩形、顶点）
  theta_       = 0.0;
  final_angle_ = 0.f;
  sin_calcu_   = 0.f;
  cos_calcu_   = 0.f;
  pre_center_  = cv::Point2f(0.f, 0.f);
  radio_       = 0.f;

  // 计算云台角度
  // 输入串口数据
}

inline void Detector::getInput(cv::Mat& _input_img, const int& _my_color) {
  src_img_  = _input_img;
  my_color_ = _my_color;
  src_img_.copyTo(dst_img_);
  is_find_target_ = false;
}

inline void Detector::displayDst() { imshow("[basic_buff] displayDst() -> dst_img_", dst_img_); }

void Detector::runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info, uart::Write_Data& _send_info) {
  // 获取基本信息
  getInput(_input_img, _receive_info.my_color);

  // 预处理
  imageProcessing(src_img_, bin_img_, my_color_, static_cast<Processing_Moudle>(buff_config_.ctrl.PROCESSING_MODE));

  // 查找目标
  findTarget(dst_img_, bin_img_, target_box_);

  // 判断目标是否为空
  is_find_target_ = isFindTarget(dst_img_, target_box_);

  // 查找圆心
  final_center_r_ = findCircleR(src_img_, bin_img_, dst_img_, is_find_target_);

  // 计算运转状态值：速度、方向、角度
  judgeCondition(is_find_target_);

  // 计算预测量 单位为弧度
  final_forecast_quantity_ = doPredict(static_cast<float>(_receive_info.bullet_velocity), is_find_target_);

  // 计算获取最终目标（矩形、顶点）
  calculateTargetPointSet(final_forecast_quantity_, final_center_r_, target_2d_point_, dst_img_, is_find_target_);

  // 计算云台角度
  if (is_find_target_) {
    // 计算云台角度
    buff_pnp_.solvePnP(28, 2, target_2d_point_, final_target_z_);
    _send_info.yaw_angle   = buff_pnp_.returnYawAngle() + buff_config_.param.OFFSET_ARMOR_YAW;
    _send_info.pitch_angle = buff_pnp_.returnPitchAngle() + buff_config_.param.OFFSET_ARMOR_PITCH;
    _send_info.depth       = final_target_z_;
    _send_info.data_type   = is_find_target_;

    fmt::print("[{}] Info, yaw: {}, pitch: {}, depth: {}\n", idntifier_yellow, _send_info.yaw_angle, _send_info.pitch_angle, _send_info.depth);
  } else {
    _send_info = uart::Write_Data();
  }

  // TODO(fqjun) :自动控制
#ifndef RELEASE
  displayDst();
#endif  // !RELEASE

  // 更新上一帧数据
  updateLastData(is_find_target_);

  // 输入串口数据
}

uart::Write_Data Detector::runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info) {
  uart::Write_Data send_info;

  // 获取基本信息
  getInput(_input_img, _receive_info.my_color);

  // 预处理
  imageProcessing(src_img_, bin_img_, my_color_, static_cast<Processing_Moudle>(buff_config_.ctrl.PROCESSING_MODE));

  // 查找目标
  findTarget(dst_img_, bin_img_, target_box_);

  // 判断目标是否为空
  is_find_target_ = isFindTarget(dst_img_, target_box_);

  // 查找圆心
  final_center_r_ = findCircleR(src_img_, bin_img_, dst_img_, is_find_target_);

  // 计算运转状态值：速度、方向、角度
  judgeCondition(is_find_target_);

  // 计算预测量 单位为弧度
  final_forecast_quantity_ = doPredict(static_cast<float>(_receive_info.bullet_velocity), is_find_target_);

  // 计算获取最终目标（矩形、顶点）
  calculateTargetPointSet(final_forecast_quantity_, final_center_r_, target_2d_point_, dst_img_, is_find_target_);

  // 计算云台角度
  if (is_find_target_) {
    // 计算云台角度
    buff_pnp_.solvePnP(28, 2, target_2d_point_, final_target_z_);
#ifdef DEBUG_MANUAL
    // send_info.yaw_angle = current_predict_quantity*100;
    // send_info.pitch_angle = final_forecast_quantity_*100;
    send_info.yaw_angle   = angleCalculation(pre_center_, 0.0048, src_img_.size(), 8).x;
    send_info.pitch_angle = angleCalculation(pre_center_, 0.0048, src_img_.size(), 8).y;
    cv::Point yaw_angle   = cv::Point(dst_img_.cols - 100, 60);
    cv::putText(dst_img_, std::to_string(send_info.yaw_angle), yaw_angle, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0), 1, 8, false);
    cv::Point pitch_angle = cv::Point(dst_img_.cols - 100, 70);
    cv::putText(dst_img_, std::to_string(send_info.pitch_angle), pitch_angle, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(25, 255, 255), 1, 8, false);

#else
    send_info.yaw_angle   = buff_pnp_.returnYawAngle() + buff_config_.param.OFFSET_ARMOR_YAW;
    send_info.pitch_angle = buff_pnp_.returnPitchAngle() + buff_config_.param.OFFSET_ARMOR_PITCH;
#endif  // DEBUG_MANUAL

    send_info.depth     = final_target_z_;
    send_info.data_type = is_find_target_;

    fmt::print("[{}] Info, yaw: {}, pitch: {}, depth: {}\n", idntifier_yellow, send_info.yaw_angle, send_info.pitch_angle, send_info.depth);
  } else {
    send_info             = uart::Write_Data();
  }

  // TODO(fqjun) :自动控制
#ifndef RELEASE
  displayDst();
#endif
  // 更新上一帧数据
  updateLastData(is_find_target_);

  return send_info;
}

void Detector::readBuffConfig(const cv::FileStorage& _fs) {
  // ctrl
  _fs["IS_SHOW_BIN_IMG"] >> buff_config_.ctrl.IS_SHOW_BIN_IMG;
  _fs["PROCESSING_MODE"] >> buff_config_.ctrl.PROCESSING_MODE;
  _fs["IS_PARAM_ADJUSTMENT"] >> buff_config_.ctrl.IS_PARAM_ADJUSTMENT;

  // BGR
  _fs["RED_BUFF_GRAY_TH"] >> buff_config_.param.RED_BUFF_GRAY_TH;
  _fs["RED_BUFF_COLOR_TH"] >> buff_config_.param.RED_BUFF_COLOR_TH;
  _fs["BLUE_BUFF_GRAY_TH"] >> buff_config_.param.BLUE_BUFF_GRAY_TH;
  _fs["BLUE_BUFF_COLOR_TH"] >> buff_config_.param.BLUE_BUFF_COLOR_TH;

  // HSV-red
  _fs["H_RED_MAX"] >> buff_config_.param.H_RED_MAX;
  _fs["H_RED_MIN"] >> buff_config_.param.H_RED_MIN;
  _fs["S_RED_MAX"] >> buff_config_.param.S_RED_MAX;
  _fs["S_RED_MIN"] >> buff_config_.param.S_RED_MIN;
  _fs["V_RED_MAX"] >> buff_config_.param.V_RED_MAX;
  _fs["V_RED_MIN"] >> buff_config_.param.V_RED_MIN;

  // HSV-blue
  _fs["H_BLUE_MAX"] >> buff_config_.param.H_BLUE_MAX;
  _fs["H_BLUE_MIN"] >> buff_config_.param.H_BLUE_MIN;
  _fs["S_BLUE_MAX"] >> buff_config_.param.S_BLUE_MAX;
  _fs["S_BLUE_MIN"] >> buff_config_.param.S_BLUE_MIN;
  _fs["V_BLUE_MAX"] >> buff_config_.param.V_BLUE_MAX;
  _fs["V_BLUE_MIN"] >> buff_config_.param.V_BLUE_MIN;

  // area
  _fs["SMALL_TARGET_AREA_MAX"] >> buff_config_.param.SMALL_TARGET_AREA_MAX;
  _fs["SMALL_TARGET_AREA_MIN"] >> buff_config_.param.SMALL_TARGET_AREA_MIN;
  _fs["BIG_TARGET_AREA_MAX"] >> buff_config_.param.BIG_TARGET_AREA_MAX;
  _fs["BIG_TARGET_AREA_MIN"] >> buff_config_.param.BIG_TARGET_AREA_MIN;

  // length
  _fs["SMALL_TARGET_Length_MAX"] >> buff_config_.param.SMALL_TARGET_Length_MAX;
  _fs["SMALL_TARGET_Length_MIN"] >> buff_config_.param.SMALL_TARGET_Length_MIN;
  _fs["BIG_TARGET_Length_MAX"] >> buff_config_.param.BIG_TARGET_Length_MAX;
  _fs["BIG_TARGET_Length_MIN"] >> buff_config_.param.BIG_TARGET_Length_MIN;

  // diff_angle
  _fs["DIFF_ANGLE_MAX"] >> buff_config_.param.DIFF_ANGLE_MAX;
  _fs["DIFF_ANGLE_MIN"] >> buff_config_.param.DIFF_ANGLE_MIN;

  // aspect_ratio
  _fs["SMALL_TARGET_ASPECT_RATIO_MAX"] >> buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MAX;
  _fs["SMALL_TARGET_ASPECT_RATIO_MIN"] >> buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MIN;

  // area_ratio
  _fs["AREA_RATIO_MAX"] >> buff_config_.param.AREA_RATIO_MAX;
  _fs["AREA_RATIO_MIN"] >> buff_config_.param.AREA_RATIO_MIN;

  // center_r
  _fs["BIG_LENTH_R"] >> buff_config_.param.BIG_LENTH_R;

  _fs["CENTER_R_ROI_SIZE"] >> buff_config_.param.CENTER_R_ROI_SIZE;

  _fs["CENTER_R_ASPECT_RATIO_MIN"] >> buff_config_.param.CENTER_R_ASPECT_RATIO_MIN;
  _fs["CENTER_R_ASPECT_RATIO_MAX"] >> buff_config_.param.CENTER_R_ASPECT_RATIO_MAX;
  buff_config_.param.CENTER_R_ASPECT_RATIO_MIN *= 0.01;
  buff_config_.param.CENTER_R_ASPECT_RATIO_MAX *= 0.01;

  _fs["CENTER_R_AREA_MIN"] >> buff_config_.param.CENTER_R_AREA_MIN;
  _fs["CENTER_R_AREA_MAX"] >> buff_config_.param.CENTER_R_AREA_MAX;

  // filter coefficient
  _fs["FILTER_COEFFICIENT"] >> buff_config_.param.FILTER_COEFFICIENT;

  // 能量机关打击模型参数
  _fs["BUFF_H"] >> buff_config_.param.BUFF_H;
  _fs["BUFF_RADIUS"] >> buff_config_.param.BUFF_RADIUS;
  _fs["PLATFORM_H"] >> buff_config_.param.PLATFORM_H;
  _fs["BARREL_ROBOT_H"] >> buff_config_.param.BARREL_ROBOT_H;
  _fs["TARGET_X"] >> buff_config_.param.TARGET_X;

  // 固定预测值的补偿
  _fs["OFFSET_FIXED_RADIAN"] >> buff_config_.param.OFFSET_FIXED_RADIAN;

  // 模型深度补偿（左半边比右半边距离要远）
  _fs["OFFSET_TARGET_Z"] >> buff_config_.param.OFFSET_TARGET_Z;

  // yaw 和 pitch 轴弹道补偿
  _fs["OFFSET_ARMOR_YAW"] >> buff_config_.param.OFFSET_ARMOR_YAW;
  _fs["OFFSET_ARMOR_PITCH"] >> buff_config_.param.OFFSET_ARMOR_PITCH;
  buff_config_.param.OFFSET_ARMOR_YAW *= 0.01;
  buff_config_.param.OFFSET_ARMOR_PITCH *= 0.01;

  // 手算pitch 轴弹道补偿
  _fs["OFFSET_MANUAL_ARMOR_PITCH"] >> buff_config_.param.OFFSET_MANUAL_ARMOR_PITCH;
  buff_config_.param.OFFSET_MANUAL_ARMOR_PITCH *= 0.01;

  // 输出提示
  fmt::print("✔️ ✔️ ✔️ 🌈 能量机关初始化参数 读取成功 🌈 ✔️ ✔️ ✔️\n");
}

void Detector::imageProcessing(cv::Mat& _input_img, cv::Mat& _output_img, const int& _my_color, const Processing_Moudle& _process_moudle) {
  //  更新灰度图
  cvtColor(_input_img, gray_img_, cv::COLOR_BGR2GRAY);

  // 选择预处理的模式：BGR、HSV
  switch (_process_moudle) {
  case BGR_MODE: {
    fmt::print("[{}] Image pre-processing mode: +++ BGR_MODE +++\n", process_yellow);

    bgrProcessing(_my_color);

    break;
  }
  case HSV_MODE: {
    fmt::print("[{}] Image pre-processing mode: --- HSV_MODE ---\n", process_yellow);

    hsvProcessing(_my_color);

    break;
  }
  default: {
    fmt::print("[{}] Image pre-processing mode: === DEFAULT_MODE ===\n", process_yellow);

    bgrProcessing(_my_color);

    break;
  }
  }

// 显示各部分的二值图
#ifndef RELEASE
  if (buff_config_.ctrl.IS_SHOW_BIN_IMG == 1 && buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
    cv::imshow("[basic_buff] imageProcessing() -> bin_img_color_", bin_img_color_);
    cv::imshow("[basic_buff] imageProcessing() -> bin_img_gray_", bin_img_gray_);
  }
#endif  // !RELEASE

  // 求交集
  bitwise_and(bin_img_color_, bin_img_gray_, bin_img_);

  // 膨胀处理
  morphologyEx(bin_img_, bin_img_, cv::MORPH_DILATE, ele_);

// 显示最终合并的二值图
#ifndef RELEASE
  if (buff_config_.ctrl.IS_SHOW_BIN_IMG == 1 && buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
    cv::imshow("[basic_buff] imageProcessing() -> bin_img_final", bin_img_);
  }
#endif  // !RELEASE
}

void Detector::bgrProcessing(const int& _my_color) {
  // 分离通道
  cv::split(src_img_, split_img_);

  // 选择颜色
  switch (_my_color) {
  case uart::RED: {
    fmt::print("[{}] Image pre-processing color: RED\n", process_yellow);

    // my_color 为红色，则处理红色的情况 灰度图与 RGB 同样做红色处理
    cv::subtract(split_img_[2], split_img_[0], bin_img_color_);  // r-b

#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] bgrProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_RED:", window_name, &buff_config_.param.RED_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("COLOR_TH_RED:", window_name, &buff_config_.param.RED_BUFF_COLOR_TH, 255, nullptr);

      cv::imshow(window_name, trackbar_img_);
      fmt::print("[{}] BGR红色预处理调参面板已打开 \n", process_yellow);
    }

#endif  // !RELEASE

    // 亮度部分
    cv::threshold(gray_img_, bin_img_gray_, buff_config_.param.RED_BUFF_GRAY_TH, 255, cv::THRESH_BINARY);

    // 颜色部分
    cv::threshold(bin_img_color_, bin_img_color_, buff_config_.param.RED_BUFF_COLOR_TH, 255, cv::THRESH_BINARY);

    break;
  }
  case uart::BLUE: {
    fmt::print("[{}] Image pre-processing color: BLUE\n", process_yellow);

    // my_color为蓝色，则处理蓝色的情况 灰度图与RGB同样做蓝色处理
    cv::subtract(split_img_[0], split_img_[2], bin_img_color_);  // b-r

#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] bgrProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("COLOR_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_COLOR_TH, 255, nullptr);

      cv::imshow(window_name, trackbar_img_);
      fmt::print("[{}] BGR蓝色预处理调参面板已打开 \n", process_yellow);
    }
#endif  // !RELEASE

    // 亮度部分
    cv::threshold(gray_img_, bin_img_gray_, buff_config_.param.BLUE_BUFF_GRAY_TH, 255, cv::THRESH_BINARY);

    // 颜色部分
    cv::threshold(bin_img_color_, bin_img_color_, buff_config_.param.BLUE_BUFF_COLOR_TH, 255, cv::THRESH_BINARY);

    break;
  }
  default: {
    std::cout << "My color is default!" << std::endl;

    cv::subtract(split_img_[0], split_img_[2], bin_img_color1_);  // b-r
    cv::subtract(split_img_[2], split_img_[0], bin_img_color2_);  // r-b

#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] bgrProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_RED:", window_name, &buff_config_.param.RED_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("COLOR_TH_RED:", window_name, &buff_config_.param.RED_BUFF_COLOR_TH, 255, nullptr);
      cv::createTrackbar("GRAY_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("COLOR_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_COLOR_TH, 255, nullptr);

      cv::imshow(window_name, trackbar_img_);
      fmt::print("[{}] BGR红蓝两色预处理调参面板已打开 \n", process_yellow);
    }
#endif  // !RELEASE

    // 亮度部分
    average_th_ = static_cast<int>((buff_config_.param.RED_BUFF_GRAY_TH + buff_config_.param.BLUE_BUFF_GRAY_TH) * 0.5);
    cv::threshold(gray_img_, bin_img_gray_, average_th_, 255, cv::THRESH_BINARY);

    // 颜色部分
    cv::threshold(bin_img_color1_, bin_img_color1_, buff_config_.param.BLUE_BUFF_COLOR_TH, 255, cv::THRESH_BINARY);
    cv::threshold(bin_img_color2_, bin_img_color2_, buff_config_.param.RED_BUFF_COLOR_TH, 255, cv::THRESH_BINARY);

    // 求并集
    cv::bitwise_or(bin_img_color1_, bin_img_color2_, bin_img_color_);

    break;
  }
  }

  split_img_.clear();
  std::vector<cv::Mat>(split_img_).swap(split_img_);  // TODO(fqjun) :查看容量有多大
}

void Detector::hsvProcessing(const int& _my_color) {
  cvtColor(src_img_, hsv_img_, cv::COLOR_BGR2HSV_FULL);

  switch (_my_color) {
  case uart::RED:

    fmt::print("[{}] Image pre-processing color: RED\n", process_yellow);
#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] hsvProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_RED:", window_name, &buff_config_.param.RED_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("H_RED_MAX:", window_name, &buff_config_.param.H_RED_MAX, 360, nullptr);
      cv::createTrackbar("H_RED_MIN:", window_name, &buff_config_.param.H_RED_MIN, 360, nullptr);
      cv::createTrackbar("S_RED_MAX:", window_name, &buff_config_.param.S_RED_MAX, 255, nullptr);
      cv::createTrackbar("S_RED_MIN:", window_name, &buff_config_.param.S_RED_MIN, 255, nullptr);
      cv::createTrackbar("V_RED_MAX:", window_name, &buff_config_.param.V_RED_MAX, 255, nullptr);
      cv::createTrackbar("V_RED_MIN:", window_name, &buff_config_.param.V_RED_MIN, 255, nullptr);

      imshow(window_name, trackbar_img_);
      fmt::print("[{}] HSV红色预处理调参面板已打开 \n", process_yellow);
    }
#endif  // !RELEASE

    // 颜色部分

    cv::inRange(hsv_img_,
                cv::Scalar(buff_config_.param.H_RED_MIN, buff_config_.param.S_RED_MIN, buff_config_.param.V_RED_MIN),
                cv::Scalar(buff_config_.param.H_RED_MAX, buff_config_.param.S_RED_MAX, buff_config_.param.V_RED_MAX),
                bin_img_color_);

    // 亮度部分
    cv::threshold(gray_img_, bin_img_gray_, buff_config_.param.RED_BUFF_GRAY_TH, 255, cv::THRESH_BINARY);

    break;
  case uart::BLUE:
    fmt::print("[{}] Image pre-processing color: BLUE\n", process_yellow);

#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] hsvProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("H_BLUE_MAX:", window_name, &buff_config_.param.H_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("H_BLUE_MIN:", window_name, &buff_config_.param.H_BLUE_MIN, 255, nullptr);
      cv::createTrackbar("S_BLUE_MAX:", window_name, &buff_config_.param.S_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("S_BLUE_MIN:", window_name, &buff_config_.param.S_BLUE_MIN, 255, nullptr);
      cv::createTrackbar("V_BLUE_MAX:", window_name, &buff_config_.param.V_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("V_BLUE_MIN:", window_name, &buff_config_.param.V_BLUE_MIN, 255, nullptr);

      cv::imshow(window_name, trackbar_img_);
      fmt::print("[{}] HSV蓝色预处理调参面板已打开 \n", process_yellow);
    }
#endif  // !RELEASE

    // 颜色部分
    cv::inRange(hsv_img_,
                cv::Scalar(buff_config_.param.H_BLUE_MIN, buff_config_.param.S_BLUE_MIN, buff_config_.param.V_BLUE_MIN),
                cv::Scalar(buff_config_.param.H_BLUE_MAX, buff_config_.param.S_BLUE_MAX, buff_config_.param.V_BLUE_MAX),
                bin_img_color_);

    // 亮度部分
    cv::threshold(gray_img_, bin_img_gray_, buff_config_.param.BLUE_BUFF_GRAY_TH, 255, cv::THRESH_BINARY);

    break;
  default:
    fmt::print("[{}] Image pre-processing color: default\n", process_yellow);

#ifndef RELEASE
    if (buff_config_.ctrl.IS_PARAM_ADJUSTMENT == 1) {
      std::string window_name = {"[basic_buff] hsvProcessing() -> trackbar"};

      cv::namedWindow(window_name);
      cv::createTrackbar("GRAY_TH_RED:", window_name, &buff_config_.param.RED_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("H_RED_MAX:", window_name, &buff_config_.param.H_RED_MAX, 360, nullptr);
      cv::createTrackbar("H_RED_MIN:", window_name, &buff_config_.param.H_RED_MIN, 360, nullptr);
      cv::createTrackbar("S_RED_MAX:", window_name, &buff_config_.param.S_RED_MAX, 255, nullptr);
      cv::createTrackbar("S_RED_MIN:", window_name, &buff_config_.param.S_RED_MIN, 255, nullptr);
      cv::createTrackbar("V_RED_MAX:", window_name, &buff_config_.param.V_RED_MAX, 255, nullptr);
      cv::createTrackbar("V_RED_MIN:", window_name, &buff_config_.param.V_RED_MIN, 255, nullptr);

      cv::createTrackbar("GRAY_TH_BLUE:", window_name, &buff_config_.param.BLUE_BUFF_GRAY_TH, 255, nullptr);
      cv::createTrackbar("H_BLUE_MAX:", window_name, &buff_config_.param.H_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("H_BLUE_MIN:", window_name, &buff_config_.param.H_BLUE_MIN, 255, nullptr);
      cv::createTrackbar("S_BLUE_MAX:", window_name, &buff_config_.param.S_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("S_BLUE_MIN:", window_name, &buff_config_.param.S_BLUE_MIN, 255, nullptr);
      cv::createTrackbar("V_BLUE_MAX:", window_name, &buff_config_.param.V_BLUE_MAX, 255, nullptr);
      cv::createTrackbar("V_BLUE_MIN:", window_name, &buff_config_.param.V_BLUE_MIN, 255, nullptr);

      imshow(window_name, trackbar_img_);
      fmt::print("[{}] HSV红蓝两色预处理调参面板已打开 \n", process_yellow);
    }
#endif  // !RELEASE

    // 亮度部分
    average_th_ = static_cast<int>((buff_config_.param.RED_BUFF_GRAY_TH + buff_config_.param.BLUE_BUFF_GRAY_TH) * 0.5);
    cv::threshold(gray_img_, bin_img_gray_, average_th_, 255, cv::THRESH_BINARY);

    // 红色
    cv::inRange(hsv_img_,
                cv::Scalar(buff_config_.param.H_RED_MIN, buff_config_.param.S_RED_MIN, buff_config_.param.V_RED_MIN),
                cv::Scalar(buff_config_.param.H_RED_MAX, buff_config_.param.S_RED_MAX, buff_config_.param.V_RED_MAX),
                bin_img_color2_);
    // 蓝色
    cv::inRange(hsv_img_,
                cv::Scalar(buff_config_.param.H_BLUE_MIN, buff_config_.param.S_BLUE_MIN, buff_config_.param.V_BLUE_MIN),
                cv::Scalar(buff_config_.param.H_BLUE_MAX, buff_config_.param.S_BLUE_MAX, buff_config_.param.V_BLUE_MAX),
                bin_img_color1_);

    // 求并集
    cv::bitwise_or(bin_img_color1_, bin_img_color2_, bin_img_color_);
    break;
  }
}

void Detector::findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box) {
  cv::findContours(_input_bin_img, contours_, hierarchy_, 2, cv::CHAIN_APPROX_NONE);

  for (size_t i = 0; i != contours_.size(); ++i) {
    // 用于寻找小轮廓，没有父轮廓的跳过，以及不满足6点拟合椭圆
    if (hierarchy_[i][3] < 0 || contours_[i].size() < 6 || contours_[static_cast<uint>(hierarchy_[i][3])].size() < 6) {
      continue;
    }

    // 小轮廓周长条件
    small_rect_length_ = cv::arcLength(contours_[i], true);
    if (small_rect_length_ < buff_config_.param.SMALL_TARGET_Length_MIN) {
      continue;
    }

    // 小轮廓面积条件
    small_rect_area_ = cv::contourArea(contours_[i]);
    if (small_rect_area_ < buff_config_.param.SMALL_TARGET_AREA_MIN || small_rect_area_ > buff_config_.param.SMALL_TARGET_AREA_MAX) {
      continue;
    }

    // // 大轮廓周长条件
    // big_rect_length_ =
    // cv::arcLength(contours_[static_cast<uint>(hierarchy_[i][3])], true); if
    // (big_rect_length_ < buff_config_.param.BIG_TARGET_Length_MIN)
    // {
    //   continue;
    // }

    // 大轮廓面积条件
    big_rect_area_ = cv::contourArea(contours_[static_cast<uint>(hierarchy_[i][3])]);
    if (big_rect_area_ < buff_config_.param.BIG_TARGET_AREA_MIN || big_rect_area_ > buff_config_.param.BIG_TARGET_AREA_MAX) {
      continue;
    }

    // 保存扇叶和装甲板的数据
    small_target_.inputParams(contours_[i]);
    big_target_.inputParams(contours_[static_cast<uint>(hierarchy_[i][3])]);
    candidated_target_.inputParams(big_target_, small_target_);

    // 组合判断角度差
    if (candidated_target_.diffAngle() >= buff_config_.param.DIFF_ANGLE_MAX || candidated_target_.diffAngle() <= buff_config_.param.DIFF_ANGLE_MIN) {
      continue;
    }

    // 判断内轮廓的长宽比是否正常
    if (candidated_target_.Armor().aspectRatio() >= buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MAX || candidated_target_.Armor().aspectRatio() <= buff_config_.param.SMALL_TARGET_ASPECT_RATIO_MIN) {
      continue;
    }

    // 判断内外轮廓的面积比是否正常
    if (candidated_target_.areaRatio() <= buff_config_.param.AREA_RATIO_MIN || candidated_target_.areaRatio() >= buff_config_.param.AREA_RATIO_MAX) {
      continue;
    }

#ifndef RELEASE
    small_target_.displayFanArmor(_input_dst_img);
    big_target_.displayFanBlade(_input_dst_img);
#endif  // !RELEASE

    // 设置默认扇叶状态
    candidated_target_.setType(abstract_object::ACTION);
    // 更新装甲板的四个顶点编号
    candidated_target_.updateVertex(_input_dst_img);
    // 更新扇叶状态
    candidated_target_.setType(_input_bin_img, _input_dst_img);

    _target_box.push_back(candidated_target_);
  }
  fmt::print("[{}] 扇叶数量: {}\n", target_yellow, _target_box.size());
}

bool Detector::isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box) {
  if (_target_box.size() < 1) {
    fmt::print("[{}] Info, XXX no target detected XXX \n", target_yellow);

    current_target_ = abstract_target::Target();
    contours_.clear();
    hierarchy_.clear();
    _target_box.clear();

    std::vector<std::vector<cv::Point>>(contours_).swap(contours_);
    std::vector<cv::Vec4i>(hierarchy_).swap(hierarchy_);
    std::vector<abstract_target::Target>(_target_box).swap(_target_box);

    return false;
  }

  action_cnt_   = 0;
  inaction_cnt_ = 0;

  // 遍历容器获取未激活目标
  for (auto iter = _target_box.begin(); iter != _target_box.end(); ++iter) {
    if (iter->Type() != abstract_object::INACTION) {
      // TODO(fqjun) 测试是否会多或者少了几个对象，如果没有顺序的话，有可能第一个就退出了

      ++action_cnt_;
      continue;
    }

    ++inaction_cnt_;
    // 获取到未激活对象后退出遍历 TODO(fqjun) 查看是否筛选出想要的内容
    current_target_ = *iter;
#ifndef RELEASE
    current_target_.displayInactionTarget(_input_img);
#endif  // RELEASE
  }

  fmt::print("[{}] 未击打数量: {},  已击打数量: {}\n", target_yellow, inaction_cnt_, action_cnt_);

  // 清除容器
  contours_.clear();
  hierarchy_.clear();
  _target_box.clear();

  std::vector<std::vector<cv::Point>>(contours_).swap(contours_);
  std::vector<cv::Vec4i>(hierarchy_).swap(hierarchy_);
  std::vector<abstract_target::Target>(_target_box).swap(_target_box);

  if (inaction_cnt_ > 0) {
    return true;
  } else {
    return false;
  }
}

cv::Point2f Detector::findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target) {
  // 更新图像
  _input_src_img.copyTo(roi_img_);
  _input_bin_img.copyTo(result_img_);

  cv::Point2f center_r_point2f = cv::Point2f(0.f, 0.f);

  // 若没有扇叶目标则提前退出
  if (!(_is_find_target)) {
    is_circle_        = false;
    roi_local_center_ = cv::Point2f(0.f, 0.f);

    // 清理容器
    center_r_box_.clear();
    std::vector<abstract_center_r::Center_R>(center_r_box_).swap(center_r_box_);
    return center_r_point2f;
  }

  // 计算圆心大概位置
  delta_height_point_ = current_target_.deltaPoint();
  roi_global_center_  = current_target_.Armor().Rect().center - buff_config_.param.BIG_LENTH_R * delta_height_point_;

  // roi中心安全条件
  if (roi_global_center_.x < 0 || roi_global_center_.y < 0 || roi_global_center_.x > _input_src_img.cols || roi_global_center_.y > _input_src_img.rows) {
    if (roi_global_center_.x < 0) {
      roi_global_center_.x = 1;
    }
    if (roi_global_center_.y < 0) {
      roi_global_center_.y = 1;
    }
    if (roi_global_center_.x > _input_src_img.cols) {
      roi_global_center_.x = _input_src_img.cols - 1;
    }
    if (roi_global_center_.y > _input_src_img.rows - 1) {
      roi_global_center_.y = _input_src_img.rows - 1;
    }
  }

  // 画出假定圆心的roi矩形
  cv::RotatedRect roi_R(roi_global_center_, cv::Size(buff_config_.param.CENTER_R_ROI_SIZE, buff_config_.param.CENTER_R_ROI_SIZE), 0);
  cv::Rect        roi = roi_R.boundingRect();

  // roi安全条件
  roi = roi_tool_.makeRectSafeTailor(_input_src_img, roi);

  // 截取roi大小的图像，并绘制截取区域
  result_img_ = roi_tool_.cutRoIRect(_input_bin_img, roi);
  roi_img_    = roi_tool_.cutRoIRect(_input_src_img, roi);
#ifndef RELEASE
  cv::rectangle(_dst_img, roi, cv::Scalar(0, 255, 200), 2, 8, 0);
#endif  // !RELEASE

  is_circle_ = false;

  // 更新roi的中心点
  roi_local_center_ = cv::Point2f(roi_img_.cols * 0.5, roi_img_.rows * 0.5);

  // 查找轮廓
  cv::findContours(result_img_, contours_r_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  fmt::print("[{}] 圆心目标遍历轮廓数量: {} \n", center_yellow, contours_r_.size());

  // 选择并记录合适的圆心目标
  for (size_t i = 0; i != contours_r_.size(); ++i) {
    if (contours_r_[i].size() < 6) {
      continue;
    }

    center_r_.inputParams(contours_r_[i], roi_img_);

    fmt::print("[{}] 矩形 {} 比例:{}\n", center_yellow, i, center_r_.aspectRatio());

    if (center_r_.aspectRatio() < buff_config_.param.CENTER_R_ASPECT_RATIO_MIN || center_r_.aspectRatio() > buff_config_.param.CENTER_R_ASPECT_RATIO_MAX) {
      continue;
    }

    fmt::print("[{}] 矩形 {} 面积:{}\n", center_yellow, i, center_r_.Rect().boundingRect().area());

    if (center_r_.Rect().boundingRect().area() < buff_config_.param.CENTER_R_AREA_MIN || center_r_.Rect().boundingRect().area() > buff_config_.param.CENTER_R_AREA_MAX) {
      continue;
    }

    fmt::print("[{}] Find center R target success !!!   ", center_yellow);
    fmt::print(" --》 矩形 {}  --》 Ratio: {} / Area: {} ", i, center_r_.aspectRatio(), center_r_.Rect().boundingRect().area());

    center_r_box_.push_back(center_r_);

#ifndef RELEASE
    for (int k = 0; k < 4; ++k) {
      cv::line(roi_img_, center_r_.Vertex(k), center_r_.Vertex((k + 1) % 4), cv::Scalar(0, 130, 255), 3);
    }
#endif  // !RELEASE
    fmt::print("\n");
  }

  fmt::print("[{}] 符合比例条件的有: {}\n", center_yellow, center_r_box_.size());

  // 如果没有圆心目标，则退出
  if (center_r_box_.size() < 1) {
    fmt::print("[{}] 圆心为:假定圆心 \n", center_yellow);
    is_circle_       = false;
    center_r_point2f = roi_global_center_;

#ifndef RELEASE
    // 画出小轮廓到假定圆心的距离线
    cv::line(_dst_img, current_target_.Armor().Rect().center, center_r_point2f, cv::Scalar(0, 0, 255), 2);
    // 画出假定圆心
    cv::circle(_dst_img, center_r_point2f, 2, cv::Scalar(0, 0, 255), 2, 8, 0);
#endif  // !RELEASE
  } else {
    std::sort(center_r_box_.begin(), center_r_box_.end(), [](abstract_center_r::Center_R& c1, abstract_center_r::Center_R& c2) { return c1.centerDist() < c2.centerDist(); });

    fmt::print("[{}] 圆心为:真实圆心 \n", center_yellow);
    is_circle_       = true;
    center_r_point2f = center_r_box_[0].Rect().center + roi_R.boundingRect2f().tl();

#ifndef RELEASE
    // 画出小轮廓到假定圆心的距离线
    cv::line(_dst_img, current_target_.Armor().Rect().center, center_r_point2f, cv::Scalar(0, 255, 0), 2);
    // 画出假定圆心
    cv::circle(_dst_img, center_r_point2f, 2, cv::Scalar(0, 0, 255), 2, 8, 0);
#endif  // !RELEASE
  }

  // 清理容器
  center_r_box_.clear();
  contours_r_.clear();

  std::vector<abstract_center_r::Center_R>(center_r_box_).swap(center_r_box_);
  std::vector<std::vector<cv::Point>>(contours_r_).swap(contours_r_);

  return center_r_point2f;
}

void Detector::judgeCondition(const bool& _is_find_target) {
  if (!(_is_find_target)) {
    // 没有目标，角度为上一帧的角度，方向重置为零，速度为0
    current_angle_ = last_target_.Angle();
    diff_angle_    = 0.f;
    current_speed_ = 0.f;

    return;
  }

  // 计算角度
  calAngle();

  // 计算方向
  calDirection();

  // 计算速度
  calVelocity();

  return;
}

void Detector::calAngle() {
  // 装甲板到圆心的连线所代表的角度
  current_angle_ = atan2((current_target_.Armor().Rect().center.y - final_center_r_.y), (current_target_.Armor().Rect().center.x - final_center_r_.x)) * 180 / static_cast<float>(CV_PI);

  // 过零处理
  if (current_angle_ < 0.f) {
    current_angle_ += 360.f;
  }

  // 角度差
  diff_angle_ = current_angle_ - last_angle_;

  // 过零处理
  if (diff_angle_ > 180) {
    diff_angle_ -= 360;
  } else if (diff_angle_ < -180) {
    diff_angle_ += 360;
  }

  fmt::print("[{}] 当前角度差为: {} 度\n", judgement_yellow, diff_angle_);

  if (fabs(diff_angle_) > 30.f) {
    is_change_blade_ = true;
    diff_angle_      = 0.f;
  } else {
    is_change_blade_ = false;
  }
  // TODO(fqjun) :当变化量大于 30°时，则是切换装甲板，则重置 diff 为 0，last 为当前。
}

void Detector::calDirection() {
  ++find_cnt_;

  if (find_cnt_ % 2 == 0) {
    current_direction_ = getState();
    filter_direction_  = (1 - 0.01) * last_direction_ + 0.01 * current_direction_;
    last_direction_    = filter_direction_;
  }

  if (find_cnt_ == 10) {
    find_cnt_ = 0;
  }

  // 显示当前转动信息
  if (filter_direction_ > 0.1) {
    fmt::print("[{}] 转动方向:顺时针转动\n", judgement_yellow);

#ifndef DEBUG_BARREL_OFFSET
    final_direction_ = 1;
#else
    final_direction_      = 0;
#endif

    last_final_direction_ = final_direction_;
  } else if (filter_direction_ < -0.1) {
    fmt::print("[{}] 转动方向:逆时针转动\n", judgement_yellow);

#ifndef DEBUG_BARREL_OFFSET
    final_direction_ = -1;
#else
    final_direction_      = 0;
#endif

    last_final_direction_ = final_direction_;
  } else {
    fmt::print("[{}] 转动方向:不转动\n", judgement_yellow);

    final_direction_ = last_final_direction_;
  }
}

int Detector::getState() {
  if (fabs(diff_angle_) < 10 && fabs(diff_angle_) > 1e-6) {
    d_angle_ = (1 - buff_config_.param.FILTER_COEFFICIENT) * d_angle_ + buff_config_.param.FILTER_COEFFICIENT * diff_angle_;
  }

  if (d_angle_ > 0) {
    return 1;
  } else if (d_angle_ < -0) {
    return -1;
  } else {
    return 0;
  }
}

void Detector::calVelocity() {
  double current_time       = buff_fps_.lastTime() + last_time_ + last_last_time_;
  float  current_diff_angle = diff_angle_ + last_diff_angle_ + last_last_diff_angle_;
  // 默认单位为角度/s
  if (current_time == 0) {
    current_speed_ = 0.f;
  } else {
    // 将单位转为rad/s
    current_speed_ = fabs(current_diff_angle / current_time * CV_PI / 180);
  }
  last_last_time_       = last_time_;
  last_time_            = buff_fps_.lastTime();
  last_last_diff_angle_ = last_diff_angle_;
  last_diff_angle_      = diff_angle_;

  fmt::print("[{}] 当前风车转速为: {} rad/s \n", judgement_yellow, current_speed_);
}

float Detector::doPredict(const float& _bullet_velocity, const bool& _is_find_target) {
#ifdef DEBUG_KALMAN
#  ifndef RELEASE
  std::string window_name = {"[basic_buff] kalman -> trackbar"};

  cv::namedWindow(window_name);
  cv::createTrackbar("Q*0.01:", window_name, &Q, 1000, nullptr);
  cv::createTrackbar("R*0.01:", window_name, &R, 1000, nullptr);

  cv::imshow(window_name, kalman_trackbar_img_);
#  endif  // !RELEASE

  buff_filter_.setParam(Q, R, Q);
#endif  // DEBUG_KALMAN
  // 判断是否发现目标，没有返回0，有则进行计算预测
  if (!(_is_find_target)) {
    target_z_ = 0.f;
    // 重置参数
    return 0.f;
  }

  float predict_quantity = 0.f;

  // 计算固定预测量 原来是给0.35弧度 TODO(fqjun) :测一下最快和最慢速度时的提前量，以确定范围 predict_quantity = fixedPredict(_bullet_velocity*1000);
  predict_quantity = fixedPredict(28 * 1000);  // 默认先给28m/s

  // 计算移动预测量 TODO(fqjun)

  fmt::print("[{}] Info, 提前了: {} 度 \n", predict_yellow, predict_quantity * 180 / CV_PI);

#ifdef DEBUG_KALMAN

  current_predict_quantity = predict_quantity;
  predict_quantity         = buff_filter_.run(predict_quantity);
#endif  // DEBUG_KALMAN

  return predict_quantity;
}

float Detector::fixedPredict(const float& _bullet_velocity) {
  // 转换为弧度
  current_radian_ = current_angle_ * CV_PI / 180;

  // 通过模型计算当前目标点的位置信息（原扇叶）
  // 计算能量机关的高度
  target_buff_h_ = 800 + sin(current_radian_ - CV_PI) * 800;

  target_y_ = target_buff_h_ + barrel_buff_botton_h_;

  // 计算能量机关的水平直线距离
  target_x_ = buff_config_.param.TARGET_X;

  // 计算能量机关目标的直线距离
  target_z_ = sqrt((target_y_ * target_y_) + (target_x_ * target_x_));
  // 通过模型计算当前目标点的位置信息（原扇叶）

  // 计算子弹飞行时间
  bullet_tof_ = (target_z_ + buff_config_.param.OFFSET_TARGET_Z) / _bullet_velocity;

  // 计算固定提前量（也可以直接给定）
  if (current_direction_ != 0) {
    fixed_forecast_quantity_ = current_speed_ * bullet_tof_ + buff_config_.param.OFFSET_FIXED_RADIAN;
  } else {
    fixed_forecast_quantity_ = 0.f;
  }

  return fixed_forecast_quantity_;
}

void Detector::mutativePredict(const float& _input_predict_quantity, float& _output_predict_quantity) {}

void Detector::calculateTargetPointSet(
  const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target) {
  // 判断有无目标，若无则重置参数并提前退出
  if (!(_is_find_target)) {
    _target_2d_point.clear();
    _target_2d_point = std::vector<cv::Point2f>(4, cv::Point2f(0.f, 0.f));
    // 重置参数
    return;
  }

  // 计算theta
  theta_ = current_radian_;

  if (theta_ < 0) {
    theta_ += (2 * CV_PI);
  }

  // 计算最终角度和弧度
  final_radian_ = theta_ + final_direction_ * _predict_quantity;

  final_angle_ = final_radian_ * 180 / CV_PI;

  // 计算sin和cos
  sin_calcu_ = sin(final_radian_);
  cos_calcu_ = cos(final_radian_);

  // 计算最终坐标点
  radio_        = abstract_object::centerDistance(_final_center_r, current_target_.Armor().Rect().center);
  pre_center_.x = radio_ * cos_calcu_ + _final_center_r.x;
  pre_center_.y = radio_ * sin_calcu_ + _final_center_r.y;

  // 计算最终目标的旋转矩形
  target_rect_ = cv::RotatedRect(pre_center_, current_target_.Armor().Rect().size, 90);

  // 通过模型计算最终目标点的位置信息（预测点）TODO:待优化
  // 计算能量机关的高度
  target_buff_h_ = 800 + sin(final_radian_ - CV_PI) * 800;
  target_y_      = target_buff_h_ + barrel_buff_botton_h_;

  // 计算能量机关目标的直线距离
  final_target_z_ = sqrt((target_y_ * target_y_) + (target_x_ * target_x_));
  // 通过模型计算最终目标点的位置信息（预测点）

  // 保存最终目标的顶点，暂时用的是排序点的返序存入才比较稳定，正确使用应为0123, pnp内已进行反向放置
  _target_2d_point.clear();
  cv::Point2f target_vertex[4];
  target_rect_.points(target_vertex);

  _target_2d_point.push_back(target_vertex[3]);
  _target_2d_point.push_back(target_vertex[2]);
  _target_2d_point.push_back(target_vertex[1]);
  _target_2d_point.push_back(target_vertex[0]);

#ifndef RELEASE
  // 绘制图像
  // 最终目标装甲板（预测值）
  for (int k = 0; k < 4; ++k) {
    cv::line(_input_dst_img, _target_2d_point[k], _target_2d_point[(k + 1) % 4], cv::Scalar(0, 130, 255),
             8);  // orange
  }

  cv::circle(_input_dst_img, _final_center_r, radio_, cv::Scalar(0, 255, 125), 2, 8, 0);                       // 轨迹圆
  cv::circle(_input_dst_img, pre_center_, 3, cv::Scalar(255, 0, 0), 3, 8, 0);                                  // 预测值的中点
  cv::line(_input_dst_img, pre_center_, _final_center_r, cv::Scalar(0, 255, 255), 2);                          // 预测点和圆心的连线
  cv::line(_input_dst_img, current_target_.Armor().Rect().center, _final_center_r, cv::Scalar(0, 255, 0), 2);  // 装甲板和圆心的连线

  // 顺时针表示顶点顺序,红黄蓝绿
  cv::circle(_input_dst_img, _target_2d_point[0], 10, cv::Scalar(0, 0, 255), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[1], 10, cv::Scalar(0, 255, 255), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[2], 10, cv::Scalar(255, 0, 0), -1, 8, 0);
  cv::circle(_input_dst_img, _target_2d_point[3], 10, cv::Scalar(0, 255, 0), -1, 8, 0);
  // 绘制图像
#endif  // RELEASE
}

cv::Point2f Detector::angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length) {
  cv::Point2f angle2f;

  float target_projection_x = fabs(_image_size.width * 0.5 - _target_center.x) * _unit_pixel_length;
  angle2f.x                 = atan2(target_projection_x, _focal_length) * 180 / CV_PI;
  if (_target_center.x <= (_image_size.width * 0.5)) {
    angle2f.x = -1 * angle2f.x;
  }

  float target_projection_y = fabs(_image_size.height * 0.5 - _target_center.y) * _unit_pixel_length;
  angle2f.y                 = atan2(target_projection_y, _focal_length) * 180 / CV_PI;
  if (_target_center.y <= (_image_size.height * 0.5)) {
    angle2f.y = -1 * angle2f.y;
  }
  angle2f.y += buff_config_.param.OFFSET_MANUAL_ARMOR_PITCH;

  return angle2f;
}

void Detector::updateLastData(const bool& _is_find_target) {
  if (!(_is_find_target)) {
    fmt::print("[{}] 没有目标，不需要更新上一帧数据 XXX\n", idntifier_yellow);
    is_find_last_target_ = _is_find_target;
    target_2d_point_.clear();
    std::vector<cv::Point2f>(target_2d_point_).swap(target_2d_point_);
    target_rect_ = cv::RotatedRect();
    return;
  }

  last_target_         = current_target_;
  last_angle_          = current_angle_;
  is_find_last_target_ = _is_find_target;
  target_2d_point_.clear();
  std::vector<cv::Point2f>(target_2d_point_).swap(target_2d_point_);
  target_rect_ = cv::RotatedRect();

  fmt::print("[{}] 发现目标，已更新上一帧数据 √√√\n", idntifier_yellow);
}

}  // namespace basic_buff
