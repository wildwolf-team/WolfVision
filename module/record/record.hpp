/**
 * @file record.hpp
 * @author SMS (2436210442@qq.com)
 * @brief 视频保存
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once
#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>
#include <string>

namespace RecordMode {

auto mes_sentry = fmt::format(fg(fmt::color::orange) | fmt::emphasis::bold, "SENTRY_AUTO_GO!!!");

enum ModeSet {
  S0,  // 模式模式dafult
  S1,  // 步兵自瞄
  S2,  // 大符模式
  S3,  // 击打哨兵模式
  S4,  // 小陀螺模式
  S5,  // 录制模式
  S6,  // 飞机模式
  S7,  // 哨兵模式
  S8,  // 雷达模式
};

class Record {
 public:
  Record();
  explicit Record(std::string record_path_, std::string path_in, cv::Size size);
  ~Record();
  /**
   * @brief  图像录制函数
   * @param input_img              传入图像
   * @param judge                        是否录制判断
   * @param current_mode     当前模式输入
   */
  void visionRecord(const cv::Mat input_img, int judge, int current_mode);

  int             path_ = 0;                                       // 路径计数
  int             last_mode_;                                 // 记录上次串口模式
  int             cnt_ = 0;                                         // 记录帧数
  cv::VideoWriter vw_image_;                   // 录制对象说明
  std::string            video_save_path_;    // 路径
  ModeSet               mode_;

 public:
  /**
  * @brief 录制函数
  * @param img_                        传入图像
  */
  void imgRecord(cv::Mat img_);
};
}  // namespace RecordMode
