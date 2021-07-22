
/*
 * @name: record.hpp
 * @namespace: ooi
 * @class: Recordmode
 * @brief: Record the video during the match
 * @author @gcusms
 * @version 1.0.1
 * @date 2021-07-20
 */

#pragma once
#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>
#include <string>
typedef std::string String;

namespace Record_mode {
enum ChangeNext {
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
  explicit Record(const std::string record_path_, String path_in, cv::Size size);
  ~Record();
  int             Return_switch() { return switch_r; }
  int             Path_H;             // 读取xml文件路径
  int             mode_set;           // 视频/截图模式设置
  cv::VideoWriter writer;             // 写入对象
  int             Rmode_dafult = S5;  // 模式切换更新
  int             Rmode_last;
  int             Rmode_current  = S1;
  int             n              = 1;
  bool            Recording_flag = false;
  void            Change_Place(const String change_path);
  int             Priority;  // 视觉电控优先级
  bool          vidion_up;  // 开始结束判断

 private:
  String   palce_change;
  int      fps_r;                                        // 视频帧数限制
  int      switch_r;                                     // 录制开关
  int      fourcc_ = writer.fourcc('M', 'J', 'P', 'G');  // 写入格式
  char     filename_[200];                               // 图像存储空间命名
  cv::Size size_;

 public:
  void RecordIng(cv::Mat Img);
};
}  // namespace Record_mode
