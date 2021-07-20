
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
class Record {
 public:
  Record();
  explicit Record(const std::string record_path_, String path_in, cv::Size size);
  ~Record();
  int             Return_switch() { return switch_r; }
  int             Path_H;    // 读取xml文件路径
  int             mode_set;  // 视频/截图模式设置
  cv::VideoWriter writer;    // 写入对象

 private:
  int  n = 1;                                        // 图片路径计数变化
  int  fps_r;                                        // 视频帧数限制
  int  switch_r;                                     // 录制开关
  int  fourcc_ = writer.fourcc('M', 'J', 'P', 'G');  // 写入格式
  char filename_[200];                               // 图像存储空间命名
 public:
  void RecordIng(cv::Mat Img);
};
}  // namespace Record_mode
