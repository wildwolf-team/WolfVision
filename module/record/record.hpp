
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
  explicit Record(std::string record_path_, String path_in, cv::Size size);
  ~Record();
  int             Return_switch() { return switch_r; }
  void            Vision_judge(const cv::Mat input_img, int judge, int current_mode);
  int             com_uart_judge;  // 串口和电脑的切换
  int             Path_H;          // 读取xml文件路径
  int             mode_set;        // 视频/截图模式设置
  cv::VideoWriter writer;          // 写入对象
  cv::VideoWriter writer_uart;     // 串口写入对象
  int             Rmode_last;      // 视觉判断
  void            Change_Place(String change_path, int mode_vision);
  /******  视觉控制数据******************/
  int  Rmode_current  = S1;
  int  n              = 1;
  bool Recording_flag = false;
  int  Priority;           // 视觉串口优先级
  bool vision_up = false;  // 开始结束判断(视觉)

  /*******串口控制数据********************/
  bool uart_judge;            // 串口开始结束判断(uart)
  int  Rmode_last_uart = S1;  // 串口上次模式判断
  int  Rmode_current_uart;    // 串口当前模式判断
  bool Recording_flag_uart = false;
  bool vision_up_uart;     // 串口录制使能判断
  bool uart_lock = false;  // 串口信号锁

 private:
  String      xml_path;
  String      palce_change;                                 // 读取文件写入路径
  int         fps_r;                                        // 视频帧数限制
  int         switch_r;                                     // 录制开关
  int         fourcc_ = writer.fourcc('M', 'J', 'P', 'G');  // 写入格式
  char        filename_[200];                               // 图像存储空间命名
  cv::Size    size_;
  inline void Path_Maker(int p, String path = fmt::format("{}{}", CONFIG_FILE_PATH, "/record/recordpath_save.yaml")) {
    if (p == 1) {
      cv::FileStorage path_get(path, cv::FileStorage::READ);
      path_get["_PATH"] >> Path_H;
    }
    if (p == 2) {
      cv::FileStorage path_save(path, cv::FileStorage::WRITE);
      Path_H++;
      path_save << "_PATH" << Path_H;
    }
  }

 public:
  void RecordIng(cv::Mat Img);
};
}  // namespace Record_mode
