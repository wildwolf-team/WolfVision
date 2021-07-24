#include "record.hpp"

#include <iostream>

namespace Record_mode {
Record::Record() {}
Record::Record(std::string path_input, String path_in, cv::Size size) {
  xml_path = path_input;
  cv::FileStorage record(xml_path, cv::FileStorage::READ);

  record["RECORD_SWITCH"] >> switch_r;
  record["PATH_"] >> Path_H;
  record["FPS_RECORD"] >> fps_r;
  record["RECORD_MODE"] >> mode_set;
  record["VISION_ELECTRONIC_LOCK"] >> Priority;
  palce_change = path_in;
  size_        = size;
  record.release();
}
void Record::RecordIng(cv::Mat src_img_r) {
  if (mode_set == 1) {
    if (com_uart_judge == 1) {
      writer << src_img_r;
    } else {
      writer_uart << src_img_r;
    }
  }
  if (mode_set == 2) {
    if (cv::waitKey(1) == 's') {
      n += 1;
      cv::imwrite(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/" + std::to_string(n) + ".jpg"), src_img_r);
    }
  }
}

void Record::Change_Place(String path_, int mode_vision) {
  palce_change = path_;
  com_uart_judge = mode_vision;
  if (switch_r == 1) {
    if (mode_vision == 1) {
      writer.open(palce_change, fourcc_, fps_r, size_, true);
    } else {
      writer_uart.open(palce_change, fourcc_, fps_r, size_, true);
    }
  }
  if (writer.isOpened()) {
    fmt::print("ISRECORDING\n");
  } else {
    fmt::print("UNABLE TO RECORD\n");
  }
}

void Record::Vision_judge(const cv::Mat src_input, int judge_, int current_mode) {
  /****视觉控制部分***/
  if (Priority) {
    if (judge_ == 's') {
      vision_up     = true;
      Rmode_current = S5;
      if (!Recording_flag) {
        Change_Place(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/Record" + std::to_string(Path_H++) + ".avi"), 1);
      }
      Recording_flag = true;
    }

    if (judge_ == 'e') {
      Recording_flag = false;
      Rmode_current  = S1;
      vision_up      = false;
    }
    if (vision_up) {
      RecordIng(src_input);
    }
    if (!Recording_flag && Rmode_last == S5) {
      writer.release();
    }
    Rmode_last = Rmode_current;
  }

  /*******串口输入部分*********/
  if (current_mode == S5) {
    if (!uart_lock) {
      uart_judge = true;
      if (!Recording_flag_uart) {
        Change_Place(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/Record" + std::to_string(Path_H++) + ".avi"), 2);
        uart_lock = true;
      }
    }
  Recording_flag_uart = true;
  }
  if (current_mode != S5 && Rmode_last_uart == S5) {
    Recording_flag_uart = false;
    uart_judge          = false;
    uart_lock = false;
  }
  if (uart_judge) {
    RecordIng(src_input);
  }
  if (!Recording_flag_uart && Rmode_last_uart == S5 && !uart_lock) {
    uart_lock = false;
    writer.release();
  }
  Rmode_last_uart = current_mode;
}
Record::~Record() {}

}  // namespace Record_mode
