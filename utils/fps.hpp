#pragma once

#include <fmt/core.h>
#include <opencv4/opencv2/opencv.hpp>
#include <string>

namespace fps {
class RM_FPS {
 public:
  RM_FPS();
  explicit RM_FPS(std::string _name);
  ~RM_FPS();
  void calculateFPS();
  void calculateFPSGlobal();
  void getTick();

  static double lastTime();

 private:
  void displayFPS() const;

  int           cnt;    //  计算次数
  double        time1;  //  记录第一次时间点
  double        time2;  //  记录第二次时间点
  double        time;   //  记录时间段
  static double last_time;
  double        fps;      //  帧率
  double        max;      //  最大帧率
  double        min;      //  最小帧率
  double        average;  //  平均帧率（去除前10帧）
  double        total;    //  总帧率

  std::string name_;
};

inline double RM_FPS::lastTime() { return last_time; }

/**
 * @brief Construct a new rm fps::rm fps object
 *
 */
RM_FPS::RM_FPS() {
  cnt     = 0;
  time1   = 0.0;
  time2   = 0.0;
  time    = 0.0;
  fps     = 0.0;
  max     = 0.0;
  min     = 99999999.0;
  average = 0.0;
  total   = 0.0;

  name_ = {"Global"};
}

RM_FPS::RM_FPS(std::string _name) {
  cnt     = 0;
  time1   = 0.0;
  time2   = 0.0;
  time    = 0.0;
  fps     = 0.0;
  max     = 0.0;
  min     = 99999999.0;
  average = 0.0;
  total   = 0.0;

  name_ = _name;
}

/**
 * @brief Destroy the rm fps::rm fps object
 *
 */
RM_FPS::~RM_FPS() {}

/**
 * @brief 显示帧率
 *
 */
void RM_FPS::displayFPS() const {
  std::cout << "   |**************************** " << std::endl;
  std::cout << "   |" << name_ << std::endl;
  std::cout << "   |"
            << "time = " << this->time * 1000 << "毫秒" << std::endl;
  std::cout << "   |"
            << "FPS = " << this->fps << std::endl;
  std::cout << "   |"
            << "min = " << this->min << " max = " << this->max << std::endl;
  std::cout << "   |"
            << "average = " << this->average * 1000 << std::endl;
  std::cout << "   |**************************** " << std::endl;
}

/**
 * @brief 获取时间点
 *
 */
void RM_FPS::getTick() { this->time1 = cv::getTickCount(); }

/**
 * @brief 计算帧率
 *
 */
void RM_FPS::calculateFPS() {
  this->time2 = cv::getTickCount();
  this->time  = (this->time2 - this->time1) / cv::getTickFrequency();
  this->fps   = 1.f / this->time;
  ++this->cnt;
  if (this->cnt > 10) {
    if (this->fps > this->max) {
      this->max = this->fps;
    }
    if (this->fps < this->min) {
      this->min = this->fps;
    }
    this->total += this->time;
    this->average = this->total / ((this->cnt) - 10);
  }

  this->displayFPS();
}

void RM_FPS::calculateFPSGlobal() {
  this->time2 = cv::getTickCount();
  this->time  = (this->time2 - this->time1) / cv::getTickFrequency();
  this->fps   = 1.f / this->time;
  ++this->cnt;
  if (this->cnt > 10) {
    if (this->fps > this->max) {
      this->max = this->fps;
    }
    if (this->fps < this->min) {
      this->min = this->fps;
    }
    this->total += this->time;
    this->average = this->total / ((this->cnt) - 10);
  }

  last_time = average;

  this->displayFPS();
}
}  // namespace fps
