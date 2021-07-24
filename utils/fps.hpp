#pragma once

#include <string>

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/core.hpp>

namespace fps {

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "fps");

class FPS {
 public:
  FPS() {
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

  explicit FPS(std::string _name) {
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

  ~FPS() = default;

  void calculateFPS() {
    time2 = cv::getTickCount();
    time  = (time2 - time1) / cv::getTickFrequency();
    fps   = 1.f / time;

    if (++cnt > 10) {
      if (fps > max) {
        max = fps;
      }
      if (fps < min) {
        min = fps;
      }

      total   += time;
      average  = total / (cnt - 10);
    }

    displayFPS();
  }

  void calculateFPSGlobal() {
    time2 = cv::getTickCount();
    time  = (time2 - time1) / cv::getTickFrequency();
    fps   = 1.f / time;

    if (++cnt > 10) {
      if (fps > max) {
        max = fps;
      }
      if (fps < min) {
        min = fps;
      }
      total += time;
      average = total / ((cnt) - 10);
    }

    last_time = average;

    displayFPS();
  }

  void          getTick()   { time1 = cv::getTickCount(); }
  static double lastTime()  { return last_time; }
  inline float  returnFps() { return time * 1000; }

 private:
  void displayFPS() const {
    fmt::print("[{}] {} FPS of current/min/max: {}, {}, {}, time of current/averge: {}, {}\n",
               idntifier,
               name_,
               fps,
               min,
               max,
               time * 1000,
               average * 1000);
  }

  int           cnt;      //  计算次数
  double        time1;    //  记录第一次时间点
  double        time2;    //  记录第二次时间点
  double        time;     //  记录时间段
  static double last_time;
  double        fps;      //  帧率
  double        max;      //  最大帧率
  double        min;      //  最小帧率
  double        average;  //  平均帧率（去除前10帧）
  double        total;    //  总帧率

  std::string name_;
};

}  // namespace fps
