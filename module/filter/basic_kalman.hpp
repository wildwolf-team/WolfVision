#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>

namespace basic_kalman {

class RM_kalmanfilter {
 private:
  cv::KalmanFilter KF_;
  cv::Mat          measurement_matrix;

  float p;
  float x = 640;
  float y = 320;

  typedef struct {
    const int         QUEUE_NUM = 5 + 1;  // 队列大小+1
    float             top_angle;          // 上一刻陀螺仪数据
    float             top_angle_;         // 当前的陀螺仪数据
    float             differ;             // 前后时刻的差值
    int               get_top_times = 0;  // 陀螺仪数据获取次数
    std::queue<float> difference_queue;   // difference_队列
  } top_diff_Queue;
  top_diff_Queue  top_angle_differ;

 public:
  RM_kalmanfilter();
  ~RM_kalmanfilter();
  cv::Point2f predict_point(cv::Point2f _p);
  void    reset();

  float use_RM_KF(float top);

  float anti_range = 1.5;
};

class Kalman1 {
 public:
    Kalman1() {
        Q_ = 0.01f;
        R_ = 0.02f;
        t_ = 1.0f;
        x_ = 0.0f;
        p_ = 0.01f;
    }
    Kalman1(float Q, float R, float t, float x0, float p0) {
        Q_ = Q;
        R_ = R;
        t_ = t;
        x_ = x0;
        p_ = p0;
    }
    void setParam(int R, int Q, int t) {
        if (R < 1)
            R = 1;
        if (Q < 1)
            Q = 1;
        if (t < 1)
            t = 1;
        R_ = static_cast<float>(R*0.01f);
        Q_ = static_cast<float>(Q*0.01f);
        t_ = static_cast<float>(t);
    }
    float run(float data) {
       x_pre_ = x_;                                   // x(k|k-1) = AX(k-1|k-1)+BU(k)
        p_pre_ = p_ + Q_;                             // p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);                 // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data - x_pre_);          // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                      // p(k|k)   = (I-kg(k)H)P(k|k-1)
        return x_;
    }
    float merge_run(float data1, float data2) {
        x_pre_ = data1;
        p_pre_ = p_ + Q_;                             // p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);                 // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data2 - x_pre_);         // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                      // p(k|k)   = (I-kg(k)H)P(k|k-1)
        return x_;
    }

 public:
    float R_;
    float Q_;
    float p_pre_;
    float x_pre_;
    float x_;
    float p_;
    float kg_;
    float t_;
};

}  // namespace basic_kalman
