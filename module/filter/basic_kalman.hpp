/**
 * @file   basic_kalman.hpp
 * @author SHL (2694359979@qq.com)
 * @brief  卡尔曼滤波类
 * @date   2021-08-28
 * 
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 * 
 */
#pragma once

#include "abstract_kalman.hpp"

namespace basic_kalman {
class firstKalman {
 public:
  firstKalman() {
    Q_ = 0.01f;
    R_ = 0.02f;
    t_ = 1.0f;
    x_ = 0.0f;
    p_ = 0.01f;
  }
  /**
   * @brief  保护数据
   * 
   * @param Q   过程噪声
   * @param R   测量噪声
   * @param t   公式数据
   * @param x0  公式数据 
   * @param p0  公式数据
   * @author SHL 
   */
  firstKalman(float Q, float R, float t, float x0, float p0) {
    Q_ = Q;
    R_ = R;
    t_ = t;
    x_ = x0;
    p_ = p0;
  }

  /**
   * @brief 设置数据类型
   * 
   * @param Q    过程噪声
   * @param R    测量噪声
   * @param t    公式数据
   * @author     SHL
   */
  void setParam(int _Q, int _R, int _t) {
    if (_R < 1) _R = 1;
    if (_Q < 1) _Q = 1;
    if (_t < 1) _t = 1;
    R_ = static_cast<float>(_R) * 0.01f;
    Q_ = static_cast<float>(_Q) * 0.01f;
    t_ = static_cast<float>(_t);
  }
  /**
  * @brief               针对单个数据的一阶卡尔曼
  * 
  * @param _data         传入需要滤波的数据
  * @return float        返回滤波完毕的数据
  * @author SHL 
  */
  float run(float _data) {
    x_pre_ = x_;                               // x(k|k-1) = AX(k-1|k-1)+BU(k)
    p_pre_ = p_ + Q_;                          // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);           // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data - x_pre_);  // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;               // p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
  }
  /**
   * @brief  针对两个数据具有一定相关性的一阶卡尔曼
   * 
   * @param _data1     传入需要滤波的数据
   * @param _data2     传入需要滤波的数据
   * @return float     传出处理后数据
   * @author           SHL 
   */
  float mergeRun(float _data1, float _data2) {
    x_pre_ = _data1;
    p_pre_ = p_ + Q_;                           // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);            // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data2 - x_pre_);  // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;                // p(k|k)   = (I-kg(k)H)P(k|k-1)

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
