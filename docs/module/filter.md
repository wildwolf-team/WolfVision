# Basic_Kalman 使用手册

---

## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`basic_kalman.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_kalman.hpp`   | 卡尔曼基类           |

### 步骤二：实例化对象

实例化对象时调用构造函数：`firstKalman();`

  
### 步骤三：函数说明

调用接口函数即可，接口函数根据数据的数量提供了两种选择：

  ```cpp
  float run(float _data);
  float mergeRun(float _data1, float _data2);
   void setParam(int Q, int R, int t);
  ```

  参数解释：
  | 参数名          | 参数解释                                                           |
  | --------------- | ------------------------------------------------------------------ |
  | `_data`    | 传入滤波的数据                                      |
  | `_Q` |测量噪声               |
  | `_R`    | 过程噪声 |
  | `_t`    | 公式数据 |

## 二、自定义函数及参数说明

---

### 一阶滤波器
 ```cpp
/**
  * @brief        针对单个数据的一阶卡尔曼
  * 
  * @param _data  传入需要滤波的数据
  * @return float 返回滤波完毕的数据
  * @author       SHL 
  */
  float run(float _data) {
    x_pre_ = x_;                                // x(k|k-1) = AX(k-1|k-1)+BU(k)
    p_pre_ = p_ + Q_;                           // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);            // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data - x_pre_);   // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;                // p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
  }
  /**
   * @brief         针对两个数据具有一定相关性的一阶卡尔曼
   * 
   * @param _data1  传入需要滤波的数据
   * @param _data2  传入需要滤波的数据
   * @return float  传出处理后数据
   * @author        SHL 
   */
  float mergeRun(float _data1, float _data2) {
    x_pre_ = _data1;
    p_pre_ = p_ + Q_;                             // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_    = p_pre_ / (p_pre_ + R_);              // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_     = x_pre_ + kg_ * (_data2 - x_pre_);    // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_     = (1 - kg_) * p_pre_;                  // p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
  }
  ```
  ### 函数数据设置
  ```cpp
/**
   * @brief    设置数据类型
   * 
   * @param Q  过程噪声
   * @param R  测量噪声
   * @param t  公式数据
   * @author   SHL 
   */
  void setParam(int _Q, int _R, int _t) {
    if (_R < 1) _R = 1;
    if (_Q < 1) _Q = 1;
    if (_t < 1) _t = 1;
    R_ = static_cast<float>(_R) * 0.01f;
    Q_ = static_cast<float>(_Q) * 0.01f;
    t_ = static_cast<float>(_t);
  }
  ```