# Basic_Buff 使用手册

## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`basic_buff.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_center_r.hpp`   | 中心 R 对象类           |
| `abstract_target.hpp`     | 扇叶目标类              |
| `uart_serial.hpp`         | 串口数据类              |
| `basic_kalman.hpp`        | 卡尔曼滤波器            |
| `basic_pnp.hpp`           | PnP 云台角度解算器       |
| `basic_roi.hpp`           | Roi 工具类             |
| `fps.hpp`                 | 帧率计算工具类           |

### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit Detector(const std::string& _buff_config_path);`

参数解释：`_buff_config_path` 为 `buff_config.xml` 的路径地址
  
### 步骤三：函数说明

在模式选择后调用接口函数即可，接口函数重载了2种调用方式：

  ```cpp
  void runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info, uart::Write_Data& _send_info);

  uart::Write_Data runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info);
  ```

  参数解释：
  | 参数名          | 参数解释                                                           |
  | --------------- | ------------------------------------------------------------------ |
  | `_input_img`    | 相机输入的图像，如：`src_img`                                      |
  | `_receive_info` | 串口接收的结构体，提供信息给`RM_Buff`类,如：`color`                |
  | `_send_info`    | 串口发送的结构体，接口函数会对发送值进行修改，最终存储到该结构体中 |


### 步骤四：修改 Buff 中的 pnp 对象构造函数所需地址

在文件 `basic_buff.hpp` 中的对象 `buff_pnp_` ，将其所需的相机标定文件以及参数配置文件地址修改正确即可

## 二、自定义函数及参数说明

---

### 获取基本信息

  ```cpp
  /**
   * @brief 获取基本信息
   * @details 图像和颜色
   * @param[in]  _input_img       输入图像
   * @param[in]  _my_color        己方颜色
   */
  void getInput(cv::Mat& _input_img, const int& _my_color);
  ```

### 预处理执行函数

  ```cpp
  /**
   * @brief 预处理执行函数
   * @param[in]  _input_img       输入图像（src）
   * @param[out] _output_img      输出图像（bin）
   * @param[in]  _my_color        颜色参数
   * @param[in]  _process_moudle  预处理模式
   */
  void imageProcessing(cv::Mat& _input_img, cv::Mat& _output_img, const int& _my_color, const Processing_Moudle& _process_moudle);

  /**
   * @brief BGR颜色空间预处理
   * @param  _my_color        颜色参数
   */
  void bgrProcessing(const int& _my_color);

  /**
   * @brief HSV颜色空间预处理
   * @param  _my_color        颜色参数
   */
  void hsvProcessing(const int& _my_color);
  ```

### 查找目标

  ```cpp
  /**
   * @brief 查找目标
   * @param  _input_dst_img   输入图像
   * @param  _input_bin_img   输入二值图
   * @param  _target_box      输出目标容器
   */
  void findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box);
  ```

### 判断是否有目标

  ```cpp
  /**
   * @brief 判断是否有目标
   * @param[in]  _input_img       绘制未激活的目标
   * @param[in]  _target_box      扇叶目标容器
   * @return true                 发现目标
   * @return false                丢失目标
   */
  bool isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box);
  ```

### 查找圆心

  ```cpp
  /**
   * @brief  查找圆心
   * @param  _input_src_img   输入src原图
   * @param  _input_bin_img   输入bin二值图
   * @param  _dst_img         输入dst画板图
   * @param  _is_find_target  是否发现扇叶目标
   * @return cv::Point2f          返回圆心R中点坐标
   */
  cv::Point2f findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target);

  ```

### 计算运转状态值：速度、方向、角度

  ```cpp
  /**
   * @brief 计算运转状态值：速度、方向、角度
   * @param  _is_find_target  是否发现目标
   */
  void judgeCondition(const bool& _is_find_target);

  /**
   * @brief 计算角度和角度差
   */
  void calAngle();

  /**
   * @brief 计算转动方向
   * @details 1：顺时针 -1：逆时针 0：不转动
   */
  void calDirection();

  /**
   * @brief 获取风车转向
   * @return int
   */
  int getState();

  /**
   * @brief 计算当前扇叶转动速度
   */
  void calVelocity();
  ```

### 计算预测量

  ```cpp
  /**
   * @brief 计算预测量
   * @param[in]  _bullet_velocity 子弹速度
   * @param[in]  _is_find_target  是否发现目标
   * @return float 预测量
   */
  float doPredict(const float& _bullet_velocity, const bool& _is_find_target);

  /**
   * @brief 计算固定预测量
   * @param[in]  _bullet_velocity 子弹速度
   * @return float  预测量
   */
  float fixedPredict(const float& _bullet_velocity);

  ```

### 计算最终目标矩形顶点点集

  ```cpp
  /**
   * @brief 计算最终目标矩形顶点点集
   * @param  _predict_quantity 预测量
   * @param  _final_center_r   圆心坐标（src）
   * @param  _target_2d_point  目标矩形顶点容器
   * @param  _input_dst_img    输入画板
   * @param  _is_find_target   是否有目标
   */
  void calculateTargetPointSet(const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target);

  ```

### 计算云台角度

  ```cpp
  /**
   * @brief  手动计算云台角度
   * @param  _target_center   目标中心点
   * @param  _unit_pixel_length 成像像素元件长度 mm
   * @param  _image_size      图像大小
   * @param  _focal_length    相机焦距 mm
   * @return cv::Point2f 
   */
  cv::Point2f angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length);
  ```

### 更新上一帧数据

  ```cpp
  /**
   * @brief 更新上一帧数据
   * @param  _is_find_target  是否有目标
   */
  void updateLastData(const bool& _is_find_target);
  ```
