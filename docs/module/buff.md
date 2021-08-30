# Basic_Buff 使用手册

## 一、使用说明

### 步骤一：头文件说明

- 包含头文件 `basic_buff.hpp`

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

在模式选择后调用接口函数即可，接口函数重载了 2 种调用方式：

  ```cpp
  void runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info, uart::Write_Data& _send_info);

  uart::Write_Data runTask(cv::Mat& _input_img, const uart::Receive_Data& _receive_info);
  ```

  参数解释：
  | 参数名          | 参数解释                                                           |
  | --------------- | ------------------------------------------------------------------ |
  | `_input_img`    | 相机输入的图像，如：`src_img`                                      |
  | `_receive_info` | 串口接收的结构体，提供信息给 `RM_Buff` 类,如：`color`                |
  | `_send_info`    | 串口发送的结构体，接口函数会对发送值进行修改，最终存储到该结构体中 |


### 步骤四：修改 Buff 中的 pnp 对象构造函数所需地址

在文件 `basic_buff.hpp` 中的对象 `buff_pnp_` ，将其所需的相机标定文件以及参数配置文件地址修改正确即可

## 二、自定义函数说明

### 获取基本信息

  ```cpp
  void getInput(cv::Mat& _input_img, const int& _my_color);
  ```

  设计思路：
  
  设计之初是为了将数据流和数据处理部分进行分离，方便后期进行筛查问题，在数据流确定了之后再进行数据处理

### 预处理执行函数

  ```cpp
  void imageProcessing(cv::Mat& _input_img, cv::Mat& _output_img, const int& _my_color, const Processing_Moudle& _process_moudle);

  void bgrProcessing(const int& _my_color);

  void hsvProcessing(const int& _my_color);
  ```

  设计思路：
  
  设计的思路是将预处理部分作为单独的模块，并细分成 BGR 模式和 HSV 模式，根据需求进行更改，使用的时候很方便地根据配置文件的标志位进行切换即可。

### 查找目标

  ```cpp
  void findTarget(cv::Mat& _input_dst_img, cv::Mat& _input_bin_img, std::vector<abstract_target::Target>& _target_box);
  ```

  设计思路：

  由于查找目标时大部分时间都是利用遍历的方式来进行筛选，所以设计提前退出的选项可以大大减少遍历次数，且使用 `continue` 也是为了让整体条件筛选的结构更加清晰，方便排查哪个条件出现问题，避免了出现大量 `if` 和 `{}` 造成的杂乱现象。各个条件的参数也利用外部配置文件进行控制，方便修改也方便阅读。每次进行筛选的时候都会将目标对象进行整理，利用自定义的数据类型来对目标进行属性记录，方便后续数据属性的调用和筛查。
  
  遇到的问题：

  设计条件的时候有一部分是利用限定具体数值，而不是特征间的比例关系，这一类的条件容易出现鲁棒性低的情况，所以后期是对条件进行了一定的剔除或改为比例关系的条件，防止赛场上出现炸程序的现象。
  
### 判断是否有目标

  ```cpp
  bool isFindTarget(cv::Mat& _input_img, std::vector<abstract_target::Target>& _target_box);
  ```

  设计思路：

  该函数是专门为了控制 `_is_find_target` 标志位的信息，为后续各个模块进行选择性提前退出，减少了单线程上不必要的运算。
    
### 查找圆心

  ```cpp
  cv::Point2f findCircleR(cv::Mat& _input_src_img, cv::Mat& _input_bin_img, cv::Mat& _dst_img, const bool& _is_find_target);
  ```

  设计思路：

  由于后续部分是基于目标存在来定位目标圆心的，所以设计了一个标志位的读取来判断程序是否需要提前退出。查找圆心是为了重新计算目标的运行轨迹，且圆心和目标扇叶是具有一定关系，所以圆心可以通过目标往圆心区域创建搜索区域，并对该区域进行针对性筛选即可得到目标圆心。并通过自定义类型来储存圆心的属性，方便后续调用。
  
  遇到的问题：

  ROI 的范围选取会比较影响圆心的筛选，所以 ROI 的区域大小从一开始的 90x90 拓展到了 180x180 ，有效地解决了在特殊的截取区域中迷惑目标出现的问题。
  
### 计算运转状态值：速度、方向、角度

  ```cpp
  void judgeCondition(const bool& _is_find_target);

  void calAngle();

  void calDirection();

  int getState();

  void calVelocity();
  ```

  设计思路：

  设计时也是考虑到后期调试和问题排查的效率问题，模块的分解能让阅读者更轻易地建立起程序结构图，另一方面也是防止模块间的耦合，导致程序难改的现象出现。考虑到程序帧率过高的问题，所以数据处理的时候会导致低频噪声被放大的现象，为了抑制噪声的出现，在状态值计算时会隔帧进行计算，降低计算频率，从而抑制低频噪声的出现。且由于数据的更新迭代过快，在判断运转方向的时候可能会出现目标减速时被误判成停止转动的情况，所以设计了一个一阶延时滤波器，调节参数后即可防止误判现象的出现。
  
  遇到的问题：

  一阶延时滤波器的参数调节需要根据法则来进行调整，并通过实机测试才可正常使用，需要使用者在延时性和状态更新速度上进行相应的取舍。
  
### 计算预测量

  ```cpp
  float doPredict(const float& _bullet_velocity, const bool& _is_find_target);

  float fixedPredict(const float& _bullet_velocity);
  ```

  设计思路：

  设计之初是将大小符的预测分开进行，所以才会有一个整体的预测函数和附属的预测量计算函数，后期发现两者皆可通用，则将大符部分的函数合并至 `fixedPredict` 中。设计时该函数作为预测量的计算器进行设计，目的是将预测量和原目标位置值进行解耦，方便后期排查。
  
### 计算最终目标矩形顶点点集

  ```cpp
  void calculateTargetPointSet(const float& _predict_quantity, const cv::Point2f& _final_center_r, std::vector<cv::Point2f>& _target_2d_point, cv::Mat& _input_dst_img, const bool& _is_find_target);
  ```

  设计思路：

  设计该函数是将预测量和原始目标位置进行合成，最终套用极坐标模型来计算预测目标的位置，返回给云台角度计算函数进行下一步的处理。每一个函数都是根据每一步的处理步骤进行设计，对整体运算内容进行解耦，方便调试修改。
    
### 计算云台角度

  ```cpp
  cv::Point2f angleCalculation(const cv::Point2f& _target_center, const float& _unit_pixel_length, const cv::Size& _image_size, const float& _focal_length);
  ```

  设计思路：
  设计之初是考虑使用 PnP 算法的方案来进行云台角度解算，但由于出现了位置的 BUG ，所以改用自行设计的相机模型来对打击目标进行解算，保证上场发挥的稳定性。
  
### 更新上一帧数据

  ```cpp
  void updateLastData(const bool& _is_find_target);
  ```

  设计思路：

  设计该函数是为了解决旧版本中数据更新位置的不统一，导致后续数据流排查时所需的时间剧增，进而设计了一个统一进行数据更新的函数，来方便进行管理。
  