# Basic_Armor 使用手册

## 一、使用说明

### 步骤一：头文件说明

- 包含头文件`basic_armor.hpp`

### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit Detector(const std::string _armor_config);`

参数解释:
- `_armor_config` 为 `basic_armor_config.xml` 的路径地址
  
### 步骤三：函数说明

在模式选择后调用接口函数即可:

  ```cpp
  bool runBasicArmor(const cv::Mat& _src_img, const uart::Receive_Data _receive_data);
  bool sentryMode(const cv::Mat& _src_img, const uart::Receive_Data _receive_data);
  ```
  参数解释：
  |      参数名         |           参数解释             |
  | ------------------ | -----------------------------|
  | `_src_img`         | 相机输入的图像，如：`src_img`   |
  | `_receive_data`    | 串口接受到的结构体              |


## 二、自定义函数及参数说明

### 预处理执行函数

  ```cpp
  void runImage(const cv::Mat &_src_img, const int _my_color);
  cv::Mat bgrPretreat(const cv::Mat &_src_img, const int _my_color);
  cv::Mat hsvPretreat(const cv::Mat &_src_img, const int _my_color);
  cv::Mat grayPretreat(const cv::Mat &_src_img, const int _my_color);
  cv::Mat fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img);
  ```
  预处理部分主要是将图像二值化，而二值化的关键就在于阈值，这里我们采用了通道相减。若自身为红色则原图像的红色通道减去蓝色通道，若自身为蓝色则原图像的蓝色通道减去红色通道，接着找出图像最大灰度，最终值：灰度二值与颜色二值取交集，就是二值化的阈值，经过测试，这个方法可以将绝大多数不发光的物体排除在外，在后面处理轮廓和轮廓筛选环节能节省较多时间。
### 寻找灯条

  ```cpp
  bool  findLight();
  ```
  通过轮廓周长、轮廓高宽比和旋转矩形角度，对其判断是否为灯条。排除了大部分不为灯条的轮廓。为后面装甲板拟合节省了较多时间。
### 拟合装甲板

  ```cpp
  bool  lightJudge(const int i, const int j);
  bool  fittingArmor();
  int   averageColor();
  ```
  灯条两两匹配为装甲板，通过倾斜弧度 左右灯条高宽比 灯条高度差 灯条角度差和装甲板长宽比进行判断是否为装甲板。再通过平均颜色强度，排除装甲板中心较亮的装甲板。选出真正的装甲板。
### 最优装甲板排序

  ```cpp
  void  finalArmor();
  ```
  通过装甲板到图像中心点的距离，进行排序由低到高。便于操作手进行选择装甲板击打。
### 哨兵模式

  ```cpp
  bool sentryMode(const cv::Mat& _src_img, const uart::Receive_Data _receive_data);
  void initialPredictionData(const float _gyro_speed, const int _bullet_velocity, const float _yaw_angle);
  inline void initializationSentryMode();
  ```
  这是一个自瞄的附属模式。需要在使用前，跑到环形高地上对准中间的银色矿石再开启该模式。因为哨兵的运动轨迹是完全已知的。
- 1、可以直接计算出哨兵距离枪管的实际距离。
- 2、通过每 10ms 的陀螺仪角度差计算出当前云台的转动角速度。
- 3、通过延时滤波和陀螺仪的速度判断预测方向。
- 4、通过实际距离算出子弹到装甲板的飞行时间。
- 5、用飞行时间乘以转动角速度可以得到我们要的预测角度。
- 6、通过小孔成像原理将角度转换成像素点个数。
- 7、判断预测方向将6计算出来的结果补到装甲板中心点上。
- 8、最后将补偿后的装甲板旋转矩形放进`pnp`进行计算。

### 返回数据

  ```cpp
  inline int             returnLostCnt();
  inline int             returnArmorNum();
  inline bool            returnSuccessArmor();
  inline Armor_Data      returnFinalArmor(const int _num);
  inline int             returnFinalArmorDistinguish(const int _num);
  inline cv::RotatedRect returnFinalArmorRotatedRect(const int _num);
  ```
  返回装甲板的各种数据，可以自己进行修改。