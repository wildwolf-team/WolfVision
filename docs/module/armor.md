# Basic_Armor 使用手册


## 一、使用说明

---

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

---

### 预处理执行函数

  ```cpp
  /**
   * @brief 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   */
  void runImage(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief BGR 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat bgrPretreat(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief HSV 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat hsvPretreat(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief GRAY 预处理
   * 
   * @param _src_img    原图 （ CV_8UC3 ）
   * @param _my_color   自身的颜色
   * @return cv::Mat    返回处理后的二值图
   */
  cv::Mat grayPretreat(const cv::Mat &_src_img, const int _my_color);
  /**
   * @brief 图片合并（保证图片类型一致）
   * 
   * @param _bin_gray_img   图片一
   * @param _bin_color_img  图片二
   * @return cv::Mat        返回合并后的图片
   */
  cv::Mat fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img);
  ```

### 寻找灯条

  ```cpp
  /**
   * @brief 寻找灯条
   * 
   * @return true   找到灯条
   * @return false  没有找到灯条
   */
  bool  findLight();
  ```

### 拟合装甲板

  ```cpp
  /**
   * @brief 灯条拟合装甲板
   * 
   * @param i       左灯条在 light_ 的位置
   * @param j       右灯条在 light_ 的位置
   * @return true   左右灯条能拟合成装甲板
   * @return false  左右灯条不能拟合成装甲板
   */
  bool  lightJudge(const int i, const int j);
  /**
   * @brief 装甲板匹配
   * 
   * @return true   匹配到装甲板
   * @return false  没有匹配到装甲板
   */
  bool  fittingArmor();
  /**
   * @brief 计算图像颜色平均强度
   *
   * @return int 返回图像颜色平均强度
   */
  int   averageColor();
  ```

### 最优装甲板排序

  ```cpp
  /**
   * @brief 最优装甲板排序
   * 
   */
  void  finalArmor();
  ```

### 哨兵模式

  ```cpp
  /**
   * @brief 击打哨兵模式
   *
   * @param _src_img       原图（ CV_8UC3 ）
   * @param _receive_data  串口接受的数据
   * @return true          识别到装甲板
   * @return false         没有识别到装甲板
   */
  bool sentryMode(const cv::Mat& _src_img, const uart::Receive_Data _receive_data);
  /**
   * @brief 更新击打哨兵模式参数
   * 
   * @param _gyro_speed       Yaw 轴陀螺仪速度
   * @param _bullet_velocity  子弹速度
   * @param _yaw_angle        Yaw 轴陀螺仪值
   */
  void initialPredictionData(const float _gyro_speed, const int _bullet_velocity, const float _yaw_angle);
  /**
   * @brief 初始化哨兵模式参数 (请在使用完哨兵模式的时候对其清空)
   * 
   */
  inline void initializationSentryMode();
  ```

### 返回数据

  ```cpp
  /**
   * @brief 返回丢失次数
   * 
   * @return int  
   */
  inline int             returnLostCnt();
  /**
   * @brief 返回装甲板数量
   * 
   * @return int 
   */
  inline int             returnArmorNum();
  /**
   * @brief 返回是否找到装甲板
   * 
   * @return true   找到装甲板
   * @return false  没找到装甲板
   */
  inline bool            returnSuccessArmor();
  /**
   * @brief 返回最优装甲板的结构体
   * 
   * @param _num         第0个为最优装甲板
   * @return Armor_Data  返回第 _num 个装甲板的结构体
   */
  inline Armor_Data      returnFinalArmor(const int _num);
  /**
   * @brief 返回装甲板类型
   * 
   * @param _num  返回第 _num 个装甲板的类型
   * @return int  返回装甲板类型（ 0 小装甲板 1 大装甲板）
   */
  inline int             returnFinalArmorDistinguish(const int _num);
  /**
   * @brief 返回最优装甲板的旋转矩形
   *
   * @param _num             返回第 _num 个装甲板的旋转矩形
   * @return cv::RotatedRect 返回装甲板类型
   */
  inline cv::RotatedRect returnFinalArmorRotatedRect(const int _num);
  ```

