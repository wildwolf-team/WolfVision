# mv_video_capture 使用手册


## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`mv_video_capture.hpp`

### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit VideoCapture(const mindvision::CameraParam &_camera_param);`

参数解释：
- `_camera_param` 为 `CameraParam` 的结构体
  
### 步骤三：函数说明

在初始化后调用接口函数即可

  ```cpp
  inline cv::Mat image();
  ```

## 二、自定义函数及参数说明

---

### 判断工业相机是否在线

  ```cpp
  /**
   * @brief 判断工业相机是否在线
   *
   * @return true   检测到工业相机
   * @return false  没检测到工业相机
   */
  bool isindustryimgInput();
  ```

### 清空相机内存

  ```cpp
  /**
   * @brief 清空相机内存（每次读取相机后进行清空）
   * 
   */
  void cameraReleasebuff();
  ```

### 初始化相机参数

  ```cpp
  /**
   * @brief 设置相机参数
   * 
   * @param _CAMERA_RESOLUTION_COLS  设置相机宽度  
   * @param _CAMERA_RESOLUTION_ROWS  设置相机高度
   * @param _CAMERA_EXPOSURETIME     设置相机曝光
   * @return int 
   */
  int cameraInit(const int _CAMERA_RESOLUTION_COLS,
                 const int _CAMERA_RESOLUTION_ROWS,
                 const int _CAMERA_EXPOSURETIME);
  ```

### 返回相机读取图片

  ```cpp
  /**
   * @brief 返回相机读取图片
   * 
   * @return cv::Mat 
   */
  inline cv::Mat image();
  ```

