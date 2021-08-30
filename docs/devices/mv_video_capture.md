# MV_Video_Capture 使用手册


## 一、使用说明

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

## 二、自定义函数说明

### 判断工业相机是否在线

  ```cpp
  bool isindustryimgInput();
  ```
  设计思路:  
  判断是否工业相机是否在线，并分配内存去储存工业相机在当前的图像。
### 清空相机内存

  ```cpp
  void cameraReleasebuff();
  ```
  设计思路:  
  读取当前图像后，需要对其内存进行清空，否则会造成无法储存下一次图像。
### 初始化相机参数

  ```cpp
  int cameraInit(const int _CAMERA_RESOLUTION_COLS,
                 const int _CAMERA_RESOLUTION_ROWS,
                 const int _CAMERA_EXPOSURETIME);
  ```
  设计思路:  
  设置当前相机的各种参数，确定读取后图像的大小和曝光时间，来保证在本次代码中读取到的图像效果基本一致。
### 返回相机读取图片

  ```cpp
  inline cv::Mat image();
  ```