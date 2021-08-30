# Angle_Solve 使用手册


## 一、使用说明

### 步骤一：头文件说明

- 包含头文件`basic_pnp.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_pnp.hpp`     | 角度结算抽象类           |
### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit PnP(std::string _camera_path, std::string _pnp_config_path);`

参数解释：
- `_camera_path` 为 `相机标定文件` 的路径地址
- `_pnp_config_path` 为 `basic_pnp_config.xml` 的路径地址
  
### 步骤三：函数说明

在模式选择后调用接口函数即可,接口函数重载了12种调用方式：

  ```cpp
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Mat& _src_img, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Mat& _src_img, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height,  cv::Mat& _src_img, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, cv::Mat& _src_img, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::RotatedRect _rect);            
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, 
  cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type,
  std::vector<cv::Point2f> _target_2d);
  void PnP::solvePnP(int  _ballet_speed, int _armor_type, cv::Mat&  _src_img, std::vector<cv::Point2f> _target_2d);
  void PnP::solvePnP(int _ballet_speed, int _armor_type,cv::RotatedRect _rect, int _depth)
  void PnP::solvePnP(int _ballet_speed, int _armor_type, std::vector<cv::Point2f> _target_2d, int _depth)


  ```
  参数解释：
  |      参数名         |           参数解释             |
  | ------------------ | -----------------------------|
  | `_ballet_speed`    | 子弹发射速度 |
  | `_armor_type`      | 装甲板类型 0 小装甲 1 大装甲 2 能量机关| 
  | `_src_img`         | 相机输入的图像，如：`src_img`，可删除不进行绘制 |
  | `_rect`            | 计算目标旋转矩形或者外接矩形 |
  | `_width`           | 目标实际宽度，与高度一起可替换`_armor_type` |
  | `_height`          | 目标实际高度，与宽度一起可替换`_armor_type` | 
  | `_target_2d`       | 目标2d点可替换 `_rect` |
  | `_depth`           | 自定义深度，例如：在环形高度，击打哨兵时，深度可以直接进行计算。 |

## 二、自定义函数及参数说明

### 初始化目标 3d 点

  ```cpp

  std::vector<cv::Point3f> initialize3DPoints(int _armor_type);
  std::vector<cv::Point3f> initialize3DPoints(int _width, int _heigth);
  ```
  设计思路:  
  将装甲板模型放在类的参数里面，通过选择可以得到不同的装甲板模型（ 0 小装甲板 1 大装甲板 2 能量机关）也提供了接口可以进行自定义目标实际高度和宽度。
### 初始化目标 2d 点

  ```cpp
  std::vector<cv::Point2f> initialize2DPoints(cv::RotatedRect _rect);
  std::vector<cv::Point2f> initialize2DPoints(cv::Rect _rect);
  ```
  设计思路:  
  在OpenCV 提供的`pnp`姿态结算算法中，`SOLVEPNP_ITERATIVE`方法只能用4个共面的特征点来解位姿,使用5个特征点或4点非共面的特征点都得不到正确的位姿。这里提供了`cv::RotatedRect`和`cv::Rect`类型的转换。保证只有四个点进行姿态解算。并且顺序为 左上->右上->右下->左下。
### 外接矩形转旋转矩形

  ```cpp
  cv::RotatedRect rectChangeRotatedrect(cv::Rect _rect)
  ```

### 转换坐标系

  ```cpp
  cv::Mat cameraPtz(cv::Mat& _t);
  ```
  设计思路:  
  相机计算出来的角度偏移量是相对于相机的坐标系，我们需要将坐标系转换为云台坐标系(即云台 Yaw 轴和 Pitch 轴的交点)。请修改`basic_pnp_config.xml`文件中的这个位置。
  ```xml
  <!--
  PTZ_CAMERA_X - 相机与云台的 X 轴偏移(左负右正)
  PTZ_CAMERA_Y - 相机与云台的 Y 轴偏移(上负下正)
  PTZ_CAMERA_Z - 相机与云台的 Z 轴偏移(前正后负)
  - 单位 mm
  - 云台与相机 相机作为参考点
  -->
  <PTZ_CAMERA_X>0.0</PTZ_CAMERA_X>
  <PTZ_CAMERA_Y>46</PTZ_CAMERA_Y>
  <PTZ_CAMERA_Z>-93.6</PTZ_CAMERA_Z>
  <!-- 
    PTZ_BARREL_X - 云台与枪管的 X 轴偏移(左负右正)
    PTZ_BARREL_Y - 云台与枪管的 Y 轴偏移(上负下正)
    PTZ_BARREL_Z - 云台与枪管的 Z 轴偏移(前正后负)
    - 单位 mm
    - 云台与枪管 云台作为参考点
  -->
  <PTZ_BARREL_X>0.0</PTZ_BARREL_X>
  <PTZ_BARREL_Y>0.0</PTZ_BARREL_Y>
  <PTZ_BARREL_Z>0.0</PTZ_BARREL_Z>
  ```
### 绘制坐标系

  ```cpp
  void drawCoordinate(cv::Mat& _draw_img, cv::Mat& _rvec, cv::Mat& _tvec,cv::Mat& _cameraMatrix, cv::Mat& _distcoeffs);
  ```
  设计思路:  
  绘制`pnp`姿态结算中的，姿态的结果并以三种颜色的线绘制出来。在云台抖动的时候，可以迅速的找出问题。
### 计算子弹下坠

  ```cpp
  float getPitch(float _dist, float _tvec_y, float _ballet_speed, const int _company = 1);
  ```
  设计思路:  
  子弹并不是直线射出，会收到重力，空气阻力等因素。对其造成影响。该算法修正了重力对子弹的影响。提高了子弹的命中率。
### 计算云台偏移角度

  ```cpp
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz, const int _bullet_speed, const int _company);
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz, const int _bullet_speed, const int _company, const int _depth);
  ```
  设计思路:  
  对OpenCV 提供的`pnp`姿态结算算法。得到的数据进行解析。得到云台 Yaw 和 Pitch 轴的偏移角度。例如:
  ```cpp
  // Yaw
  angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
  angle.x  = static_cast<float>(angle.x) * 180 / CV_PI;
  // Pitch
  angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
  angle.y  = static_cast<float>(angle.y) * 180 / CV_PI;
  // Depth
  angle.z  = static_cast<float>(xyz[2]);
  ```
