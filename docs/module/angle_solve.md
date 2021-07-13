# Angle_Solve

## abstract_pnp.hpp

### 函数介绍

#### 1、初始化3d点 
* initialize3DPoints(int _armor_type)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| int  | _armor_type | 预设模型小(0) 大(1) buff装甲板(2)  |

* initialize3DPoints(int _width, int _heigth)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| int  | _width | 自定义宽度(mm) |
| int  | _heigth | 自定义高度(mm)  |
#### 2、初始化2d点
* initialize2DPoints(cv::RotatedRect _rect)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| RotatedRect  | _rect |   |

- initialize2DPoints(cv::Rect _rect)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| Rect  | _rect |   |

#### 3、纠正相机枪管偏移量
* cameraPtz(cv::Mat& _t)  
cv::Mat& _t pnp计算的平移向量

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| Mat  | _t | pnp输出的平移向量  |

#### 4、绘制坐标系
* drawCoordinate(cv::Mat& _draw_img,  
                 cv::Mat& _rvec,           cv::Mat& _tvec,  
                 cv::Mat& _cameraMatrix, cv::Mat& _distcoeffs) 

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| Mat  | _draw_img | 绘图图像  |
| Mat  | _rvec | pnp输出的旋转矩阵  |
| Mat  | _tvec | pnp输出的平移向量  |
| Mat  | _cameraMatrix | 相机内参  |
| Mat  | _distcoeffs | 相机外参  |

#### 5、获取偏移角度
* cv::Point3f getAngle(const cv::Mat& _pos_in_ptz,
                       const int      _bullet_speed,
                       const int      _company)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| Mat  | _pos_in_ptz | 偏移向量  |
| int  | _bullet_speed | 子弹速度m/s  |
| float  | _company | 补偿单位  |
#### 6、重力补偿
* float getPitch(float       _dist,
                 float       _tvec_y,
                 float       _ballet_speed,
                 const int   _company = 1)  

|  数据类型   | 命名  | 意义  |
|  ----  | ----  | ----  |
| float  | _dist | 距离  |
| float  | _tvec_y | 垂直高度  |
| int  | _ballet_speed | 子弹速度m/s  |
| float  | _company | 补偿单位  |


## basic_pnp.hpp

### 参数介绍

| 参数  |  数据类型   | 命名  | 意义  |
|  ----  |  ----  | ----  | ----  |
| 1  | int  | _ballet_speed | 子弹速度  |
| 2  | int  | _armor_type | 装甲板模型选择  |
| 2  | int  | _width | 自定义装甲板宽度(mm)  |
| 3 | int  | _height | 自定义装甲板高度(mm)  |
| 3/4 | Mat  | _src_img | 绘画板  |
| 3/4/5 | RotatedRect  | _rect | 目标旋转矩形  |
| 3/4/5 | Rect  | _rect | 目标外接矩形  |
| 3/4/5 | vector<Point2f>  | _target_2d | 自定义2d点  |
| lost | int  | _depth | 自定义深度  |

### 函数介绍

将至少三个参数传入void solvePnP() 后
再通过
* returnYawAngle()
* returnPitchAngle()
* returnDepth()
* returnTvecTx()
* returnTvecTy()
* returnTvecTz()
得到目标的Yaw Pitch 偏移量和Depth 