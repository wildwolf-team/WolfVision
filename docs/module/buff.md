# RM_Buff 使用手册

---

## 一、TODO LIST

---

- [x]  文件夹架构
  - [x]  xml
  - [x]  object
  - [x]  fanarmor
  - [x]  fanblade
  - [x]  target
  - [x]  circle
- [x]  接口函数
- [ ]  Test 文件
- [ ]  Release
- [ ]  Predict
- [ ]  DEBUG (I'm a typical BUG manufacturer /(ㄒoㄒ)/~~)
- [ ]  Auto Control

## 二、使用说明

---

### 步骤一：包含头文件

- 包含头文件`RM_Buff.hpp`

| 文件名                 | 修改方式                                    |
| ---------------------- | ------------------------------------------- |
| `Release_Controller.h` | 可去除,后期再作合并添加                     |
| `rm_solve_pnp.hpp`     | 保留                                        |
| `Center_R.hpp`         | 保留                                        |
| `Target.hpp`           | 保留                                        |
| `RM_FPS.h`             | 更改相应头文件，现已包含进`RM_Buff`文件夹中 |
| `RM_Messenger.h`       | 更改相应头文件（修改为serial）              |
| `rm_roi.h`             | 保留                                        |

### 步骤二：实例化对象

实例化对象时调用构造函数：`RM_Buff(const std::string& _buff_config_address);`

参数解释：`_buff_config_address` 为 `buff_config.xml` 的路径地址
  
### 步骤三：调用函数

在模式选择后调用接口函数即可，接口函数重载了3种调用方式：

   ```C++
  void runTask(Mat& _input_img, Receive_Info& _receive_info, Send_Info& _send_info);
  void runTask(Mat& _input_img, RM_Messenger* _messenger);
  Send_Info runTask(Mat& _input_img, Receive_Info& _receive_info);
  ```

  参数解释：
  | 参数名          | 参数解释                                                           |
  | --------------- | ------------------------------------------------------------------ |
  | `_input_img`    | 相机输入的图像，如：`src_img`                                      |
  | `_receive_info` | 串口接收的结构体，提供信息给`RM_Buff`类,如：`color`                |
  | `_send_info`    | 串口发送的结构体，接口函数会对发送值进行修改，最终存储到该结构体中 |
  | `_messenger`    | 信使类，串口接收和发送结构体的集成对象指针，可以调用内部函数       |

### 步骤四：修改 Buff 中的 pnp 对象构造函数所需地址

在文件 `RM_Buff.hpp` 中的对象 `buff_pnp_` ，将其所需的相机标定文件以及参数配置文件地址修改正确即可

## 三、自定义函数说明

---

### 获取基本信息

  ```C++
  void Input(Mat& _input_img, const int& _my_color);
  ```

### 显示最终图像

  ```C++
  void displayDst();
  ```

### 预处理执行函数

  ```C++
  void imageProcessing(Mat&                     _input_img,
                       Mat&                     _output_img,
                       const int&               _my_color,
                       const Processing_Moudle& _process_moudle);
  ```

### 查找目标

```C++
void findTarget(Mat& _input_dst_img, Mat& _input_bin_img, vector<Target>& _target_box);
```

### 判断是否有目标

```C++
bool isFindTarget(Mat& _input_img, vector<Target>& _target_box);
```

### 查找圆心

```C++
Point2f findCircleR(Mat& _input_src_img, Mat& _input_bin_img, Mat& _dst_img, const bool& _is_find_target);
```

### 计算运转状态值：速度、方向、角度

```C++
void judgeCondition(const bool& _is_find_target);
```

### 计算预测量

```C++
float Predict(const float& _bullet_velocity, const bool& _is_find_target);
```

### 计算最终目标矩形顶点点集

```C++
void calculateTargetPointSet(const float&     _predict_quantity,
                             const Point2f&   _final_center_r,
                             vector<Point2f>& _target_2d_point,
                             Mat&             _input_dst_img,
                             const bool&      _is_find_target);
```

### 计算云台角度

```C++
void run_Solvepnp(int                      _ballet_speed,
                  int                      _armor_type,
                  cv::Mat&                 _src_img,
                  std::vector<cv::Point2f> _target_2d,
                  int                      _depth);
```

### 自动控制

```C++
```

### 更新上一帧数据

```C++
void updateLastData(const bool& _is_find_target);
```
