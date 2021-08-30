# Uart_Serial 使用手册


## 一、使用说明

### 步骤一：头文件说明

- 包含头文件 `uart_serial.hpp`

### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit SerialPort(std::string _serial_config);`

参数解释:
- `_serial_config` 为 `uart_serial_config.xml` 的路径地址
  
### 步骤三：函数说明

在模式选择后调用接口函数即可:

  ```cpp
  void writeData(const int& _yaw, const int16_t& yaw, const int& _pitch, const int16_t& pitch, const int16_t& depth, const int&  data_type = 0, const int& is_shooting = 0);
  void writeData(const Write_Data& _write_data);
  void updataWriteData(const float _yaw, const float _pitch, const int _depth, const int _data_type = 0, const int _is_shooting = 0);
  ```
  参数解释：
  |      参数名         |           参数解释             |
  | ------------------ | ------------------------------|
  | `_yaw`             | yaw 符号                      |
  | `yaw`              | yaw 绝对值                     | 
  | `_pitch`           | pitch 符号                     |
  | `pitch`            | pitch 绝对值                   |
  | `depth`            | 深度                           |
  | `data_type`        | 是否发现目标                    | 
  | `is_shooting`      | 开火命令                       |
  | `_write_data`      | 串口消息发送结构                |

## 二、自定义函数说明

### 发送数据

  ```cpp
  void writeData(const int& _yaw, const int16_t& yaw, const int& _pitch, const int16_t& pitch, const int16_t& depth, const int& data_type = 0, const int& is_shooting = 0);
  void writeData(const Write_Data& _write_data);
  void updataWriteData(const float _yaw, const float _pitch, const int _depth, const int _data_type = 0, const int _is_shooting = 0);
  ```
  设计思路:  
  为了提高数据的传输效率，浮点数类型数据需要传输时，截取两位小数点将浮点数转化为整数类型进行传输，可以缩短数据长度，并且避免浮点数传输时发生异常。例如: `write_data_.yaw_angle = fabs(_write_data.yaw_angle) * 100;` 下位机收到的数据除以 100 即可。

### 接收数据

  ```cpp
  void receiveData();
  bool isEmpty();
  void updateReceiveInformation();
  ```
  设计思路:  
  为了减少数据接受时的错误，在接受数据后对收到的数据进行判断。在确保数据为正确的前提下，对数据进行处理。例如：`receive_data_.bullet_velocity = receive_buff_[14] - 2;` 意思为收到的子弹速度 - 2。方便后面进行进一步处理。
### 返回接收到的数据

  ```cpp
  inline Receive_Data returnReceive();
  inline float returnReceiveAcceleration();
  inline int   returnReceiveBulletVelocity();
  inline int   returnReceiveRobotId();
  inline int   returnReceiceColor();
  inline int   returnReceiveMode();
  inline float returnReceivePitch();
  inline float returnReceiveYaw();
  ```
