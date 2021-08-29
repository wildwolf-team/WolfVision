# Uart_Serial 使用手册


## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`uart_serial.hpp`

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

## 二、自定义函数及参数说明

---

### 发送数据

```cpp
  /**
   * @brief 发送数据
   * @param  _yaw             yaw 符号
   * @param  yaw              yaw 绝对值
   * @param  _pitch           pitch 符号
   * @param  pitch            pitch 绝对值
   * @param  depth            深度
   * @param  data_type        是否发现目标
   * @param  is_shooting      开火命令
   */
  void writeData(const int&     _yaw,   const int16_t& yaw,
                 const int&     _pitch, const int16_t& pitch,
                 const int16_t& depth,  const int&     data_type = 0,
                 const int&     is_shooting = 0);
  void writeData();
  /**
   * @brief 发送数据
   *
   * @param _write_data     需要发送的 Write_Data 结构体
   */
  void writeData(const Write_Data& _write_data);
  /**
   * @brief 发送数据
   *
   * @param _yaw          yaw 数据
   * @param _pitch        pitch 数据
   * @param _depth        深度
   * @param _data_type    是否发现目标
   * @param _is_shooting  开火命令
   */
  void updataWriteData(const float _yaw,   const float _pitch,
                       const int   _depth, const int   _data_type = 0,
                       const int   _is_shooting = 0);
```

### 接收数据

  ```cpp
  /**
   * @brief 接收数据
   */
  void receiveData();
  /**
   * @brief 接收数据是否正常
   * @return true  不正常
   * @return false 正常
   */
  bool isEmpty();
  /**
   * @brief 更新数据信息
   */
  void updateReceiveInformation();
  ```

### 返回接收到的数据

  ```cpp
  /**
   * @brief 返回接受数据的结构体
   * 
   * @return Receive_Data 
   */
  inline Receive_Data returnReceive();
  /**
   * @brief 返回陀螺仪 Yaw 速度
   * 
   * @return float 
   */
  inline float returnReceiveAcceleration();
  /**
   * @brief 返回子弹速度
   * 
   * @return int 
   */
  inline int   returnReceiveBulletVelocity();
  /**
   * @brief 返回机器人 ID
   * 
   * @return int 
   */
  inline int   returnReceiveRobotId();
  /**
   * @brief 返回自身颜色
   * 
   * @return int 
   */
  inline int   returnReceiceColor();
  /**
   * @brief 返回模式选择
   * 
   * @return int 
   */
  inline int   returnReceiveMode();
  /**
   * @brief 返回陀螺仪 Pitch 轴数据
   * 
   * @return float 
   */
  inline float returnReceivePitch();
  /**
   * @brief 返回陀螺仪 Yaw 轴数据
   * 
   * @return float 
   */
  inline float receiveYaw();

  ```