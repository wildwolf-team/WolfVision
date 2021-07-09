#include "uart_serial.hpp"

#include <string>
namespace uart {
const unsigned char CRC8Tab[300] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
    65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
    130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
    222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
    29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
    102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
    165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
    249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
    58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
    15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
    204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
    144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
    83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
    40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
    235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
    183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
    116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
    53};

/**
 * @brief Construct a new Serial Port:: Serial Port object
 * ------------------------------------------------------
 * @param:  波特率,默认为115200
 * --------------------------------------------------------
 * @param:  char parity
 *是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 * -------------------------------------------------------------
 * @param:  int databits 数据位的个数,默认值为8个数据位
 *----------------------------------------------------------
 * @return: bool  初始化是否成功
 * @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 * @author: Hzkkk
 *          Rcxxx (revised)
 */
SerialPort::SerialPort(std::string _serial_config) {
  // 更新串口部分的控制开关
  cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);

  fs_serial["SET_BANDRATE"] >> serial_config_.set_bandrate;
  fs_serial["SHOW_SERIAL_INFORMATION"] >>
      serial_config_.show_serial_information;

  std::cout << "The Serial set ......" << std::endl;
  const char* DeviceName[4] = {"", "/dev/ttyUSB0", "/dev/ttyUSB1",
                               "/dev/ttyUSB2"};

  /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
   * Terminals'll default to be set as Control Terminals
   */
  struct termios newstate;
  /*打开串口*/
  bzero(&newstate, sizeof(newstate));  // 清零
  for (size_t i = 0; i < (sizeof(DeviceName) / sizeof(char*)); ++i) {
    fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      printf("Can't Open Serial Port %s\n", DeviceName[i]);
      continue;
    } else {
      printf("Open Serial Port %s Successful\n", DeviceName[i]);
      break;
    }
  }

  /*设置发送波特率*/
  switch (serial_config_.set_bandrate) {
    case 1:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    case 10:
      cfsetospeed(&newstate, B921600);
      cfsetispeed(&newstate, B921600);
      break;
    default:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
  }

  // 本地连线, 取消控制功能 | 开始接收
  newstate.c_cflag |= CLOCAL | CREAD;
  // 设置字符大小
  newstate.c_cflag &= ~CSIZE;
  // 设置停止位1
  newstate.c_cflag &= ~CSTOPB;
  // 设置数据位8位
  newstate.c_cflag |= CS8;
  // 设置无奇偶校验位，N
  newstate.c_cflag &= ~PARENB;

  /*阻塞模式的设置*/
  newstate.c_cc[VTIME] = 0;
  newstate.c_cc[VMIN] = 0;

  /*清空当前串口*/
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newstate);
}

/**
 * @brief Destroy the Serial Port:: Serial Port object
 */
SerialPort::~SerialPort(void) {
  if (!close(fd)) printf("Close Serial Port Successful\n");
}

/**
 *  @brief: 串口数据读取函数
 *  @return: string  返回收到的字符串
 *  @note:   逐字节读取并存到字符串
 *           等待0.01s后结束读取,将所得字符串返回
 *  @authors: Rcxxx
 *            Hzkkk
 *            Wcjjj
 */
void SerialPort::rmReceiveData() {
  memset(receive_buff_, '0', REC_INFO_LENGTH * 2);  // 清空缓存
  read_message_ = read(fd, receive_buff_temp_,
                       sizeof(receive_buff_temp_));  // 读取串口中的数据
  // 对读取到的数据进行遍历排查，直到截取 'S' 开头和 'E'结尾的数据段后保存并退出
  for (int i = 0; i < static_cast<int>(sizeof(receive_buff_temp_)); ++i) {
    if (receive_buff_temp_[i] == 'S' &&
        receive_buff_temp_[i + sizeof(receive_buff_) - 1] == 'E') {
      for (int j = 0; j < (static_cast<int>(sizeof(receive_buff_))); ++j) {
        // 如果这里没进入可以检查双方协议的位数是否正确
        receive_buff_[j] = receive_buff_temp_[i + j];
        if (serial_config_.show_serial_information == 1) {
          std::cout << "recive_buff_:";
          std::cout << static_cast<int>(receive_buff_[i]) << " ";
        }
      }
      if (serial_config_.show_serial_information == 1) {
        std::cout << std::endl;
        break;
      }
    }
  }

  /* 接收协议：
      0:      S
      1:      color
      2:      model
      3:      robot_id
      4~7:    yaw_angle(union)
      8~11:   pitch_angle(union)
      12~13:  acceleration 12:High 13:low
      14:     bullet_velocity
      15:     E
   */
  tcflush(fd, TCIFLUSH);
}

/**
 *@brief: RM串口发送格式化函数
 *
 * @param: yaw 云台偏航
 * @param: pitch 云台俯仰
 * @param: _yaw yaw正负
 * @param: _pitch pitch正负
 * @param: data_type 是否正确识别的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 *           Wcjjj
 */
void SerialPort::rmSerialWrite(const int& _yaw, const int16_t& yaw,
                               const int& _pitch, const int16_t& pitch,
                               const int16_t& depth, const int& data_type,
                               const int& is_shooting) {
  getDataForCrc(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth);

  uint8_t CRC = checksumCrc(crc_buff_, sizeof(crc_buff_));
  getDataForSend(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, CRC);
  /*
  0：帧头     1：是否正确识别的标志   2：是否射击的信号
  3：yaw正负值    4：yaw低八位数据    5：yaw高八位数据
  6：pitch正负值  7：pitch低八位数据  8：pitch高八位数据
  9：深度低八位   10：深度高八位
  11：CRC
  12：帧尾
  */
  write_message_ = write(fd, write_buff_, sizeof(write_buff_));

  if (serial_config_.show_serial_information == 1) {
    yaw_reduction_ = mergeIntoBytes(write_buff_[5], write_buff_[4]);

    pitch_reduction_ = mergeIntoBytes(write_buff_[8], write_buff_[7]);

    depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);

    std::cout << "write_buff_=  " << write_buff_[0] << "  "
              << static_cast<int>(write_buff_[1]) << "  "
              << static_cast<int>(write_buff_[2]) << "  "
              << static_cast<int>(write_buff_[3]) << "  "
              << float(yaw_reduction_) / 100 << "  "
              << static_cast<int>(write_buff_[6]) << "  "
              << float(pitch_reduction_) / 100 << "  "
              << float(depth_reduction_) << "  "
              << static_cast<int>(write_buff_[11]) << "  " << write_buff_[12]
              << std::endl;

    yaw_reduction_ = 0x0000;
    pitch_reduction_ = 0x0000;
    depth_reduction_ = 0x0000;
  }
}
void SerialPort::updataWriteData(const float _yaw, const float _pitch,
                                 const int _depth, int _data_type,
                                 const int _is_shooting) {
  write_data_.symbol_yaw = 0;
  if (_yaw >= 0) {
    write_data_.symbol_yaw = 1;
  }
  write_data_.symbol_pitch = 0;
  if (_pitch >= 0) {
    write_data_.symbol_pitch = 1;
  }
  if (_data_type > 1) {
    _data_type = 1;
  }
  write_data_.yaw_angle = fabs(_yaw) * 100;
  write_data_.pitch_angle = fabs(_pitch) * 100;
  write_data_.depth = _depth;
  write_data_.data_type = _data_type;
  write_data_.is_shooting = _is_shooting;
  rmSerialWrite();
}

Write_Data SerialPort::gainWriteData(const float _yaw, const float _pitch,
                                     const int _depth, int _data_type,
                                     const int _is_shooting) {
  Write_Data write_data;
  write_data.symbol_yaw = 0;
  if (_yaw >= 0) {
    write_data.symbol_yaw = 1;
  }
  write_data.symbol_pitch = 0;
  if (_pitch >= 0) {
    write_data.symbol_pitch = 1;
  }
  if (_data_type > 1) {
    _data_type = 1;
  }
  write_data.yaw_angle = fabs(_yaw) * 100;
  write_data.pitch_angle = fabs(_pitch) * 100;
  write_data.depth = _depth;
  write_data.data_type = _data_type;
  write_data.is_shooting = _is_shooting;
  return write_data;
}
void SerialPort::rmSerialWrite(const Write_Data _write_data) {
  getDataForCrc(_write_data.data_type, _write_data.is_shooting,
                _write_data.symbol_yaw, _write_data.yaw_angle,
                _write_data.symbol_pitch, _write_data.pitch_angle,
                _write_data.depth);
  uint8_t CRC = checksumCrc(crc_buff_, sizeof(crc_buff_));
  getDataForSend(_write_data.data_type, _write_data.is_shooting,
                 _write_data.symbol_yaw, _write_data.yaw_angle,
                 _write_data.symbol_pitch, _write_data.pitch_angle,
                 _write_data.depth, CRC);
  write_message_ = write(fd, write_buff_, sizeof(write_buff_));
  if (serial_config_.show_serial_information == 1) {
    yaw_reduction_ = mergeIntoBytes(write_buff_[5], write_buff_[4]);

    pitch_reduction_ = mergeIntoBytes(write_buff_[8], write_buff_[7]);

    depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);

    std::cout << "write_buff_=  " << write_buff_[0] << "  "
              << static_cast<int>(write_buff_[1]) << "  "
              << static_ca  // namespace uart_reduction_) / 100 << "  "
              << static_cast<int>(write_buff_[6]) << "  "
              << float(pitch_reduction_) / 100 << "  "
              << float(depth_reduction_) << "  "
              << static_cast<int>(write_buff_[11]) << "  " << write_buff_[12]
              << std::endl;

    yaw_reduction_ = 0x0000;
    pitch_reduction_ = 0x0000;
    depth_reduction_ = 0x0000;
  }
}
void SerialPort::rmSerialWrite() {
  getDataForCrc(write_data_.data_type, write_data_.is_shooting,
                write_data_.symbol_yaw, write_data_.yaw_angle,
                write_data_.symbol_pitch, write_data_.pitch_angle,
                write_data_.depth);
  uint8_t CRC = checksumCrc(crc_buff_, sizeof(crc_buff_));
  getDataForSend(write_data_.data_type, write_data_.is_shooting,
                 write_data_.symbol_yaw, write_data_.yaw_angle,
                 write_data_.symbol_pitch, write_data_.pitch_angle,
                 write_data_.depth, CRC);
  write_message_ = write(fd, write_buff_, sizeof(write_buff_));
  if (serial_config_.show_serial_information == 1) {
    yaw_reduction_ = mergeIntoBytes(write_buff_[5], write_buff_[4]);

    pitch_reduction_ = mergeIntoBytes(write_buff_[8], write_buff_[7]);

    depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);

    std::cout << "write_buff_=  " << write_buff_[0] << "  "
              << static_cast<int>(write_buff_[1]) << "  "
              << static_cast<int>(write_buff_[2]) << "  "
              << static_cast<int>(write_buff_[3]) << "  "
              << float(yaw_reduction_) / 100 << "  "
              << static_cast<int>(write_buff_[6]) << "  "
              << float(pitch_reduction_) / 100 << "  "
              << float(depth_reduction_) << "  "
              << static_cast<int>(write_buff_[11]) << "  " << write_buff_[12]
              << std::endl;

    yaw_reduction_ = 0x0000;
    pitch_reduction_ = 0x0000;
    depth_reduction_ = 0x0000;
  }
}

/** CRC8校验函数
 *
 *  @param:  char *buf   需要检验的字符串
 *  @param:  uint16_t len
 *是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 *
 *  @return: bool  初始化是否成功
 *  @brief:  CRC8校验 ---MAXIM x8+x5+x4+x1  多项式 POLY（Hex）:31(110001) 初始值
 *INIT（Hex）：00 结果异或值 XOROUT（Hex）：
 *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 */
uint8_t SerialPort::checksumCrc(unsigned char* buf, uint16_t len) {
  uint8_t check = 0;

  while (len--) {
    check = CRC8Tab[check ^ (*buf++)];
  }

  return (check);
}

/**
 * @brief 提取数据进入 CRC 校验数组
 *
 * @param data_type
 * @param is_shooting
 * @param _yaw
 * @param yaw
 * @param _pitch
 * @param pitch
 * @param depth
 */
void SerialPort::getDataForCrc(const int& data_type, const int& is_shooting,
                               const int& _yaw, const int16_t& yaw,
                               const int& _pitch, const int16_t& pitch,
                               const int16_t& depth) {
  crc_buff_[0] = 0x53;
  crc_buff_[1] = static_cast<unsigned char>(data_type);
  crc_buff_[2] = static_cast<unsigned char>(is_shooting);
  crc_buff_[3] = static_cast<unsigned char>(_yaw);
  crc_buff_[4] = returnLowBit(yaw);
  crc_buff_[5] = returnHighBit(yaw);
  crc_buff_[6] = static_cast<unsigned char>(_pitch);
  crc_buff_[7] = returnLowBit(pitch);
  crc_buff_[8] = returnHighBit(pitch);
  crc_buff_[9] = returnLowBit(depth);
  crc_buff_[10] = returnHighBit(depth);
}

/**
 * @brief 提取数据进入发送的数组
 *
 * @param data_type
 * @param is_shooting
 * @param _yaw
 * @param yaw
 * @param _pitch
 * @param pitch
 * @param depth
 * @param CRC
 */
void SerialPort::getDataForSend(const int& data_type, const int& is_shooting,
                                const int& _yaw, const int16_t& yaw,
                                const int& _pitch, const int16_t& pitch,
                                const int16_t& depth, const uint8_t& CRC) {
  write_buff_[0] = 0x53;
  write_buff_[1] = static_cast<unsigned char>(data_type);
  write_buff_[2] = static_cast<unsigned char>(is_shooting);
  write_buff_[3] = static_cast<unsigned char>(_yaw);
  write_buff_[4] = returnLowBit(yaw);
  write_buff_[5] = returnHighBit(yaw);
  write_buff_[6] = static_cast<unsigned char>(_pitch);
  write_buff_[7] = returnLowBit(pitch);
  write_buff_[8] = returnHighBit(pitch);
  write_buff_[9] = returnLowBit(depth);
  write_buff_[10] = returnHighBit(depth);
  write_buff_[11] = CRC & 0xff;
  write_buff_[12] = 0x45;
}
/**
 * @brief 判断接收数据是否为空
 * @return true     为空（0）
 * @return false    不为空
 */
bool SerialPort::isEmpty() {
  if (receive_buff_[0] != '0' ||
      receive_buff_[REC_INFO_LENGTH * 2 - 1] != '0') {
    return false;
  } else {
    return true;
  }
}

void SerialPort::updateReceiveInformation() {
  rmReceiveData();
  if (isEmpty()) {
    receive_data_ = last_receive_data_;
  }
  last_receive_data_ = receive_data_;
  // 转换类型为 int
  for (size_t i = 0; i < sizeof(transform_arr_) / sizeof(transform_arr_[0]);
       ++i) {
    this->transform_arr_[i] = this->receive_buff_[i + 1] -
                              '0';  // 需要 - '0',因为char转int会剩下ascII码的值
  }
  /* 1 更新颜色信息　update color */
  switch (transform_arr_[0]) {
    case RED:
      this->receive_data_.my_color = RED;
      break;
    case BLUE:
      this->receive_data_.my_color = BLUE;
      break;
    default:
      this->receive_data_.my_color = RED;
      break;
  }
  /* 2 更新模式信息 update mode */
  switch (transform_arr_[1]) {
    case SUP_SHOOT:
      this->receive_data_.now_run_mode = SUP_SHOOT;
      break;
    case ENERGY_AGENCY:
      this->receive_data_.now_run_mode = ENERGY_AGENCY;
      break;
    case SENTRY_MODE:
      this->receive_data_.now_run_mode = SENTRY_MODE;
      break;
    case BASE_MODE:
      this->receive_data_.now_run_mode = BASE_MODE;
      break;
    default:
      this->receive_data_.now_run_mode = SUP_SHOOT;
      break;
  }
  /* 3 更新当前机器人ID update Robot ID */
  switch (transform_arr_[2]) {
    case HERO:
      this->receive_data_.my_robot_id = HERO;
      break;
    case ENGINEERING:
      this->receive_data_.my_robot_id = ENGINEERING;
      break;
    case INFANTRY:
      this->receive_data_.my_robot_id = INFANTRY;
      break;
    case UAV:
      this->receive_data_.my_robot_id = UAV;
      break;
    case SENTRY:
      this->receive_data_.my_robot_id = SENTRY;
      break;
    default:
      this->receive_data_.my_robot_id = INFANTRY;
      break;
  }
  /* 14 更新子弹速度 */
  // this->transform_arr_[3] = this->receive_buff_[14] - '0';
  // switch (this->transform_arr_[3]) {
  //   case 1:
  //     this->receive_data_.bullet_velocity = 15;
  //     break;
  //   case 2:
  //     this->receive_data_.bullet_velocity = 18;
  //     break;
  //   case 3:
  //     this->receive_data_.bullet_velocity = 30;
  //     break;
  //   default:
  //     this->receive_data_.bullet_velocity = 30;
  //     break;
  // }
  this->receive_data_.bullet_velocity = this->receive_buff_[14];
  /* 4 5 6 7 更新陀螺仪的yaw角度值 */
  for (size_t i = 0;
       i < sizeof(this->receive_data_.Receive_Yaw_Angle_Info.arr_yaw_angle);
       ++i) {
    // +4是接收串口数组中陀螺仪yaw角度值的起始位置
    this->receive_data_.Receive_Yaw_Angle_Info.arr_yaw_angle[i] =
        this->receive_buff_[i + 4];
  }

  /*8 9 10 11 更新陀螺仪的pitch角度值 */
  for (size_t i = 0;
       i < sizeof(this->receive_data_.Receive_Pitch_Angle_Info.arr_pitch_angle);
       ++i) {
    // +8是接收串口数组中陀螺仪pitch角度值的起始位置
    this->receive_data_.Receive_Pitch_Angle_Info.arr_pitch_angle[i] =
        this->receive_buff_[i + 8];
  }

  /* 12 13 更新陀螺仪的角加速度值 */
  this->receive_data_.acceleration =
      SerialPort::mergeIntoBytes(this->receive_buff_[12],
                                 this->receive_buff_[13]) /
      100.f;
  if (serial_config_.show_serial_information == 1) {
    displayReceiveInformation();
  }
}
/**
 * @brief 打印当前接收的信息
 */
void SerialPort::displayReceiveInformation() {
  std::cout << "color:" << this->receive_data_.my_color
            << " model:" << this->receive_data_.now_run_mode
            << " ID:" << this->receive_data_.my_robot_id
            << " yaw:" << this->receive_data_.Receive_Yaw_Angle_Info.yaw_angle
            << " pitch:"
            << this->receive_data_.Receive_Pitch_Angle_Info.pitch_angle
            << " acceleration:" << this->receive_data_.acceleration
            << " volacity:" << this->receive_data_.bullet_velocity << std::endl;
}
}  // namespace uart
