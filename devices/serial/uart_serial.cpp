#include "uart_serial.hpp"

namespace uart {

const unsigned char CRC8Tab[] = {
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


SerialPort::SerialPort(std::string _serial_config) {
  cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);

  fs_serial["SET_BAUDRATE"]            >> serial_config_.set_baudrate;
  fs_serial["SHOW_SERIAL_INFORMATION"] >> serial_config_.show_serial_information;

  const char* DeviceName[] = {"", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

  struct termios newstate;
  bzero(&newstate, sizeof(newstate));

  for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char*); ++i) {
    fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      fmt::print("[{}] Open serial device failed: {}\n", idntifier_red, DeviceName[i]);
    } else {
      fmt::print("[{}] Open serial device success: {}\n", idntifier_green, DeviceName[i]);

      break;
    }
  }

  switch (serial_config_.set_baudrate) {
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

  newstate.c_cflag |= CLOCAL | CREAD;
  newstate.c_cflag &= ~CSIZE;
  newstate.c_cflag &= ~CSTOPB;
  newstate.c_cflag |= CS8;
  newstate.c_cflag &= ~PARENB;

  newstate.c_cc[VTIME] = 0;
  newstate.c_cc[VMIN]  = 0;

  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newstate);
}

SerialPort::~SerialPort(void) {
  if (!close(fd)) { fmt::print("[{}] Close serial device success: {}\n", idntifier_green, fd); }
}

/* Receiving protocol:
 *  0:      'S'
 *  1:      color
 *  2:      model
 *  3:      robot_id
 *  4~7:    yaw_angle (union)
 *  8~11:   pitch_angle (union)
 *  12~13:  acceleration (12:high,13:low)
 *  14:     bullet_velocity
 *  15:     'E'
*/
void SerialPort::receiveData() {
  memset(receive_buff_, '0', REC_INFO_LENGTH * 2);
  read_message_ = read(fd, receive_buff_temp_, sizeof(receive_buff_temp_));

  for (size_t i = 0; i != sizeof(receive_buff_temp_); ++i) {
    if (receive_buff_temp_[i] == 'S' && receive_buff_temp_[i + sizeof(receive_buff_) - 1] == 'E') {
      if (serial_config_.show_serial_information == 1) {
        fmt::print("[{}] receiveData() ->", idntifier_green);

        for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
          receive_buff_[j] = receive_buff_temp_[i + j];

          fmt::print(" {}", static_cast<int>(receive_buff_[i]));
        }

        fmt::print("\n");
      } else {
        for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
          receive_buff_[j] = receive_buff_temp_[i + j];
        }
      }

      break;
    }
  }

  tcflush(fd, TCIFLUSH);
}

/* Sending protocol:
 *  0:      'S'
 *  1:      data_type
 *  2:      is_shooting
 *  3:      _yaw (sign)
 *  4~5:    yaw (4:low, 5:high)
 *  6:      _pitch (sign)
 *  7~8:    pitch (7:low, 8:high)
 *  9~10:   depth (9:low, 10:high)
 *  11:     CRC
 *  12:     'E'
*/
void SerialPort::writeData(const int&     _yaw,   const int16_t& yaw,
                           const int&     _pitch, const int16_t& pitch,
                           const int16_t& depth,  const int&     data_type,
                           const int&     is_shooting) {
  getDataForCRC(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth);

  uint8_t CRC = checksumCRC(crc_buff_, sizeof(crc_buff_));

  getDataForSend(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, CRC);

  write_message_ = write(fd, write_buff_, sizeof(write_buff_));

  if (serial_config_.show_serial_information == 1) {
    yaw_reduction_   = mergeIntoBytes(write_buff_[5],  write_buff_[4]);
    pitch_reduction_ = mergeIntoBytes(write_buff_[8],  write_buff_[7]);
    depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);

    fmt::print("[{}] writeData() ->", idntifier_green);
    for (size_t i = 0; i != 4; ++i) { fmt::print(" {}", write_buff_[i]);  }
    fmt::print(" {} {} {} {}",
      static_cast<float>(yaw_reduction_) / 100,
      static_cast<int>(write_buff_[6]),
      static_cast<float>(pitch_reduction_) / 100,
      static_cast<float>(depth_reduction_));
    for (size_t i = 11; i != 12; ++i) { fmt::print(" {}", write_buff_[i]); }
    fmt::print("\n");

    yaw_reduction_   = 0x0000;
    pitch_reduction_ = 0x0000;
    depth_reduction_ = 0x0000;
  }
}

void SerialPort::updataWriteData(const float _yaw,   const float _pitch,
                                 const int   _depth, const int   _data_type,
                                 const int   _is_shooting) {
  write_data_.data_type    = _data_type > 1 ? 1 : _data_type;
  write_data_.is_shooting  = _is_shooting;
  write_data_.symbol_yaw   = _yaw >= 0 ? 1 : 0;
  write_data_.yaw_angle    = fabs(_yaw) * 100;
  write_data_.symbol_pitch = _pitch >= 0 ? 1 : 0;
  write_data_.pitch_angle  = fabs(_pitch) * 100;
  write_data_.depth        = _depth;

  writeData();
}

Write_Data SerialPort::gainWriteData(const float _yaw,   const float _pitch,
                                     const int   _depth, const int   _data_type,
                                     const int   _is_shooting) {
  Write_Data write_data;

  write_data.data_type    = _data_type > 1 ? 1 : _data_type;
  write_data.is_shooting  = _is_shooting;
  write_data.symbol_yaw   = _yaw >= 0 ? 1 : 0;
  write_data.yaw_angle    = fabs(_yaw) * 100;
  write_data.symbol_pitch = _pitch >= 0 ? 1 : 0;
  write_data.pitch_angle  = fabs(_pitch) * 100;
  write_data.depth        = _depth;

  return write_data;
}

void SerialPort::writeData(const Write_Data _write_data) {
  writeData(_write_data.symbol_yaw,   _write_data.yaw_angle,
            _write_data.symbol_pitch, _write_data.pitch_angle,
            _write_data.depth,        _write_data.data_type,
            _write_data.is_shooting);
}

void SerialPort::writeData() {
  writeData(write_data_.symbol_yaw,   write_data_.yaw_angle,
            write_data_.symbol_pitch, write_data_.pitch_angle,
            write_data_.depth,        write_data_.data_type,
            write_data_.is_shooting);
}

uint8_t SerialPort::checksumCRC(unsigned char* buf, uint16_t len) {
  uint8_t check = 0;

  while (len--) { check = CRC8Tab[check ^ (*buf++)]; }

  return check;
}

void SerialPort::getDataForCRC(const int&     data_type, const int&     is_shooting,
                               const int&     _yaw,      const int16_t& yaw,
                               const int&     _pitch,    const int16_t& pitch,
                               const int16_t& depth) {
  crc_buff_[0]  = 0x53;
  crc_buff_[1]  = static_cast<unsigned char>(data_type);
  crc_buff_[2]  = static_cast<unsigned char>(is_shooting);
  crc_buff_[3]  = static_cast<unsigned char>(_yaw);
  crc_buff_[4]  = returnLowBit(yaw);
  crc_buff_[5]  = returnHighBit(yaw);
  crc_buff_[6]  = static_cast<unsigned char>(_pitch);
  crc_buff_[7]  = returnLowBit(pitch);
  crc_buff_[8]  = returnHighBit(pitch);
  crc_buff_[9]  = returnLowBit(depth);
  crc_buff_[10] = returnHighBit(depth);
}

void SerialPort::getDataForSend(const int&     data_type, const int&     is_shooting,
                                const int&     _yaw,      const int16_t& yaw,
                                const int&     _pitch,    const int16_t& pitch,
                                const int16_t& depth,     const uint8_t& CRC) {
  write_buff_[0]  = 0x53;
  write_buff_[1]  = static_cast<unsigned char>(data_type);
  write_buff_[2]  = static_cast<unsigned char>(is_shooting);
  write_buff_[3]  = static_cast<unsigned char>(_yaw);
  write_buff_[4]  = returnLowBit(yaw);
  write_buff_[5]  = returnHighBit(yaw);
  write_buff_[6]  = static_cast<unsigned char>(_pitch);
  write_buff_[7]  = returnLowBit(pitch);
  write_buff_[8]  = returnHighBit(pitch);
  write_buff_[9]  = returnLowBit(depth);
  write_buff_[10] = returnHighBit(depth);
  write_buff_[11] = CRC & 0xff;
  write_buff_[12] = 0x45;
}

bool SerialPort::isEmpty() {
  if (receive_buff_[0] != '0' || receive_buff_[REC_INFO_LENGTH - 1] != '0') {
    return false;
  } else {
    return true;
  }
}

void SerialPort::updateReceiveInformation() {
  receiveData();

  if (isEmpty()) {
    receive_data_ = last_receive_data_;
  } else {
    last_receive_data_ = receive_data_;
  }

  for (size_t i = 0; i != sizeof(transform_arr_) / sizeof(transform_arr_[0]); ++i) {
    transform_arr_[i] = receive_buff_[i + 1] - '0';
  }

  switch (transform_arr_[0]) {
    case RED:
      receive_data_.my_color = RED;
      break;
    case BLUE:
      receive_data_.my_color = BLUE;
      break;
    default:
      receive_data_.my_color = RED;
      break;
  }

  switch (transform_arr_[1]) {
    case SUP_SHOOT:
      receive_data_.now_run_mode = SUP_SHOOT;
      break;
    case ENERGY_AGENCY:
      receive_data_.now_run_mode = ENERGY_AGENCY;
      break;
    case SENTRY_MODE:
      receive_data_.now_run_mode = SENTRY_MODE;
      break;
    case BASE_MODE:
      receive_data_.now_run_mode = BASE_MODE;
      break;
    default:
      receive_data_.now_run_mode = SUP_SHOOT;
      break;
  }

  switch (transform_arr_[2]) {
    case HERO:
      receive_data_.my_robot_id = HERO;
      break;
    case ENGINEERING:
      receive_data_.my_robot_id = ENGINEERING;
      break;
    case INFANTRY:
      receive_data_.my_robot_id = INFANTRY;
      break;
    case UAV:
      receive_data_.my_robot_id = UAV;
      break;
    case SENTRY:
      receive_data_.my_robot_id = SENTRY;
      break;
    default:
      receive_data_.my_robot_id = INFANTRY;
      break;
  }

  transform_arr_[3] = receive_buff_[14] - '0';
  switch (transform_arr_[3]) {
    case 1:
      receive_data_.bullet_velocity = 15;
      break;
    case 2:
      receive_data_.bullet_velocity = 18;
      break;
    case 3:
      receive_data_.bullet_velocity = 30;
      break;
    default:
      receive_data_.bullet_velocity = 30;
      break;
  }
  receive_data_.bullet_velocity = receive_buff_[14];

  for (size_t i = 0; i != sizeof(receive_data_.Receive_Yaw_Angle_Info.arr_yaw_angle); ++i) {
    receive_data_.Receive_Yaw_Angle_Info.arr_yaw_angle[i] = receive_buff_[i + 4];
  }

  for (size_t i = 0; i != sizeof(this->receive_data_.Receive_Pitch_Angle_Info.arr_pitch_angle); ++i) {
    receive_data_.Receive_Pitch_Angle_Info.arr_pitch_angle[i] = receive_buff_[i + 8];
  }

  receive_data_.acceleration = SerialPort::mergeIntoBytes(receive_buff_[12], receive_buff_[13]) / 100.f;
}

}  // namespace uart
