#pragma once
#include <string>
#include "Protocol.h"

namespace power_rune {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool init(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    bool receive(ControlToVision& data);
    bool send(const VisionToControl& data);
    
private:
    int m_fd;  // 串口文件描述符
    
    // 数据处理函数
    bool verifyHeader(const uint8_t* data);
    uint8_t calculateCRC8(const uint8_t* data, int len);
    void packData(const VisionToControl& src, uint8_t* dst);
    bool unpackData(const uint8_t* src, ControlToVision& dst);
};

} // namespace power_rune 