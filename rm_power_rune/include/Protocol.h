#pragma once
#include <stdint.h>
#include "Param.h"

namespace power_rune {

#pragma pack(1)  // 1字节对齐

// 电控发送给视觉的数据包
struct ControlToVision {
    uint8_t header[2] = {0xA5, 0x5A};  // 包头
    
    // 云台数据
    float yaw;              // 当前yaw角度
    float pitch;            // 当前pitch角度
    float roll;             // 当前roll角度
    
    // 发射数据
    float bullet_speed;     // 子弹射速
    uint8_t shoot_freq;     // 射频
    uint8_t remain_heat;    // 剩余热量
    
    // 状态数据
    uint8_t mode;          // 当前模式(对应您的Mode枚举)
    uint8_t target_type;   // 目标类型(小能量机关/大能量机关)
    bool shoot_ready;      // 发射机构就绪
    
    uint8_t crc8;         // CRC校验
};

// 视觉发送给电控的数据包
struct VisionToControl {
    uint8_t header[2] = {0xA5, 0x5A};  // 包头
    
    // 预测位置(对应Calculator的输出)
    float predict_yaw;      // 预测yaw
    float predict_pitch;    // 预测pitch
    
    // 状态信息
    bool target_found;      // 是否识别到目标
    uint8_t target_type;    // 目标类型(对应RuneType)
    uint8_t shoot_cmd;      // 发射指令
    
    uint8_t crc8;          // CRC校验
};
#pragma pack()

} // namespace power_rune 