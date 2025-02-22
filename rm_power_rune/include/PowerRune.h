#pragma once

#include "Calculator.h"
#include "Detector.h"
#include "Param.h"
#include "Utility.h"

#define CONFIG_PATH "../config.yaml"

namespace power_rune {

enum class RuneType {
    SMALL_RUNE,      // 小能量机关
    LARGE_RUNE,      // 大能量机关
    OUTPOST_RUNE     // 前哨站能量机关
};

class PowerRune {
   public:
    PowerRune(RuneType type = RuneType::SMALL_RUNE);
    bool runOnce(const cv::Mat& image, double pitch, double yaw, double roll = 0.0);
    
    // 设置能量机关类型
    void setRuneType(RuneType type);

   private:
    RuneType m_runeType;
    Param m_param;
    Detector m_detector;
    Calculator m_calculator;
    
    // 激活状态
    enum class ActivationState {
        INACTIVE,       // 未激活
        ACTIVATING,     // 激活中
        ACTIVATED       // 已激活
    };
    
    struct HitRecord {
        double timestamp;      // 击打时间戳
        bool hit_success;      // 是否击中
    };
    
    ActivationState m_state;
    std::deque<HitRecord> m_hitRecords;
    int m_successHits;        // 成功击打次数
    
    // 激活相关函数
    bool checkActivation();
    void updateHitRecords();
    bool isTimeout();
};

}  // namespace power_rune
