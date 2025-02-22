#include "PowerRune.h"

#include <thread>

namespace power_rune {

extern std::mutex MUTEX;

PowerRune::PowerRune() : m_param{CONFIG_PATH} {}

bool PowerRune::runOnce(const cv::Mat& frame) {
    // 1. 目标检测
    if(!m_detector.detect(frame)) {
        return false;
    }
    
    // 2. 计算击打策略
    if(!m_calculator.calculateSmallRuneStrategy()) {
        return false;
    }
    
    // 3. 判断是否击中
    bool hit = checkHit();
    
    // 4. 记录击打结果
    m_hitRecords.push_back({
        std::chrono::duration_cast<std::chrono::seconds>
        (std::chrono::steady_clock::now().time_since_epoch()).count(),
        hit
    });
    
    // 5. 检查激活状态
    bool activated = checkActivation();
    
    // 6. 可视化显示
    if(SHOW_IMAGE) {
        visualize(frame, activated);
    }
    
    return activated;
}

bool PowerRune::checkActivation() {
    // 清理超时记录
    updateHitRecords();
    
    switch(m_state) {
    case ActivationState::INACTIVE:
        // 第一次击中，进入激活中状态
        if(m_hitRecords.size() > 0 && m_hitRecords.back().hit_success) {
            m_state = ActivationState::ACTIVATING;
            m_successHits = 1;
        }
        break;
        
    case ActivationState::ACTIVATING:
        // 检查是否超时
        if(isTimeout()) {
            m_state = ActivationState::INACTIVE;
            m_successHits = 0;
            m_hitRecords.clear();
            return false;
        }
        
        // 统计成功击打次数
        if(m_hitRecords.back().hit_success) {
            m_successHits++;
        }
        
        // 判断是否激活成功
        if(m_successHits >= SmallRuneParam::ACTIVATE_HITS) {
            m_state = ActivationState::ACTIVATED;
            return true;
        }
        break;
        
    case ActivationState::ACTIVATED:
        // 已激活状态
        return true;
    }
    
    return false;
}

void PowerRune::updateHitRecords() {
    double current_time = std::chrono::duration_cast<std::chrono::seconds>
                         (std::chrono::steady_clock::now().time_since_epoch()).count();
    
    // 移除超时记录
    while(!m_hitRecords.empty()) {
        if(current_time - m_hitRecords.front().timestamp > SmallRuneParam::HIT_TIMEOUT) {
            m_hitRecords.pop_front();
        } else {
            break;
        }
    }
}

bool PowerRune::isTimeout() {
    if(m_hitRecords.empty()) return true;
    
    double current_time = std::chrono::duration_cast<std::chrono::seconds>
                         (std::chrono::steady_clock::now().time_since_epoch()).count();
    
    return (current_time - m_hitRecords.front().timestamp) > SmallRuneParam::HIT_TIMEOUT;
}

}  // namespace power_rune