#include "RuneTracker.h"

namespace power_rune {

void RuneTracker::update(const State& state) {
    m_history.push_back(state);
    if(m_history.size() > m_maxHistorySize) {
        m_history.pop_front();
    }
}

cv::Point2f RuneTracker::predict(double predict_time) {
    if(m_history.size() < 2) return m_history.back().center;
    
    // 获取当前状态
    const auto& current = m_history.back();
    
    // 计算当前角速度
    double current_speed = getCurrentSpeed();
    
    // 预测角度
    double predict_angle = current.angle + current_speed * predict_time;
    
    // 计算预测点位置
    double radius = cv::norm(current.center - cv::Point2f(0, 0)); // 假设圆心在原点
    return cv::Point2f(radius * cos(predict_angle), radius * sin(predict_angle));
}

double RuneTracker::getCurrentSpeed() {
    if(m_history.size() < 2) return 0.0;
    
    // 使用最近几帧计算平均角速度
    double avg_speed = 0.0;
    int count = 0;
    
    for(size_t i = m_history.size() - 1; i > 0 && i > m_history.size() - 5; --i) {
        avg_speed += calculateSpeed(m_history[i], m_history[i-1]);
        count++;
    }
    
    return count > 0 ? avg_speed / count : 0.0;
}

double RuneTracker::calculateSpeed(const State& current, const State& last) {
    double dt = current.timestamp - last.timestamp;
    if(dt <= 0) return 0.0;
    
    double dtheta = current.angle - last.angle;
    // 处理角度跳变
    if(dtheta > M_PI) dtheta -= 2*M_PI;
    if(dtheta < -M_PI) dtheta += 2*M_PI;
    
    return dtheta / dt;
}

} // namespace power_rune 