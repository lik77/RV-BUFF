#pragma once
#include <deque>
#include <opencv2/opencv.hpp>

namespace power_rune {

class RuneTracker {
public:
    struct State {
        double timestamp;      // 时间戳
        double angle;         // 当前角度
        double angular_speed; // 角速度
        cv::Point2f center;   // 中心点
    };

    RuneTracker(size_t history_size = 20) : m_maxHistorySize(history_size) {}
    
    // 更新状态
    void update(const State& state);
    
    // 预测下一个位置
    cv::Point2f predict(double predict_time);
    
    // 获取当前速度
    double getCurrentSpeed() const;
    
private:
    std::deque<State> m_history;
    size_t m_maxHistorySize;
    
    // 计算角速度
    double calculateSpeed(const State& current, const State& last);
};

} // namespace power_rune 