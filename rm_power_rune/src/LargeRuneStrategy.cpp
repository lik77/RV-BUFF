bool LargeRuneStrategy::calculateStrategy() {
    // 1. 判断当前状态
    RuneState current_state = analyzeState();
    
    switch(current_state) {
    case RuneState::ACCELERATING:
        // 加速阶段策略
        return handleAccelerating();
        
    case RuneState::UNIFORM:
        // 匀速阶段策略
        return handleUniform();
        
    case RuneState::UNSTABLE:
        // 不稳定状态策略
        return handleUnstable();
    }
    
    return false;
}

bool LargeRuneStrategy::handleAccelerating() {
    // 1. 获取当前速度和加速度
    double current_speed = m_tracker.getCurrentSpeed();
    double current_accel = m_tracker.getCurrentAccel();
    
    // 2. 判断是否接近理论值
    if (std::abs(current_accel - LargeRuneParam::ACCELERATION) > 0.1) {
        return false;
    }
    
    // 3. 预测击打点
    cv::Point2f predict_point = calculatePredictPoint(true);
    
    // 4. 验证预测点
    return validatePredictPoint(predict_point);
} 