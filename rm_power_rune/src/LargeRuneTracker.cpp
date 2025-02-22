bool LargeRuneTracker::track(const cv::Point2f& center, const cv::Point2f& armor) {
    double current_time = std::chrono::duration_cast<std::chrono::milliseconds>
                         (std::chrono::steady_clock::now().time_since_epoch()).count() / 1000.0;
    
    if (m_firstTrack) {
        m_firstTrack = false;
        m_startTime = current_time;
        m_lastAngle = calculateAngle(center, armor);
        return true;
    }
    
    // 计算当前角度
    double current_angle = calculateAngle(center, armor);
    
    // 计算角速度
    double dt = current_time - m_lastTime;
    double dtheta = current_angle - m_lastAngle;
    
    // 处理角度跳变
    if (dtheta > M_PI) dtheta -= 2*M_PI;
    if (dtheta < -M_PI) dtheta += 2*M_PI;
    
    // 更新速度
    m_currentSpeed = dtheta / dt;
    
    // 更新加速度
    double dw = m_currentSpeed - m_lastSpeed;
    m_currentAccel = dw / dt;
    
    // 更新状态
    m_lastTime = current_time;
    m_lastAngle = current_angle;
    m_lastSpeed = m_currentSpeed;
    
    return true;
} 