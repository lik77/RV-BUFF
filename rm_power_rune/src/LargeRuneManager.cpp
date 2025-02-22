class LargeRuneManager {
public:
    enum class State {
        WAITING,        // 等待稳定
        TRACKING,       // 跟踪中
        PREDICTING,     // 预测中
        SHOOTING       // 射击中
    };
    
    void update() {
        switch(m_state) {
        case State::WAITING:
            if (isStable()) {
                m_state = State::TRACKING;
            }
            break;
            
        case State::TRACKING:
            if (hasEnoughData()) {
                m_state = State::PREDICTING;
            }
            break;
            
        case State::PREDICTING:
            if (canShoot()) {
                m_state = State::SHOOTING;
            }
            break;
            
        case State::SHOOTING:
            if (needRetrack()) {
                m_state = State::TRACKING;
            }
            break;
        }
    }
    
private:
    bool isStable() {
        return m_tracker.getSpeedStability() > 0.9;
    }
    
    bool hasEnoughData() {
        return m_tracker.getDataCount() >= MIN_DATA_COUNT;
    }
    
    bool canShoot() {
        return m_predictor.getConfidence() > 0.8;
    }
}; 