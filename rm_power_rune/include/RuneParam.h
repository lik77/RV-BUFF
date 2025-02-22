struct LargeRuneParam {
    // 基本参数
    static constexpr double RADIUS = 800.0;                 // 半径800mm
    static constexpr double ARMOR_WIDTH = 230.0;           // 装甲板宽度
    static constexpr double ARMOR_HEIGHT = 127.0;          // 装甲板高度
    
    // 运动参数
    static constexpr double INITIAL_SPEED = 0.780;         // 初始角速度(rad/s)
    static constexpr double ACCELERATION = 1.884;          // 角加速度(rad/s^2)
    static constexpr double MAX_SPEED = 2.0;              // 最大角速度(rad/s)
    
    // 预测参数
    static constexpr double MIN_PREDICT_TIME = 0.1;       // 最小预测时间(s)
    static constexpr double MAX_PREDICT_TIME = 1.0;       // 最大预测时间(s)
    static constexpr double STABLE_SPEED_THRESHOLD = 0.1; // 稳定速度阈值
}; 