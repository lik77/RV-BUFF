#include <thread>

#include "PowerRune.h"
#include "HikCamera.h"
#include "ImagePreprocess.h"
#include "SerialPort.h"

int main() {
    // 初始化串口
    power_rune::SerialPort serial;
    if (!serial.init()) {
        std::cerr << "Failed to initialize serial port!" << std::endl;
        return -1;
    }
    
    // 创建通信数据结构
    power_rune::ControlToVision recv_data;
    power_rune::VisionToControl send_data;
    
    // 创建能量机关识别对象，指定类型
    power_rune::PowerRune pr(power_rune::RuneType::LARGE_RUNE);
    
    // 加载对应类型的配置
    if(!pr.loadConfig("config/rune_config.yaml")) {
        std::cerr << "Failed to load rune config!" << std::endl;
        return -1;
    }
    
    // 2. 创建并初始化相机
    power_rune::HikCamera camera;
    if (!camera.init()) {
        std::cerr << "Failed to initialize camera!" << std::endl;
        return -1;
    }
    
    // 3. 加载相机配置
    if (!camera.loadConfig("config/camera_config.yaml")) {
        std::cerr << "Failed to load camera config!" << std::endl;
        return -1;
    }
    
    // 4. 创建图像预处理器
    auto preprocessor = std::make_shared<power_rune::ImagePreprocess>();
    if (!preprocessor->loadConfig("config/image_config.yaml")) {
        std::cerr << "Failed to load image process config!" << std::endl;
        return -1;
    }
    camera.setPreprocessor(preprocessor);
    
    // 5. 开始采集
    if (!camera.start()) {
        std::cerr << "Failed to start camera!" << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    while (true) {
        // 1. 接收电控数据
        if (serial.receive(recv_data)) {
            // 更新参数
            power_rune::Param::CURRENT_BULLET_SPEED = recv_data.bullet_speed;
            
            // 根据模式选择能量机关类型
            if (recv_data.target_type == 0) {
                pr.setRuneType(power_rune::RuneType::SMALL_RUNE);
            } else {
                pr.setRuneType(power_rune::RuneType::LARGE_RUNE);
            }
        }
        
        // 2. 处理图像
        if (camera.read(frame)) {
            if (pr.runOnce(frame, recv_data.pitch, recv_data.yaw, recv_data.roll)) {
                // 获取预测结果
                auto [predict_yaw, predict_pitch] = pr.getPredictAngle();
                
                // 打包发送数据
                send_data.predict_yaw = predict_yaw;
                send_data.predict_pitch = predict_pitch;
                send_data.target_found = true;
                send_data.target_type = static_cast<uint8_t>(pr.getRuneType());
                send_data.shoot_cmd = pr.canShoot() ? 1 : 0;
                
                // 发送数据
                serial.send(send_data);
            }
        }
    }
    
    return 0;
}