#pragma once
#include <opencv2/opencv.hpp>

namespace power_rune {

class ImagePreprocess {
public:
    ImagePreprocess();
    
    // 加载配置
    bool loadConfig(const std::string& config_path);
    
    // 图像预处理
    void process(cv::Mat& image);
    
private:
    // 亮度对比度调整
    void adjustBrightnessContrast(cv::Mat& image);
    
    // 白平衡调整
    void adjustWhiteBalance(cv::Mat& image);
    
    // 图像去噪
    void denoiseImage(cv::Mat& image);
    
    // Gamma校正
    void gammaCorrection(cv::Mat& image);
    
    struct Config {
        float brightness;
        float contrast;
        float r_gain;
        float g_gain;
        float b_gain;
        bool denoise_enable;
        int denoise_h;
        int template_window_size;
        int search_window_size;
        float gamma;
    } m_config;
};

} // namespace power_rune 