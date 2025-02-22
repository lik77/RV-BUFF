#include "ImagePreprocess.h"
#include <yaml-cpp/yaml.h>

namespace power_rune {

ImagePreprocess::ImagePreprocess() {
    // 设置默认参数
    m_config.brightness = 0;
    m_config.contrast = 1.0;
    m_config.r_gain = 1.0;
    m_config.g_gain = 1.0;
    m_config.b_gain = 1.0;
    m_config.denoise_enable = true;
    m_config.denoise_h = 10;
    m_config.template_window_size = 7;
    m_config.search_window_size = 21;
    m_config.gamma = 1.0;
}

bool ImagePreprocess::loadConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        auto image_process = config["ImageProcess"];
        
        m_config.brightness = image_process["brightness"].as<float>();
        m_config.contrast = image_process["contrast"].as<float>();
        
        auto wb = image_process["white_balance"];
        m_config.r_gain = wb["r_gain"].as<float>();
        m_config.g_gain = wb["g_gain"].as<float>();
        m_config.b_gain = wb["b_gain"].as<float>();
        
        auto denoise = image_process["denoise"];
        m_config.denoise_enable = denoise["enable"].as<bool>();
        m_config.denoise_h = denoise["h"].as<int>();
        m_config.template_window_size = denoise["template_window_size"].as<int>();
        m_config.search_window_size = denoise["search_window_size"].as<int>();
        
        m_config.gamma = image_process["gamma"].as<float>();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return false;
    }
}

void ImagePreprocess::process(cv::Mat& image) {
    // 1. 亮度对比度调整
    adjustBrightnessContrast(image);
    
    // 2. 白平衡调整
    adjustWhiteBalance(image);
    
    // 3. 图像去噪
    if (m_config.denoise_enable) {
        denoiseImage(image);
    }
    
    // 4. Gamma校正
    gammaCorrection(image);
}

void ImagePreprocess::adjustBrightnessContrast(cv::Mat& image) {
    image.convertTo(image, -1, m_config.contrast, m_config.brightness);
}

void ImagePreprocess::adjustWhiteBalance(cv::Mat& image) {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    channels[0] *= m_config.b_gain;  // B通道
    channels[1] *= m_config.g_gain;  // G通道
    channels[2] *= m_config.r_gain;  // R通道
    
    cv::merge(channels, image);
}

void ImagePreprocess::denoiseImage(cv::Mat& image) {
    cv::fastNlMeansDenoisingColored(image, image,
                                   m_config.denoise_h,
                                   m_config.denoise_h,
                                   m_config.template_window_size,
                                   m_config.search_window_size);
}

void ImagePreprocess::gammaCorrection(cv::Mat& image) {
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for(int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, m_config.gamma) * 255.0);
    }
    cv::LUT(image, lookUpTable, image);
}

} // namespace power_rune 