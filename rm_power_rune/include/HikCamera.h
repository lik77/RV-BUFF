#pragma once
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ImagePreprocess.h"

namespace power_rune {

class HikCamera {
public:
    HikCamera();
    ~HikCamera();

    bool init();  // 初始化相机
    bool start(); // 开始采集
    bool read(cv::Mat& frame); // 读取一帧
    void release(); // 释放相机

    // 设置相机参数
    bool setExposureTime(float exposure);
    bool setGain(float gain);
    bool setFrameRate(float fps);
    bool setROI(int x, int y, int width, int height);

    // 设置预处理器
    void setPreprocessor(std::shared_ptr<ImagePreprocess> preprocessor) {
        m_preprocessor = preprocessor;
    }

private:
    void* m_handle;
    unsigned char* m_pData;
    MV_FRAME_OUT_INFO_EX m_stImageInfo;
    bool m_isGrabbing;
    
    // 将原始图像转换为OpenCV格式
    bool convertToMat(cv::Mat& image);
    std::shared_ptr<ImagePreprocess> m_preprocessor;
};

} // namespace power_rune 