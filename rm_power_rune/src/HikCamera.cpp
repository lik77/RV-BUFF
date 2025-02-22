#include "HikCamera.h"

namespace power_rune {

HikCamera::HikCamera() : m_handle(nullptr), m_pData(nullptr), m_isGrabbing(false) {
}

HikCamera::~HikCamera() {
    release();
}

bool HikCamera::init() {
    // 1. 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList) != MV_OK) {
        return false;
    }

    // 2. 创建句柄
    if (MV_CC_CreateHandle(&m_handle, stDeviceList.pDeviceInfo[0]) != MV_OK) {
        return false;
    }

    // 3. 打开设备
    if (MV_CC_OpenDevice(m_handle) != MV_OK) {
        return false;
    }

    // 4. 设置触发模式为off
    if (MV_CC_SetEnumValue(m_handle, "TriggerMode", 0) != MV_OK) {
        return false;
    }

    // 5. 设置像素格式为BGR8
    if (MV_CC_SetEnumValue(m_handle, "PixelFormat", PixelType_Gvsp_BGR8_Packed) != MV_OK) {
        return false;
    }

    return true;
}

bool HikCamera::start() {
    // 开始取流
    if (MV_CC_StartGrabbing(m_handle) != MV_OK) {
        return false;
    }
    m_isGrabbing = true;
    return true;
}

bool HikCamera::read(cv::Mat& frame) {
    if (!m_isGrabbing) return false;

    // 获取一帧图像
    if (MV_CC_GetOneFrameTimeout(m_handle, m_pData, 
        m_stImageInfo.nFrameLen, &m_stImageInfo, 1000) != MV_OK) {
        return false;
    }

    if (!convertToMat(frame)) {
        return false;
    }
    
    // 应用图像预处理
    if (m_preprocessor) {
        m_preprocessor->process(frame);
    }
    
    return true;
}

void HikCamera::release() {
    if (m_isGrabbing) {
        MV_CC_StopGrabbing(m_handle);
    }
    if (m_handle) {
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
    }
    if (m_pData) {
        free(m_pData);
        m_pData = nullptr;
    }
}

bool HikCamera::setExposureTime(float exposure) {
    return MV_CC_SetFloatValue(m_handle, "ExposureTime", exposure) == MV_OK;
}

bool HikCamera::setGain(float gain) {
    return MV_CC_SetFloatValue(m_handle, "Gain", gain) == MV_OK;
}

bool HikCamera::setFrameRate(float fps) {
    return MV_CC_SetFloatValue(m_handle, "AcquisitionFrameRate", fps) == MV_OK;
}

bool HikCamera::convertToMat(cv::Mat& image) {
    if (!m_pData || !m_handle) return false;

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    
    // 获取一帧图像
    unsigned char* pData = (unsigned char*)malloc(m_stImageInfo.nFrameLen);
    if (pData == nullptr) {
        return false;
    }
    
    // 从相机获取数据
    if (MV_CC_GetOneFrameTimeout(m_handle, pData, m_stImageInfo.nFrameLen, &stImageInfo, 1000) != MV_OK) {
        free(pData);
        return false;
    }

    // 转换为OpenCV格式
    switch(stImageInfo.enPixelType) {
    case PixelType_Gvsp_BGR8_Packed:
        image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData).clone();
        break;
    case PixelType_Gvsp_Mono8:
        image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData).clone();
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        break;
    default:
        free(pData);
        return false;
    }

    free(pData);
    return true;
}

bool HikCamera::loadConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        auto camera_config = config["Camera"];
        
        // 设置曝光
        bool exposure_auto = camera_config["exposure_auto"].as<bool>();
        MV_CC_SetEnumValue(m_handle, "ExposureAuto", exposure_auto ? 2 : 0);
        if (!exposure_auto) {
            setExposureTime(camera_config["exposure_time"].as<float>());
        }
        
        // 设置增益
        bool gain_auto = camera_config["gain_auto"].as<bool>();
        MV_CC_SetEnumValue(m_handle, "GainAuto", gain_auto ? 2 : 0);
        if (!gain_auto) {
            setGain(camera_config["gain"].as<float>());
        }
        
        // 设置帧率
        setFrameRate(camera_config["frame_rate"].as<float>());
        
        // 设置ROI
        auto roi = camera_config["roi"];
        if (roi["enable"].as<bool>()) {
            setROI(roi["x"].as<int>(), 
                  roi["y"].as<int>(),
                  roi["width"].as<int>(), 
                  roi["height"].as<int>());
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load camera config: " << e.what() << std::endl;
        return false;
    }
}

} // namespace power_rune 