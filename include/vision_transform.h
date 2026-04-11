#pragma once

#include "common_system.h"
#include "camera_calibration.h"

class VisionTransformPipeline
{
public:
    VisionTransformPipeline() = default;

    // 读取并缓存统一的视觉变换配置。
    bool loadConfig(const std::string &configPath, std::string *errorMessage = nullptr);

    // 执行单次 remap；当配置关闭时直接透传输入图像。
    bool apply(const cv::Mat &src, cv::Mat &dst, int interpolation);

    bool isEnabled() const;
    cv::Size outputSize() const;

private:
    bool initializeIfNeeded(const cv::Size &inputSize, std::string *errorMessage = nullptr);
    bool buildPerspectiveMap(const cv::Size &inputSize, cv::Mat &mapX, cv::Mat &mapY);
    bool composeWithUndistortMap(const cv::Mat &perspectiveMapX,
                                 const cv::Mat &perspectiveMapY,
                                 const cv::Mat &undistortMapX,
                                 const cv::Mat &undistortMapY,
                                 cv::Mat &outMapX,
                                 cv::Mat &outMapY);

    bool configLoaded_ = false;
    bool initialized_ = false;

    bool enable_ = false;
    bool undistortEnable_ = false;
    bool inversePerspectiveEnable_ = false;

    cv::Size expectedInputSize_;
    cv::Size outputSize_;
    cv::Mat perspectiveMatrix_; // 3x3, CV_64F

    std::string calibrationFilePath_;
    CameraCalibrationCorrector calibrationCorrector_;

    cv::Size mapInputSize_;
    cv::Mat fusedMapX_;
    cv::Mat fusedMapY_;
};
