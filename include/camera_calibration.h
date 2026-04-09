#pragma once

#include "common_system.h"

class CameraCalibrationCorrector
{
public:
    // 自动按扩展名加载配置：.yaml/.yml 使用 OpenCV FileStorage，.json 使用 nlohmann::json。
    bool load(const std::string &configPath, std::string *errorMessage = nullptr);

    bool isReady() const;

    // 对输入图像执行去畸变校正，成功返回 true。
    bool correct(const cv::Mat &src, cv::Mat &dst);

private:
    bool loadFromYaml(const std::string &configPath, std::string *errorMessage);
    bool loadFromJson(const std::string &configPath, std::string *errorMessage);
    bool parseMatrixFromJson(const nlohmann::json &node, int rows, int cols, cv::Mat &out);
    void buildUndistortMapIfNeeded(const cv::Size &frameSize);

    bool loaded_ = false;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Size calibratedSize_;

    cv::Mat map1_;
    cv::Mat map2_;
    cv::Size mapSize_;
};
