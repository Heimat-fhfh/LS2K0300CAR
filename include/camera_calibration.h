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

    // 对二值图执行去畸变，使用最近邻插值保持 0/255 二值特性。
    bool correctBinary(const cv::Mat &src, cv::Mat &dst);

    // 生成浮点去畸变映射表，便于与其它几何变换融合成单次 remap。
    bool buildUndistortFloatMap(const cv::Size &frameSize, cv::Mat &mapX, cv::Mat &mapY);

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
