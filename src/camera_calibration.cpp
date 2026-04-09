#include "camera_calibration.h"

#include <fstream>

namespace
{
bool EndsWith(const std::string &value, const std::string &suffix)
{
    if (value.size() < suffix.size())
    {
        return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
}

bool JsonToDouble(const nlohmann::json &node, double &out)
{
    if (!node.is_number())
    {
        return false;
    }
    out = node.get<double>();
    return true;
}
}

bool CameraCalibrationCorrector::load(const std::string &configPath, std::string *errorMessage)
{
    loaded_ = false;
    cameraMatrix_.release();
    distCoeffs_.release();
    map1_.release();
    map2_.release();
    mapSize_ = cv::Size();

    if (EndsWith(configPath, ".json"))
    {
        return loadFromJson(configPath, errorMessage);
    }
    return loadFromYaml(configPath, errorMessage);
}

bool CameraCalibrationCorrector::isReady() const
{
    return loaded_;
}

bool CameraCalibrationCorrector::correct(const cv::Mat &src, cv::Mat &dst)
{
    if (!loaded_ || src.empty())
    {
        return false;
    }

    buildUndistortMapIfNeeded(src.size());
    if (map1_.empty() || map2_.empty())
    {
        return false;
    }

    cv::remap(src, dst, map1_, map2_, cv::INTER_LINEAR);
    return !dst.empty();
}

bool CameraCalibrationCorrector::loadFromYaml(const std::string &configPath, std::string *errorMessage)
{
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        if (errorMessage)
        {
            *errorMessage = "无法打开 YAML 标定文件: " + configPath;
        }
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["dist_coeffs"] >> distCoeffs_;

    int width = 0;
    int height = 0;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    calibratedSize_ = cv::Size(width, height);

    if (cameraMatrix_.empty() || distCoeffs_.empty())
    {
        if (errorMessage)
        {
            *errorMessage = "YAML 缺少 camera_matrix 或 dist_coeffs";
        }
        return false;
    }

    cameraMatrix_.convertTo(cameraMatrix_, CV_64F);
    distCoeffs_.convertTo(distCoeffs_, CV_64F);
    loaded_ = true;
    return true;
}

bool CameraCalibrationCorrector::parseMatrixFromJson(const nlohmann::json &node, int rows, int cols, cv::Mat &out)
{
    if (!node.is_array() || static_cast<int>(node.size()) != rows * cols)
    {
        return false;
    }

    out = cv::Mat(rows, cols, CV_64F);
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            double value = 0.0;
            if (!JsonToDouble(node[r * cols + c], value))
            {
                return false;
            }
            out.at<double>(r, c) = value;
        }
    }
    return true;
}

bool CameraCalibrationCorrector::loadFromJson(const std::string &configPath, std::string *errorMessage)
{
    std::ifstream ifs(configPath);
    if (!ifs.is_open())
    {
        if (errorMessage)
        {
            *errorMessage = "无法打开 JSON 标定文件: " + configPath;
        }
        return false;
    }

    nlohmann::json root;
    try
    {
        ifs >> root;
    }
    catch (const std::exception &e)
    {
        if (errorMessage)
        {
            *errorMessage = std::string("JSON 解析失败: ") + e.what();
        }
        return false;
    }

    if (!root.contains("camera_matrix") || !root.contains("dist_coeffs"))
    {
        if (errorMessage)
        {
            *errorMessage = "JSON 缺少 camera_matrix 或 dist_coeffs";
        }
        return false;
    }

    if (!parseMatrixFromJson(root["camera_matrix"], 3, 3, cameraMatrix_))
    {
        if (errorMessage)
        {
            *errorMessage = "camera_matrix 格式错误，应为 9 个数的一维数组";
        }
        return false;
    }

    const nlohmann::json &distNode = root["dist_coeffs"];
    if (!distNode.is_array() || distNode.empty())
    {
        if (errorMessage)
        {
            *errorMessage = "dist_coeffs 格式错误，应为数值数组";
        }
        return false;
    }
    distCoeffs_ = cv::Mat(1, static_cast<int>(distNode.size()), CV_64F);
    for (int i = 0; i < distCoeffs_.cols; ++i)
    {
        double value = 0.0;
        if (!JsonToDouble(distNode[i], value))
        {
            if (errorMessage)
            {
                *errorMessage = "dist_coeffs 包含非数值元素";
            }
            return false;
        }
        distCoeffs_.at<double>(0, i) = value;
    }

    int width = root.value("image_width", 0);
    int height = root.value("image_height", 0);
    calibratedSize_ = cv::Size(width, height);

    loaded_ = true;
    return true;
}

void CameraCalibrationCorrector::buildUndistortMapIfNeeded(const cv::Size &frameSize)
{
    if (frameSize.width <= 0 || frameSize.height <= 0)
    {
        return;
    }
    if (mapSize_ == frameSize && !map1_.empty() && !map2_.empty())
    {
        return;
    }

    cv::Mat optimizedCameraMatrix = cv::getOptimalNewCameraMatrix(
        cameraMatrix_,
        distCoeffs_,
        frameSize,
        1.0,
        frameSize);

    cv::initUndistortRectifyMap(
        cameraMatrix_,
        distCoeffs_,
        cv::Mat(),
        optimizedCameraMatrix,
        frameSize,
        CV_16SC2,
        map1_,
        map2_);

    mapSize_ = frameSize;
}
