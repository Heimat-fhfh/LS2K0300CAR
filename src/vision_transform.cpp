#include "vision_transform.h"

namespace
{
using json = nlohmann::json;

bool readMatrix3x3(const json &node, cv::Mat &matrix)
{
    if (!node.is_array() || node.size() != 9)
    {
        return false;
    }

    matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i)
    {
        if (!node[i].is_number())
        {
            return false;
        }
        matrix.at<double>(i / 3, i % 3) = node[i].get<double>();
    }
    return true;
}

bool readSizeNode(const json &node, cv::Size &size)
{
    if (!node.is_object())
    {
        return false;
    }
    if (!node.contains("width") || !node.contains("height"))
    {
        return false;
    }

    if (!node["width"].is_number_integer() || !node["height"].is_number_integer())
    {
        return false;
    }

    const int width = node["width"].get<int>();
    const int height = node["height"].get<int>();
    if (width <= 0 || height <= 0)
    {
        return false;
    }

    size = cv::Size(width, height);
    return true;
}

bool sampleMapBilinear(const cv::Mat &map, float x, float y, float &value)
{
    if (map.empty() || map.type() != CV_32FC1)
    {
        return false;
    }

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if (x0 < 0 || y0 < 0 || x1 >= map.cols || y1 >= map.rows)
    {
        return false;
    }

    const float dx = x - static_cast<float>(x0);
    const float dy = y - static_cast<float>(y0);

    const float v00 = map.at<float>(y0, x0);
    const float v01 = map.at<float>(y0, x1);
    const float v10 = map.at<float>(y1, x0);
    const float v11 = map.at<float>(y1, x1);

    value = (1.0f - dx) * (1.0f - dy) * v00 +
            dx * (1.0f - dy) * v01 +
            (1.0f - dx) * dy * v10 +
            dx * dy * v11;
    return true;
}
} // namespace

bool VisionTransformPipeline::loadConfig(const std::string &configPath, std::string *errorMessage)
{
    configLoaded_ = false;
    initialized_ = false;
    enable_ = false;
    undistortEnable_ = false;
    inversePerspectiveEnable_ = false;
    expectedInputSize_ = cv::Size();
    outputSize_ = cv::Size();
    perspectiveMatrix_.release();
    calibrationFilePath_.clear();
    mapInputSize_ = cv::Size();
    fusedMapX_.release();
    fusedMapY_.release();

    std::ifstream ifs(configPath);
    if (!ifs.is_open())
    {
        if (errorMessage)
        {
            *errorMessage = "无法打开视觉变换配置文件: " + configPath;
        }
        return false;
    }

    json root;
    try
    {
        ifs >> root;
    }
    catch (const std::exception &e)
    {
        if (errorMessage)
        {
            *errorMessage = std::string("视觉变换配置 JSON 解析失败: ") + e.what();
        }
        return false;
    }

    if (!root.is_object())
    {
        if (errorMessage)
        {
            *errorMessage = "视觉变换配置根节点必须是 object";
        }
        return false;
    }

    enable_ = root.value("enable", false);

    if (!readSizeNode(root.value("input_size", json::object()), expectedInputSize_))
    {
        if (errorMessage)
        {
            *errorMessage = "input_size 格式错误，必须包含 width/height 正整数";
        }
        return false;
    }

    if (!readSizeNode(root.value("output_size", json::object()), outputSize_))
    {
        if (errorMessage)
        {
            *errorMessage = "output_size 格式错误，必须包含 width/height 正整数";
        }
        return false;
    }

    const json undistortNode = root.value("undistort", json::object());
    if (!undistortNode.is_object())
    {
        if (errorMessage)
        {
            *errorMessage = "undistort 节点格式错误";
        }
        return false;
    }
    undistortEnable_ = undistortNode.value("enable", false);
    calibrationFilePath_ = undistortNode.value("calibration_file", std::string("../config/calibration.yaml"));

    const json inversePerspectiveNode = root.value("inverse_perspective", json::object());
    if (!inversePerspectiveNode.is_object())
    {
        if (errorMessage)
        {
            *errorMessage = "inverse_perspective 节点格式错误";
        }
        return false;
    }
    inversePerspectiveEnable_ = inversePerspectiveNode.value("enable", false);
    if (inversePerspectiveEnable_)
    {
        if (!readMatrix3x3(inversePerspectiveNode.value("matrix", json::array()), perspectiveMatrix_))
        {
            if (errorMessage)
            {
                *errorMessage = "inverse_perspective.matrix 格式错误，必须为 9 个数";
            }
            return false;
        }
    }

    configLoaded_ = true;
    return true;
}

bool VisionTransformPipeline::isEnabled() const
{
    return configLoaded_ && enable_ && (undistortEnable_ || inversePerspectiveEnable_);
}

cv::Size VisionTransformPipeline::outputSize() const
{
    return outputSize_;
}

bool VisionTransformPipeline::buildPerspectiveMap(const cv::Size &inputSize, cv::Mat &mapX, cv::Mat &mapY)
{
    mapX = cv::Mat(outputSize_.height, outputSize_.width, CV_32FC1, cv::Scalar(-1.0f));
    mapY = cv::Mat(outputSize_.height, outputSize_.width, CV_32FC1, cv::Scalar(-1.0f));

    if (!inversePerspectiveEnable_)
    {
        const int copyWidth = std::min(inputSize.width, outputSize_.width);
        const int copyHeight = std::min(inputSize.height, outputSize_.height);
        for (int y = 0; y < copyHeight; ++y)
        {
            for (int x = 0; x < copyWidth; ++x)
            {
                mapX.at<float>(y, x) = static_cast<float>(x);
                mapY.at<float>(y, x) = static_cast<float>(y);
            }
        }
        return true;
    }

    for (int y = 0; y < outputSize_.height; ++y)
    {
        for (int x = 0; x < outputSize_.width; ++x)
        {
            const double denominator = perspectiveMatrix_.at<double>(2, 0) * x +
                                       perspectiveMatrix_.at<double>(2, 1) * y +
                                       perspectiveMatrix_.at<double>(2, 2);
            if (std::abs(denominator) < 1e-9)
            {
                continue;
            }

            const double sourceX = (perspectiveMatrix_.at<double>(0, 0) * x +
                                    perspectiveMatrix_.at<double>(0, 1) * y +
                                    perspectiveMatrix_.at<double>(0, 2)) /
                                   denominator;
            const double sourceY = (perspectiveMatrix_.at<double>(1, 0) * x +
                                    perspectiveMatrix_.at<double>(1, 1) * y +
                                    perspectiveMatrix_.at<double>(1, 2)) /
                                   denominator;

            if (sourceX >= 0.0 && sourceY >= 0.0 &&
                sourceX < static_cast<double>(inputSize.width - 1) &&
                sourceY < static_cast<double>(inputSize.height - 1))
            {
                mapX.at<float>(y, x) = static_cast<float>(sourceX);
                mapY.at<float>(y, x) = static_cast<float>(sourceY);
            }
        }
    }

    return true;
}

bool VisionTransformPipeline::composeWithUndistortMap(const cv::Mat &perspectiveMapX,
                                                      const cv::Mat &perspectiveMapY,
                                                      const cv::Mat &undistortMapX,
                                                      const cv::Mat &undistortMapY,
                                                      cv::Mat &outMapX,
                                                      cv::Mat &outMapY)
{
    outMapX = cv::Mat(outputSize_.height, outputSize_.width, CV_32FC1, cv::Scalar(-1.0f));
    outMapY = cv::Mat(outputSize_.height, outputSize_.width, CV_32FC1, cv::Scalar(-1.0f));

    for (int y = 0; y < outputSize_.height; ++y)
    {
        for (int x = 0; x < outputSize_.width; ++x)
        {
            const float undistortedX = perspectiveMapX.at<float>(y, x);
            const float undistortedY = perspectiveMapY.at<float>(y, x);

            if (undistortedX < 0.0f || undistortedY < 0.0f)
            {
                continue;
            }

            float rawX = -1.0f;
            float rawY = -1.0f;
            if (!sampleMapBilinear(undistortMapX, undistortedX, undistortedY, rawX) ||
                !sampleMapBilinear(undistortMapY, undistortedX, undistortedY, rawY))
            {
                continue;
            }

            outMapX.at<float>(y, x) = rawX;
            outMapY.at<float>(y, x) = rawY;
        }
    }

    return true;
}

bool VisionTransformPipeline::initializeIfNeeded(const cv::Size &inputSize, std::string *errorMessage)
{
    if (!configLoaded_)
    {
        if (errorMessage)
        {
            *errorMessage = "视觉变换配置尚未加载";
        }
        return false;
    }

    if (!enable_ || (!undistortEnable_ && !inversePerspectiveEnable_))
    {
        initialized_ = true;
        mapInputSize_ = inputSize;
        fusedMapX_.release();
        fusedMapY_.release();
        return true;
    }

    if (initialized_ && mapInputSize_ == inputSize && !fusedMapX_.empty() && !fusedMapY_.empty())
    {
        return true;
    }

    if (expectedInputSize_ != inputSize)
    {
        if (errorMessage)
        {
            *errorMessage = "输入尺寸与配置不一致，期望 " +
                            std::to_string(expectedInputSize_.width) + "x" +
                            std::to_string(expectedInputSize_.height) +
                            "，实际 " + std::to_string(inputSize.width) + "x" +
                            std::to_string(inputSize.height);
        }
        return false;
    }

    cv::Mat perspectiveMapX;
    cv::Mat perspectiveMapY;
    if (!buildPerspectiveMap(inputSize, perspectiveMapX, perspectiveMapY))
    {
        if (errorMessage)
        {
            *errorMessage = "逆透视映射表构建失败";
        }
        return false;
    }

    if (undistortEnable_)
    {
        std::string calibrationError;
        if (!calibrationCorrector_.isReady())
        {
            if (!calibrationCorrector_.load(calibrationFilePath_, &calibrationError))
            {
                if (errorMessage)
                {
                    *errorMessage = "去畸变参数加载失败: " + calibrationError;
                }
                return false;
            }
        }

        cv::Mat undistortMapX;
        cv::Mat undistortMapY;
        if (!calibrationCorrector_.buildUndistortFloatMap(inputSize, undistortMapX, undistortMapY))
        {
            if (errorMessage)
            {
                *errorMessage = "去畸变映射表构建失败";
            }
            return false;
        }

        if (!composeWithUndistortMap(
                perspectiveMapX,
                perspectiveMapY,
                undistortMapX,
                undistortMapY,
                fusedMapX_,
                fusedMapY_))
        {
            if (errorMessage)
            {
                *errorMessage = "融合映射表构建失败";
            }
            return false;
        }
    }
    else
    {
        fusedMapX_ = perspectiveMapX;
        fusedMapY_ = perspectiveMapY;
    }

    mapInputSize_ = inputSize;
    initialized_ = true;
    return true;
}

bool VisionTransformPipeline::apply(const cv::Mat &src, cv::Mat &dst, int interpolation)
{
    if (src.empty())
    {
        return false;
    }

    if (!configLoaded_)
    {
        return false;
    }

    std::string initError;
    if (!initializeIfNeeded(src.size(), &initError))
    {
        std::cerr << "[VisionTransform] 初始化失败: " << initError << std::endl;
        return false;
    }

    if (!enable_ || (!undistortEnable_ && !inversePerspectiveEnable_))
    {
        if (src.size() == outputSize_)
        {
            dst = src.clone();
        }
        else
        {
            cv::resize(src, dst, outputSize_, 0.0, 0.0, cv::INTER_NEAREST);
        }
        return true;
    }

    if (fusedMapX_.empty() || fusedMapY_.empty())
    {
        return false;
    }

    cv::remap(src,
              dst,
              fusedMapX_,
              fusedMapY_,
              interpolation,
              cv::BORDER_CONSTANT,
              cv::Scalar::all(0));
    return !dst.empty();
}
