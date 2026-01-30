// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
// 分类模型检测版本

#include <iostream>
#include <vector>
#include <getopt.h>
#include <algorithm>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include "inference.h"

using namespace std;
using namespace cv;

// 分类结果结构体
struct ClassificationResult {
    int class_id{0};
    std::string className{};
    float confidence{0.0};
};

// 分类模型推理函数
std::vector<ClassificationResult> runClassification(Inference& inf, const cv::Mat& input) {
    std::vector<ClassificationResult> results;
    
    // 运行推理
    std::vector<Detection> detections = inf.runInference(input);
    
    // 将检测结果转换为分类结果
    // 注意：对于分类模型，我们只关心置信度最高的类别
    if (!detections.empty()) {
        // 找到置信度最高的检测
        auto max_it = std::max_element(detections.begin(), detections.end(),
            [](const Detection& a, const Detection& b) {
                return a.confidence < b.confidence;
            });
        
        if (max_it != detections.end()) {
            ClassificationResult result;
            result.class_id = max_it->class_id;
            result.className = max_it->className;
            result.confidence = max_it->confidence;
            results.push_back(result);
        }
    }
    
    return results;
}

int main(int argc, char **argv)
{
    std::string projectBasePath = "/home/fhfh/Work/LS2K0300CAR/third_party/ultralytics";

    bool runOnGPU = false;

    // 使用分类模型
    // V1-cls.onnx 看起来是一个分类模型
    cout << "Loading classification model..." << std::endl;
    Inference inf("yolo/V1-cls.onnx", cv::Size(96, 96), "classes.txt", runOnGPU);
    cout << "Model loaded." << std::endl;

    // 测试图像
    std::vector<std::string> imageNames;
    imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/upload/pictures/re_dagger_001.jpg");
    imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/upload/pictures/explosive_001.jpg");
    imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/upload/pictures/dagger_001.jpg");
    imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/upload/pictures/first_aid_kit_001.jpg");
    
    cout << "Starting classification..." << std::endl;
    
    for (int i = 0; i < imageNames.size(); ++i)
    {
        cv::Mat frame = cv::imread(imageNames[i]);

        cout << "Reading image: " << imageNames[i] << std::endl;
        if (frame.empty()) {
            std::cerr << "Error: Could not load image" << std::endl;
            continue;
        }

        // 分类推理
        std::vector<ClassificationResult> results = runClassification(inf, frame);

        // 显示结果
        std::cout << "\n=== Classification Results for image " << i + 1 << " ===" << std::endl;
        std::cout << "Image: " << imageNames[i] << std::endl;
        
        if (results.empty()) {
            std::cout << "No classification results found." << std::endl;
        } else {
            for (const auto& result : results) {
                std::cout << "Class: " << result.className 
                          << " (ID: " << result.class_id << ")"
                          << " - Confidence: " << std::fixed << std::setprecision(4) 
                          << result.confidence * 100 << "%" << std::endl;
                
                // 在图像上显示分类结果
                std::string resultText = result.className + ": " + 
                                        std::to_string(result.confidence).substr(0, 4);
                
                // 在图像顶部显示结果
                int baseline = 0;
                cv::Size textSize = cv::getTextSize(resultText, cv::FONT_HERSHEY_SIMPLEX, 
                                                   1.0, 2, &baseline);
                
                // 创建背景矩形
                cv::Rect textRect(10, 10, textSize.width + 20, textSize.height + 20);
                cv::rectangle(frame, textRect, cv::Scalar(0, 0, 0), cv::FILLED);
                
                // 添加文本
                cv::putText(frame, resultText, cv::Point(20, 40), 
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            }
        }

        // 保存结果图像
        float scale = 0.8;
        cv::resize(frame, frame, cv::Size(frame.cols*scale, frame.rows*scale));
        std::string outputFileName = "classification_output_" + std::to_string(i) + ".jpg";
        cv::imwrite(outputFileName, frame);
        std::cout << "Saved result to " << outputFileName << std::endl;
        std::cout << "====================================\n" << std::endl;
    }
    
    std::cout << "Classification completed!" << std::endl;
    return 0;
}