// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include <iostream>
#include <vector>
#include <getopt.h>

#include <opencv2/opencv.hpp>

#include "inference.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    std::string projectBasePath = "/home/fhfh/Work/LS2K0300CAR/third_party/ultralytics"; // Set your ultralytics base path

    bool runOnGPU = false;

    //
    // Pass in either:
    //
    // "yolov8s.onnx" or "yolov5s.onnx"
    //
    // To run Inference with yolov8/yolov5 (ONNX)
    //

    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    // Inference inf("/home/fhfh/Work/LS2K0300CAR/third_party/ultralytics/yolo26n.onnx", cv::Size(160, 160), "classes.txt", runOnGPU);
    // Inference inf(projectBasePath + "/yolov8s.onnx", cv::Size(640, 640), "classes.txt", runOnGPU);
    cout << "Loading model..." << std::endl;
    Inference inf("yolo/upload/model/v8V3.onnx", cv::Size(96, 96), "classes.txt", runOnGPU);
    cout << "Model loaded." << std::endl;

    std::vector<std::string> imageNames;
    imageNames.push_back("yolo/upload/pictures/dagger_001.jpg");
    cout << "Added image path." << std::endl;
    imageNames.push_back("yolo/upload/pictures/explosive_001.jpg");
    cout << "Added image path." << std::endl;
    imageNames.push_back("yolo/upload/pictures/first_aid_kit_001.jpg");
    cout << "Added image path." << std::endl;
    // imageNames.push_back(projectBasePath + "/ultralytics/assets/bus.jpg");
    // imageNames.push_back(projectBasePath + "/ultralytics/assets/zidane.jpg");
    // imageNames.push_back(projectBasePath + "/ultralytics/assets/firearms_001.jpg");
    // imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/explosive_001.jpg");
    // imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/dagger_001.jpg");
    // imageNames.push_back("/home/fhfh/Work/LS2K0300CAR/yolo/first_aid_kit_001.jpg");
    
    for (int i = 0; i < imageNames.size(); ++i)
    {
        cv::Mat frame = cv::imread(imageNames[i]);

        cout << "read image" << std::endl;
        if (frame.empty()) {
            std::cerr << "Error: Could not load image" << std::endl;
            return 1;
        }

        // Inference starts here...
        std::vector<Detection> output = inf.runInference(frame);
        if (!output.empty()) {
            auto &r = output[0];
            std::string txt = r.className + " " + std::to_string(r.confidence).substr(0, 5);
            cv::putText(frame, txt, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
        }

        // int detections = output.size();
        // std::cout << "Number of detections:" << detections << std::endl;

        // for (int i = 0; i < detections; ++i)
        // {
        //     Detection detection = output[i];

        //     cv::Rect box = detection.box;
        //     cv::Scalar color = detection.color;

        //     // Detection box
        //     cv::rectangle(frame, box, color, 2);

        //     // Detection box text
        //     std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
        //     cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        //     cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

        //     cv::rectangle(frame, textBox, color, cv::FILLED);
        //     cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        // }
        // Inference ends here...

        // This is only for preview purposes
        float scale = 0.8;
        cv::resize(frame, frame, cv::Size(frame.cols*scale, frame.rows*scale));
        // cv::imshow("Inference", frame);
        // cv::waitKey(-1);
        std::string outputFileName = "output/output_" + std::to_string(i) + ".jpg";
        cv::imwrite(outputFileName, frame);
        std::cout << "Saved result to " << outputFileName << std::endl;
    }
}
