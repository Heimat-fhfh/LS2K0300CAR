#include "task_scheduler.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <iomanip>

// 模拟传感器和车辆组件
class Camera {
public:
    std::vector<uint8_t> captureImage() {
        // 模拟图像捕获
        std::vector<uint8_t> image(640 * 480 * 3);
        // 这里会填充一些随机数据
        return image;
    }
};

class ImageProcessor {
public:
    void processImage(const std::vector<uint8_t>& image) {
        // 模拟图像处理
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    std::vector<int> detectLanes(const std::vector<uint8_t>& image) {
        // 模拟车道线检测
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        return {1, 2, 3}; // 返回一些假数据
    }
};

class MotorController {
public:
    void setSpeed(double left, double right) {
        // 模拟电机控制
        current_left_ = left;
        current_right_ = right;
    }
    
    std::pair<double, double> getCurrentSpeed() {
        return {current_left_, current_right_};
    }
    
private:
    double current_left_ = 0.0;
    double current_right_ = 0.0;
};

class SensorHub {
public:
    double readUltrasonic() {
        // 模拟超声波读数
        return 100.0 * (rand() % 100) / 100.0;
    }
    
    std::vector<double> readIMU() {
        // 模拟IMU读数 (加速度计和陀螺仪)
        return {
            0.1 * (rand() % 20 - 10),
            0.1 * (rand() % 20 - 10),
            0.1 * (rand() % 20 - 10)
        };
    }
};

class Display {
public:
    void update(const std::string& data) {
        // 模拟显示更新
        std::cout << "\033[2J\033[1;1H"; // 清屏
        std::cout << "======== 智能小车状态 ========" << std::endl;
        std::cout << data << std::endl;
        std::cout << "=============================" << std::endl;
    }
};

class DataUploader {
public:
    void uploadData(const std::string& data) {
        // 模拟数据上传
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // std::cout << "数据已上传: " << data << std::endl;
    }
};

int main() {
    // 创建组件
    Camera camera;
    ImageProcessor imageProcessor;
    MotorController motorController;
    SensorHub sensorHub;
    Display display;
    DataUploader dataUploader;
    
    // 共享状态
    std::vector<uint8_t> latestImage;
    std::vector<int> laneInfo;
    double distanceReading = 0.0;
    std::vector<double> imuReadings;
    std::mutex stateMutex;
    
    // 创建任务调度器 (4个工作线程)
    robot::TaskScheduler scheduler(4);
    
    // 添加图像捕获任务 (30Hz)
    scheduler.addTask("图像捕获", [&]() {
        auto image = camera.captureImage();
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestImage = image;
        }
    }, 30, robot::Priority::HIGH, 30);
    
    // 添加图像处理任务 (20Hz)
    scheduler.addTask("图像处理", [&]() {
        std::vector<uint8_t> image;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            image = latestImage;
        }
        if (!image.empty()) {
            imageProcessor.processImage(image);
        }
    }, 20, robot::Priority::MEDIUM, 45);
    
    // 添加车道线检测任务 (15Hz)
    scheduler.addTask("车道检测", [&]() {
        std::vector<uint8_t> image;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            image = latestImage;
        }
        if (!image.empty()) {
            auto lanes = imageProcessor.detectLanes(image);
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                laneInfo = lanes;
            }
        }
    }, 15, robot::Priority::MEDIUM, 60);
    
    // 添加电机控制任务 (100Hz)
    scheduler.addTask("电机控制", [&]() {
        double distance;
        std::vector<int> lanes;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            distance = distanceReading;
            lanes = laneInfo;
        }
        
        // 简单的控制逻辑
        double leftSpeed = 50.0;
        double rightSpeed = 50.0;
        
        // 如果距离太近，减速
        if (distance < 50.0) {
            leftSpeed *= (distance / 100.0);
            rightSpeed *= (distance / 100.0);
        }
        
        // 根据车道线调整方向
        if (!lanes.empty()) {
            if (lanes[0] < 1) {
                leftSpeed *= 0.8;
            } else if (lanes[0] > 2) {
                rightSpeed *= 0.8;
            }
        }
        
        motorController.setSpeed(leftSpeed, rightSpeed);
    }, 100, robot::Priority::CRITICAL, 5);
    
    // 添加传感器读取任务 (50Hz)
    scheduler.addTask("传感器读取", [&]() {
        double distance = sensorHub.readUltrasonic();
        auto imu = sensorHub.readIMU();
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            distanceReading = distance;
            imuReadings = imu;
        }
    }, 50, robot::Priority::HIGH, 10);
    
    // 添加数据上传任务 (5Hz)
    scheduler.addTask("数据上传", [&]() {
        double distance;
        std::vector<double> imu;
        auto speeds = motorController.getCurrentSpeed();
        
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            distance = distanceReading;
            imu = imuReadings;
        }
        
        std::stringstream ss;
        ss << "距离: " << distance << "cm, ";
        ss << "IMU: [" << (imu.size() > 0 ? imu[0] : 0) << ", " 
                       << (imu.size() > 1 ? imu[1] : 0) << ", " 
                       << (imu.size() > 2 ? imu[2] : 0) << "], ";
        ss << "电机: [" << speeds.first << ", " << speeds.second << "]";
        
        dataUploader.uploadData(ss.str());
    }, 5, robot::Priority::LOW, 100);
    
    // 添加显示更新任务 (10Hz)
    scheduler.addTask("显示更新", [&]() {
        double distance;
        std::vector<double> imu;
        auto speeds = motorController.getCurrentSpeed();
        auto stats = scheduler.getTasksStats();
        
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            distance = distanceReading;
            imu = imuReadings;
        }
        
        std::stringstream ss;
        ss << "距离: " << std::fixed << std::setprecision(2) << distance << "cm\n";
        ss << "IMU: [" << std::setprecision(2) 
                      << (imu.size() > 0 ? imu[0] : 0) << ", " 
                      << (imu.size() > 1 ? imu[1] : 0) << ", " 
                      << (imu.size() > 2 ? imu[2] : 0) << "]\n";
        ss << "电机: [" << speeds.first << ", " << speeds.second << "]\n\n";
        ss << "任务统计:\n";
        
        for (const auto& task : stats) {
            ss << "- " << task.first << ": " 
               << task.second.at("frequency_actual_hz") << "Hz / " 
               << task.second.at("frequency_target_hz") << "Hz, 执行时间: " 
               << task.second.at("execution_time_us") / 1000.0 << "ms, 优先级: "
               << task.second.at("priority") << ", 超时: "
               << task.second.at("missed_deadlines") << "\n";
        }
        
        display.update(ss.str());
    }, 10, robot::Priority::BACKGROUND, 100);
    
    // 启动调度器
    scheduler.start();
    
    // 运行一段时间
    std::cout << "智能小车正在运行..." << std::endl;
    std::cout << "按Ctrl+C停止" << std::endl;
    
    // 主线程等待
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
