#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <array>
#include <cstdio>

#include "encoder.hpp"
#include "DualMotorController.h"
#include "PID.hpp"

// UDP发送函数的声明（假设已有）
void send_udp_data(const char* topic, const float* data, size_t size);

class MotorControlTask {
public:
    /**
     * @brief 构造函数
     * @param leftParams 左轮PID参数
     * @param rightParams 右轮PID参数
     * @param motors 电机控制对象指针
     * @param leftEncoder 左编码器指针
     * @param rightEncoder 右编码器指针
     * @param controlPeriod 控制周期（秒）
     */
    MotorControlTask(
        const Control::PID::Parameters& leftParams,
        const Control::PID::Parameters& rightParams,
        DualMotorController* motors,
        Encoder* leftEncoder,
        Encoder* rightEncoder,
        double controlPeriod = 0.01
    );
    
    ~MotorControlTask();
    
    // 禁止拷贝
    MotorControlTask(const MotorControlTask&) = delete;
    MotorControlTask& operator=(const MotorControlTask&) = delete;
    
    // 允许移动
    MotorControlTask(MotorControlTask&&) = default;
    MotorControlTask& operator=(MotorControlTask&&) = default;
    
    // 启动任务
    bool start();
    
    // 停止任务
    void stop();
    
    // 设置目标速度（左右轮独立）
    void setTargetSpeed(double leftSpeed, double rightSpeed);
    
    // 设置左轮目标速度
    void setLeftTargetSpeed(double speed);
    
    // 设置右轮目标速度
    void setRightTargetSpeed(double speed);
    
    // 获取当前左轮目标速度
    double getLeftTargetSpeed() const;
    
    // 获取当前右轮目标速度
    double getRightTargetSpeed() const;
    
    // 获取左右轮目标速度
    std::pair<double, double> getTargetSpeeds() const;
    
    // 检查任务是否在运行
    bool isRunning() const;
    
    // 检查是否有错误
    bool hasError() const;
    
    // 设置实时优先级（可选）
    void setRealtimePriority(int priority = 50, int policy = SCHED_FIFO);

private:
    void run();
    
    // 速度有效性检查
    bool isValidSpeed(double speed) const;
    
private:
    // 原子变量存储目标速度（左右轮独立）
    std::atomic<double> leftTargetSpeed;
    std::atomic<double> rightTargetSpeed;
    
    // PID参数（非原子，初始化后不变）
    const Control::PID::Parameters leftParams;
    const Control::PID::Parameters rightParams;
    
    // 外部对象指针
    DualMotorController* motors;
    Encoder* leftEncoder;
    Encoder* rightEncoder;
    
    // 控制周期
    const double controlPeriod;
    
    // 线程控制
    std::atomic<bool> running;
    std::atomic<bool> taskError;
    std::unique_ptr<std::thread> workerThread;
    
    // 实时优先级设置
    int rtPriority;
    int rtPolicy;
};