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
#include "zf_driver_udp.hpp"
#include "zf_device_imu_core.h"

class MotorControlTask {
public:
    /**
     * @brief 构造函数（不带IMU）
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
    
    /**
     * @brief 构造函数（带IMU）
     * @param leftParams 左轮PID参数
     * @param rightParams 右轮PID参数
     * @param motors 电机控制对象指针
     * @param leftEncoder 左编码器指针
     * @param rightEncoder 右编码器指针
     * @param imu IMU设备指针（用于角速度控制）
     * @param controlPeriod 控制周期（秒）
     */
    MotorControlTask(
        const Control::PID::Parameters& leftParams,
        const Control::PID::Parameters& rightParams,
        DualMotorController* motors,
        Encoder* leftEncoder,
        Encoder* rightEncoder,
        IMUDevice* imu,
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
    
    // ==================== 角速度控制相关方法 ====================
    
    /**
     * @brief 设置目标角速度
     * @param angularVelocity 目标角速度（°/s）
     */
    void setTargetAngularVelocity(double angularVelocity);
    
    /**
     * @brief 获取当前目标角速度
     * @return 当前目标角速度（°/s）
     */
    double getTargetAngularVelocity() const;
    
    /**
     * @brief 启用角速度控制
     * @param enable 是否启用角速度控制
     */
    void enableAngularVelocityControl(bool enable = true);
    
    /**
     * @brief 检查角速度控制是否启用
     * @return 角速度控制是否启用
     */
    bool isAngularVelocityControlEnabled() const;
    
    /**
     * @brief 设置基础线速度（角速度控制的基础速度）
     * @param baseSpeed 基础线速度（m/s）
     */
    void setBaseSpeed(double baseSpeed);
    
    /**
     * @brief 获取当前基础线速度
     * @return 当前基础线速度（m/s）
     */
    double getBaseSpeed() const;
    
    /**
     * @brief 设置轮距（用于角速度计算）
     * @param wheelbase 轮距（米）
     */
    void setWheelbase(double wheelbase);
    
    /**
     * @brief 设置车轮半径（用于速度计算）
     * @param wheelRadius 车轮半径（米）
     */
    void setWheelRadius(double wheelRadius);

private:
    void run();
    
    // 速度有效性检查
    bool isValidSpeed(double speed) const;
    
    // 角速度有效性检查
    bool isValidAngularVelocity(double angularVelocity) const;
    
    // 运动学分解：将基础速度和角速度转换为左右轮速度
    std::pair<double, double> kinematicsDecomposition(double baseSpeed, double angularVelocityRad) const;
    
    // 单位转换：度/秒 转 弧度/秒
    double degToRad(double deg) const;
    
    // 单位转换：弧度/秒 转 度/秒
    double radToDeg(double rad) const;

private:
    // 原子变量存储目标速度（左右轮独立）
    std::atomic<double> leftTargetSpeed;
    std::atomic<double> rightTargetSpeed;
    
    // 角速度控制相关原子变量
    std::atomic<double> targetAngularVelocity;      // 目标角速度（°/s）
    std::atomic<double> baseSpeed;                  // 基础线速度（m/s）
    std::atomic<bool> angularVelocityControlEnabled; // 角速度控制是否启用
    
    // PID参数（非原子，初始化后不变）
    const Control::PID::Parameters leftParams;
    const Control::PID::Parameters rightParams;
    
    // 角速度PID参数
    Control::PID::Parameters angularVelocityParams;
    
    // 外部对象指针
    DualMotorController* motors;
    Encoder* leftEncoder;
    Encoder* rightEncoder;
    IMUDevice* imu;                                 // IMU设备指针（可为空）
    
    // 车辆参数
    double wheelbase;                               // 轮距（米）
    double wheelRadius;                             // 车轮半径（米）
    
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
