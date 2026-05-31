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
     * @brief 设置速度PID的积分项
     * @param integral 积分项值
     */
    void setSpeedPidIntegral(double integral);
    
    /**
     * @brief 获取当前基础线速度
     * @return 当前基础线速度（m/s）
     */
    double getBaseSpeed() const;
    
    /**
     * @brief 获取最近一次实际角速度（来自IMU）
     * @return 实际角速度（°/s）
     */
    double getActualAngularVelocity() const;
    
    /**
     * @brief 获取角速度PID控制输出量
     * @return 角速度PID输出（°/s）
     */
    double getAngularVelocityPidOutput() const;
    
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
    
    /**
     * @brief 设置电机最大占空比限制
     * @param duty 最大占空比（百分比）
     */
    void setMotorMaxDuty(double duty);
    
    /**
     * @brief 设置角速度PID参数
     * @param params 角速度PID参数
     */
    void setAngularVelocityParams(const Control::PID::Parameters& params);

    // ==================== 低通滤波控制相关方法 ====================
    
    /**
     * @brief 启用或禁用低通滤波
     * @param enable 是否启用
     */
    void enableLowPassFilter(bool enable = true);
    
    /**
     * @brief 检查低通滤波是否启用
     * @return 低通滤波是否启用
     */
    bool isLowPassFilterEnabled() const;
    
    /**
     * @brief 设置轮速滤波器时间常数
     * @param tau 时间常数（秒），τ = dt/(1-α)*dt，α = dt/(dt+τ)
     */
    void setSpeedFilterTimeConstant(double tau);
    
    /**
     * @brief 设置角速度滤波器时间常数
     * @param tau 时间常数（秒）
     */
    void setAngularFilterTimeConstant(double tau);
    
    /**
     * @brief 重置滤波器状态
     * @param leftValue 左轮初始值
     * @param rightValue 右轮初始值
     * @param angularValue 角速度初始值
     */
    void resetFilters(double leftValue = 0.0, double rightValue = 0.0, double angularValue = 0.0);

    // ==================== 斜坡控制相关方法 ====================
    
    /**
     * @brief 启用或禁用斜坡限制
     * @param enable 是否启用斜坡限制
     */
    void enableRampLimiting(bool enable = true);
    
    /**
     * @brief 检查斜坡限制是否启用
     * @return 斜坡限制是否启用
     */
    bool isRampLimitingEnabled() const;
    
    /**
     * @brief 设置斜坡限制参数
     * @param maxAcceleration 最大加速度（占空比/秒）
     * @param maxDeceleration 最大减速度（占空比/秒）
     */
    void setRampLimits(double maxAcceleration, double maxDeceleration);
    
    /**
     * @brief 获取当前斜坡限制参数
     * @return 当前最大加速度和减速度（占空比/秒）
     */
    std::pair<double, double> getRampLimits() const;
    
    /**
     * @brief 重置斜坡限制器的当前值
     * @param leftValue 左轮重置值
     * @param rightValue 右轮重置值
     */
    void resetRampLimiters(double leftValue = 0.0, double rightValue = 0.0);

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
    // 一阶低通滤波器类
    class LowPassFilter {
    public:
        LowPassFilter(double tau = 0.02, double dt = 0.01);
        
        double apply(double raw);
        void reset(double value = 0.0);
        void setTimeConstant(double tau);
        double getTimeConstant() const { return tau_; }
        double getAlpha() const { return alpha_; }
        
    private:
        double tau_;         // 时间常数（秒）
        double dt_;          // 采样周期（秒）
        double alpha_;       // 滤波系数 α = dt / (dt + τ)
        double lastValue_;   // 上一次滤波输出
        bool initialized_;   // 是否已初始化
    };

    // 斜坡限制器类
    class RampLimiter {
    public:
        RampLimiter(double maxAcceleration = 0.5, double maxDeceleration = 0.5);
        
        // 应用斜坡限制
        double apply(double target, double current, double dt);
        
        // 重置当前值
        void reset(double value);
        
        // 设置限制参数
        void setLimits(double maxAcceleration, double maxDeceleration);
        
        // 获取当前限制参数
        std::pair<double, double> getLimits() const;
        
    private:
        double maxAcceleration_;    // 最大加速度（占空比/秒）
        double maxDeceleration_;    // 最大减速度（占空比/秒）
    };

private:
    // 原子变量存储目标速度（左右轮独立）
    std::atomic<double> leftTargetSpeed;
    std::atomic<double> rightTargetSpeed;
    
    // 角速度控制相关原子变量
    std::atomic<double> targetAngularVelocity;      // 目标角速度（°/s）
    std::atomic<double> baseSpeed;                  // 基础线速度（m/s）
    std::atomic<double> lastActualAngularVelocity;   // 最近一次实际角速度（°/s）
    std::atomic<double> lastAngularVelocityPidOutput;// 角速度PID控制器输出量（°/s）
    std::atomic<bool> angularVelocityControlEnabled; // 角速度控制是否启用
    std::atomic<bool> speedPidIntegralResetRequested{false};
    std::atomic<double> speedPidIntegralValue{0.0};
    
    // 低通滤波相关原子变量
    std::atomic<bool> lowPassFilterEnabled;         // 低通滤波是否启用
    std::atomic<double> speedFilterTau;              // 轮速滤波器时间常数
    std::atomic<double> angularFilterTau;            // 角速度滤波器时间常数
    
    // 斜坡控制相关原子变量
    std::atomic<bool> rampLimitingEnabled;          // 斜坡限制是否启用
    
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
    double motorMaxDuty;                            // 电机最大占空比（百分比）
    
    // 控制周期
    const double controlPeriod;
    
    // 线程控制
    std::atomic<bool> running;
    std::atomic<bool> taskError;
    std::unique_ptr<std::thread> workerThread;
    
    // 实时优先级设置
    int rtPriority;
    int rtPolicy;
    
    // 一阶低通滤波器
    LowPassFilter leftSpeedFilter;
    LowPassFilter rightSpeedFilter;
    LowPassFilter angularVelocityFilter;
    
    // 斜坡限制器
    RampLimiter leftRampLimiter;
    RampLimiter rightRampLimiter;
    
    // 上一次的输出值（用于斜坡计算）
    double lastLeftOutput_;
    double lastRightOutput_;
};
