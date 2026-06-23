#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <array>
#include <cstdio>

#include "devices/encoder.hpp"
#include "control/DualMotorController.h"
#include "control/PID.hpp"
#include "drivers/zf_driver_udp.hpp"
#include "devices/zf_device_imu_core.h"
#include "drivers/zf_driver_gpio.h"

class MotorControlTask {
public:
    MotorControlTask(
        const Control::PID::Parameters& diffOuterParams,
        const Control::PID::Parameters& diffInnerParams,
        const Control::IncrementalPID::Parameters& speedIncrParams,
        DualMotorController* motors,
        Encoder* leftEncoder,
        Encoder* rightEncoder,
        IMUDevice* imu,
        double controlPeriod = 0.01
    );

    ~MotorControlTask();

    MotorControlTask(const MotorControlTask&) = delete;
    MotorControlTask& operator=(const MotorControlTask&) = delete;
    MotorControlTask(MotorControlTask&&) = default;
    MotorControlTask& operator=(MotorControlTask&&) = default;

    bool start();
    void stop();
    bool isRunning() const;
    bool hasError() const;
    void setRealtimePriority(int priority = 50, int policy = SCHED_FIFO);

    // ==================== 串级差速环 API ====================

    void setSteerError(double error);
    void setTargetSpeed(double speed);
    double getTargetSpeed() const;

    void setDiffOuterParams(const Control::PID::Parameters& params);
    void setDiffInnerParams(const Control::PID::Parameters& params);
    void setSpeedIncrementalParams(const Control::IncrementalPID::Parameters& params);
    void setMotorMaxDuty(double duty);
    void setCurvatureSpeedGain(double gain);
    double getCurvatureSpeedGain() const;
    void setCurvatureSpeedMin(double minSpeed);
    double getCurvatureSpeedMin() const;

    // ==================== 碰撞保护 API ====================

    void enableCollisionProtection(bool enable = true);
    bool isCollisionProtectionEnabled() const;
    void setCollisionImuJerkThreshold(double threshold);
    void setCollisionStallThresholds(double dutyThreshold, double speedThreshold, int cycles);
    void setCollisionKeyConfig(int resetKeyIndex, int bumperKeyIndex);
    void configureCollisionGpio(const std::array<std::string, 4>& keyPaths);
    void resetCollisionState();
    bool isCollisionDetected() const;

    // ==================== 低通滤波 API ====================

    void enableLowPassFilter(bool enable = true);
    bool isLowPassFilterEnabled() const;
    void setSpeedFilterTimeConstant(double tau);
    void setAngularFilterTimeConstant(double tau);
    void setSteerErrorFilterTimeConstant(double tau);
    void resetFilters(double leftValue = 0.0, double rightValue = 0.0, double angularValue = 0.0);

    // ==================== 斜坡控制 API ====================

    void enableRampControl(bool enable = true);
    bool isRampControlEnabled() const;
    void setRampRates(double accelRate, double decelRate);
    void resetRampState();

    // ==================== 外环PD输出斜坡控制 API ====================

    void enableDiffOutputRamp(bool enable = true);
    bool isDiffOutputRampEnabled() const;
    void setDiffOutputRampRates(double accelRate, double decelRate);

    // ==================== 紧急停机/出界保护 API ====================

    void emergencyStop();               // 紧急停机：电机输出立即归零
    void clearEmergencyStop();          // 清除紧急停机并复位所有PID积分
    bool isEmergencyStopActive() const;

private:
    void run();

    bool isValidSpeed(double speed) const;
    bool isValidAngularVelocity(double angularVelocity) const;

    // ==================== 碰撞检测方法 ====================

    bool detectImuCollision(const imu_unit_data_t& imuData) const;
    bool detectStallCollision(double leftOutput, double rightOutput,
                              double leftSpeed, double rightSpeed);
    bool detectGpioCollision() const;
    void handleCollision();
    bool checkCollisionReset() const;
    const char* getKeyPath(int keyIndex) const;

private:
    class LowPassFilter {
    public:
        LowPassFilter(double tau = 0.02, double dt = 0.01);
        double apply(double raw);
        void reset(double value = 0.0);
        void setTimeConstant(double tau);
        double getTimeConstant() const { return tau_; }
        double getAlpha() const { return alpha_; }
    private:
        double tau_;
        double dt_;
        double alpha_;
        double lastValue_;
        bool initialized_;
    };

    class RampLimiter {
    public:
        RampLimiter(double accelRate = 50.0, double decelRate = 100.0, double dt = 0.01);
        double apply(double target);
        void reset(double value = 0.0);
        void setAccelRate(double rate);
        void setDecelRate(double rate);
        double getAccelRate() const { return accelRate_; }
        double getDecelRate() const { return decelRate_; }
    private:
        double accelRate_;
        double decelRate_;
        double dt_;
        double lastValue_;
        bool initialized_;
    };

private:
    // 原子变量
    std::atomic<double> steerError;       // 归一化偏差 [-1, 1]
    std::atomic<double> targetSpeed;      // 期望速度 (m/s)

    // PID参数
    const Control::PID::Parameters diffOuterParams;   // 外环差速PD
    const Control::PID::Parameters diffInnerParams;   // 内环角速度PI
    const Control::IncrementalPID::Parameters speedIncrParams; // 速度环增量PID

    // 外部对象指针
    DualMotorController* motors;
    Encoder* leftEncoder;
    Encoder* rightEncoder;
    IMUDevice* imu;

    double motorMaxDuty;

    // 碰撞保护
    std::atomic<bool> collisionProtectEnabled;
    std::atomic<double> collisionImuJerkThreshold;
    std::atomic<double> collisionStallDutyThreshold;
    std::atomic<double> collisionStallSpeedThreshold;
    std::atomic<int> collisionStallCycles;
    std::atomic<int> collisionResetKey;
    std::atomic<int> collisionBumperKey;
    std::atomic<bool> collisionDetected;
    std::array<std::string, 4> collisionKeyPaths_;
    int stallCounter_{0};

    // 低通滤波
    std::atomic<bool> lowPassFilterEnabled;
    std::atomic<double> speedFilterTau;
    std::atomic<double> angularFilterTau;
    std::atomic<double> steerFilterTau;
    LowPassFilter leftSpeedFilter;
    LowPassFilter rightSpeedFilter;
    LowPassFilter angularVelocityFilter;
    LowPassFilter steerErrorFilter;

    // 斜坡控制
    std::atomic<bool> rampControlEnabled;
    RampLimiter leftRampLimiter;
    RampLimiter rightRampLimiter;

    // 外环PD输出斜坡控制
    std::atomic<bool> diffOutputRampEnabled;
    RampLimiter diffOutputRampLimiter;

    // 曲率自适应降速
    std::atomic<double> curvatureSpeedGain{0.0};
    std::atomic<double> curvatureSpeedMin{0.1};

    // 紧急停机/出界保护
    std::atomic<bool> emergencyStopActive{false};
    std::atomic<bool> pidResetRequested{false};

    // 控制周期
    const double controlPeriod;

    // 线程控制
    std::atomic<bool> running;
    std::atomic<bool> taskError;
    std::unique_ptr<std::thread> workerThread;

    int rtPriority;
    int rtPolicy;
};
