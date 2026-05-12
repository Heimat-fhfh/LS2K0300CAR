// MotorController.cpp
#include "MotorController.h"
#include "zf_common_headfile.h"
#include <cmath>
#include <thread>
#include <chrono>

MotorController::MotorController(const std::string& dirPath, const std::string& pwmPath)
    : dirPath_(dirPath)
    , pwmPath_(pwmPath)
    , currentSpeed_(0.0f)
    , maxDutyPercent_(100.0f)  // 默认最大100%占空比
    , pwmMaxValue_(10000)     // 默认PWM最大值
    , isRunning_(false)
    , currentDirection_(true) {  // 默认正转方向
}


MotorController::~MotorController() {
    stop();
}

void MotorController::setSpeed(float speed) {
    // 限制速度在-1.0到1.0之间
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    
    currentSpeed_ = speed;
    updateMotor(speed);
}

void MotorController::stop() {
    currentSpeed_ = 0.0f;
    pwm_set_duty(pwmPath_.c_str(), 0);
    isRunning_ = false;
}

float MotorController::getCurrentSpeed() const {
    return currentSpeed_;
}

void MotorController::setMaxDutyLimit(float maxDutyPercent) {
    if (maxDutyPercent > 100.0f) maxDutyPercent = 100.0f;
    if (maxDutyPercent < 0.0f) maxDutyPercent = 0.0f;
    maxDutyPercent_ = maxDutyPercent;
    
    // 如果当前正在运行，重新应用新的限制
    if (isRunning_) {
        updateMotor(currentSpeed_);
    }
}

void MotorController::setPwmMaxValue(uint16_t maxValue) {
    pwmMaxValue_ = maxValue;
    
    // 如果当前正在运行，重新应用新的PWM值
    if (isRunning_) {
        updateMotor(currentSpeed_);
    }
}

bool MotorController::isRunning() const {
    return isRunning_;
}

uint16_t MotorController::calculateDutyValue(float speed) const {
    // 计算绝对速度百分比，考虑最大占空比限制
    float absSpeed = std::abs(speed);
    float dutyPercent = absSpeed * maxDutyPercent_;
    
    // 计算PWM占空比值
    return static_cast<uint16_t>(dutyPercent / 100.0f * pwmMaxValue_);
}

void MotorController::updateMotor(float speed) {
    if (std::abs(speed) < 0.001f) {  // 接近0的速度视为停止
        stop();
        return;
    }
    
    // 计算目标方向
    bool targetDirection = (speed >= 0);
    
    // 方向切换保护：如果方向发生改变，先停止电机再切换方向
    // 避免在电机高速运转时直接切换方向导致电流冲击和机械应力
    if (isRunning_ && targetDirection != currentDirection_) {
        // 1. 先将PWM降到0，停止电机
        pwm_set_duty(pwmPath_.c_str(), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 等待电机停转
        
        // 2. 切换方向GPIO
        gpio_set_level(dirPath_.c_str(), targetDirection ? 1 : 0);
        currentDirection_ = targetDirection;
        
        // 3. 从较小的占空比开始重新加速
        uint16_t dutyValue = calculateDutyValue(speed);
        // 限制反向启动时的初始占空比，避免突然满转
        uint16_t maxStartDuty = static_cast<uint16_t>(0.3f * pwmMaxValue_);  // 最大30%起步
        if (dutyValue > maxStartDuty) {
            dutyValue = maxStartDuty;
        }
        pwm_set_duty(pwmPath_.c_str(), dutyValue);
    } else {
        // 正常情况：直接设置方向和PWM
        if (!isRunning_) {
            currentDirection_ = targetDirection;
            gpio_set_level(dirPath_.c_str(), targetDirection ? 1 : 0);
        }
        
        uint16_t dutyValue = calculateDutyValue(speed);
        pwm_set_duty(pwmPath_.c_str(), dutyValue);
    }
    
    isRunning_ = true;
}


