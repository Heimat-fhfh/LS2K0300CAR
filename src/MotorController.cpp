// MotorController.cpp
#include "MotorController.h"
#include "zf_common_headfile.h"
#include <cmath>

MotorController::MotorController(const std::string& dirPath, const std::string& pwmPath)
    : dirPath_(dirPath)
    , pwmPath_(pwmPath)
    , currentSpeed_(0.0f)
    , maxDutyPercent_(100.0f)  // 默认最大100%占空比
    , pwmMaxValue_(10000)     // 默认PWM最大值
    , isRunning_(false) {
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
    
    // 设置方向
    bool direction = (speed >= 0);
    gpio_set_level(dirPath_.c_str(), direction ? 1 : 0);
    
    // 计算并设置PWM占空比
    uint16_t dutyValue = calculateDutyValue(speed);
    pwm_set_duty(pwmPath_.c_str(), dutyValue);
    
    isRunning_ = true;
}