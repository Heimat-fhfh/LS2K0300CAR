// MotorController.cpp
#include "control/MotorController.h"
#include "drivers/zf_common_headfile.h"
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
    if (speed < 0.0f) speed = 0.0f; // 禁止反转
    
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
    float absSpeed = std::abs(speed);
    float dutyPercent = absSpeed * maxDutyPercent_;

    return static_cast<uint16_t>(dutyPercent / 100.0f * pwmMaxValue_);
}

void MotorController::updateMotor(float speed) {
    if (speed <= 0.0f) {
        stop();
        return;
    }
    
    gpio_set_level(dirPath_.c_str(), 1);
    
    uint16_t dutyValue = calculateDutyValue(speed);
    pwm_set_duty(pwmPath_.c_str(), dutyValue);
    
    isRunning_ = true;
}