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
    , pwmDeadZone_(0.001f)   // 默认PWM死区0.001（与原硬编码值一致）
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

void MotorController::setPwmDeadZone(float deadZone) {
    if (deadZone < 0.0f) deadZone = 0.0f;
    if (deadZone >= 1.0f) deadZone = 0.999f;
    pwmDeadZone_ = deadZone;
    
    if (isRunning_) {
        updateMotor(currentSpeed_);
    }
}

float MotorController::getPwmDeadZone() const {
    return pwmDeadZone_;
}

bool MotorController::isRunning() const {
    return isRunning_;
}

float MotorController::applyDeadZoneRemap(float speed) const {
    float absSpeed = std::abs(speed);
    if (absSpeed <= pwmDeadZone_) {
        return 0.0f;
    }
    // 剔除死区并重映射：将 [deadZone, 1.0] → [0.0, 1.0]
    float remapped = (absSpeed - pwmDeadZone_) / (1.0f - pwmDeadZone_);
    return std::copysign(remapped, speed);
}

uint16_t MotorController::calculateDutyValue(float speed) const {
    // 计算绝对速度百分比，考虑最大占空比限制
    float absSpeed = std::abs(speed);
    float dutyPercent = absSpeed * maxDutyPercent_;
    
    // 计算PWM占空比值
    return static_cast<uint16_t>(dutyPercent / 100.0f * pwmMaxValue_);
}

void MotorController::updateMotor(float speed) {
    // 应用PWM死区重映射
    float effectiveSpeed = applyDeadZoneRemap(speed);
    if (effectiveSpeed == 0.0f) {
        stop();
        return;
    }
    
    // 设置方向
    bool direction = (effectiveSpeed >= 0);
    gpio_set_level(dirPath_.c_str(), direction ? 1 : 0);
    
    // 计算并设置PWM占空比（使用重映射后的有效速度）
    uint16_t dutyValue = calculateDutyValue(effectiveSpeed);
    pwm_set_duty(pwmPath_.c_str(), dutyValue);
    
    isRunning_ = true;
}