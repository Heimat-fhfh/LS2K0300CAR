// DualMotorController.cpp
#include "DualMotorController.h"

DualMotorController::DualMotorController() {
    // 使用示例中的设备路径
    leftMotor_ = std::make_unique<MotorController>(
        "/dev/zf_driver_gpio_motor_1",
        "/dev/zf_device_pwm_motor_1"
    );
    
    rightMotor_ = std::make_unique<MotorController>(
        "/dev/zf_driver_gpio_motor_2",
        "/dev/zf_device_pwm_motor_2"
    );
}

void DualMotorController::setSpeeds(float leftSpeed, float rightSpeed) {
    leftMotor_->setSpeed(leftSpeed);
    rightMotor_->setSpeed(rightSpeed);
}

void DualMotorController::stopAll() {
    leftMotor_->stop();
    rightMotor_->stop();
}

MotorController& DualMotorController::getLeftMotor() {
    return *leftMotor_;
}

MotorController& DualMotorController::getRightMotor() {
    return *rightMotor_;
}

void DualMotorController::setMaxDutyLimits(float maxDutyPercent) {
    leftMotor_->setMaxDutyLimit(maxDutyPercent);
    rightMotor_->setMaxDutyLimit(maxDutyPercent);
}

void DualMotorController::setPwmDeadZone(float deadZone) {
    leftMotor_->setPwmDeadZone(deadZone);
    rightMotor_->setPwmDeadZone(deadZone);
}

void DualMotorController::setPwmMaxValues(uint16_t maxValue) {
    leftMotor_->setPwmMaxValue(maxValue);
    rightMotor_->setPwmMaxValue(maxValue);
}