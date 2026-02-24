// DualMotorController.h
#ifndef DUAL_MOTOR_CONTROLLER_H
#define DUAL_MOTOR_CONTROLLER_H

#include "MotorController.h"
#include <memory>

class DualMotorController {
public:
    // 构造函数：创建两个电机控制器
    DualMotorController();
    
    // 设置两个电机的速度
    void setSpeeds(float leftSpeed, float rightSpeed);
    
    // 同时停止两个电机
    void stopAll();
    
    // 获取电机控制器引用
    MotorController& getLeftMotor();
    MotorController& getRightMotor();
    
    // 设置两个电机的最大占空比限制
    void setMaxDutyLimits(float maxDutyPercent);
    
    // 设置PWM最大值
    void setPwmMaxValues(uint16_t maxValue);

private:
    std::unique_ptr<MotorController> leftMotor_;
    std::unique_ptr<MotorController> rightMotor_;
};

#endif // DUAL_MOTOR_CONTROLLER_H