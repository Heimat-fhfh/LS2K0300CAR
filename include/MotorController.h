// MotorController.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string>
#include <functional>
#include "zf_common_typedef.h"


class MotorController {
public:
    // 构造函数：需要传入方向控制GPIO路径和PWM设备路径
    MotorController(const std::string& dirPath, const std::string& pwmPath);
    
    // 析构函数：自动停止电机
    ~MotorController();
    
    // 设置电机速度：范围-1.0到1.0，负值表示反转
    void setSpeed(float speed);
    
    // 停止电机
    void stop();
    
    // 获取当前速度
    float getCurrentSpeed() const;
    
    // 设置最大占空比限制（0-100%）
    void setMaxDutyLimit(float maxDutyPercent);
    
    // 设置PWM设备的最大值（默认为10000）
    void setPwmMaxValue(uint16_t maxValue);
    
    // 获取电机状态
    bool isRunning() const;

private:
    // 计算实际的PWM占空比值
    uint16_t calculateDutyValue(float speed) const;
    
    // 更新电机状态
    void updateMotor(float speed);
    
private:
    std::string dirPath_;      // 方向控制设备路径
    std::string pwmPath_;      // PWM设备路径
    float currentSpeed_;       // 当前速度（-1.0到1.0）
    float maxDutyPercent_;     // 最大占空比限制（0-100%）
    uint16_t pwmMaxValue_;     // PWM设备的最大值
    bool isRunning_;           // 电机是否正在运行
};

#endif // MOTOR_CONTROLLER_H