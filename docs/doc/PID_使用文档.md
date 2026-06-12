# PID 控制器库使用文档

## 概述

本PID控制器库是一个C++实现的数字PID控制器，支持比例(P)、积分(I)、微分(D)控制，带有完善的限幅功能和抗积分饱和机制。库提供了基础PID控制器以及两个扩展版本：带死区处理的PID控制器和带前馈控制的PID控制器。

## 目录

1. [特性](#特性)
2. [类结构](#类结构)
3. [快速开始](#快速开始)
4. [详细使用说明](#详细使用说明)
5. [参数调优指南](#参数调优指南)
6. [示例代码](#示例代码)
7. [常见问题](#常见问题)

## 特性

### 核心特性
- **完整的PID控制**：支持比例、积分、微分三项控制
- **自动时间计算**：自动计算时间间隔或手动指定时间间隔
- **抗积分饱和**：防止积分项过度累积导致系统不稳定
- **多重限幅**：支持比例项、积分项、微分项和总输出的独立限幅
- **积分下限限制**：可选设置积分项的最小值
- **状态管理**：支持重置、积分值设置、状态查询

### 扩展特性
- **带死区处理的PID**：在误差小于死区时不进行控制，减少系统抖动
- **带前馈控制的PID**：结合前馈控制提高系统响应速度

### 技术特性
- **类型安全**：使用C++11/14特性，类型安全
- **易于集成**：简单的API设计，易于集成到现有项目
- **高性能**：优化的计算逻辑，适合实时控制系统

## 类结构

### 主要类

#### 1. `Control::PID` - 基础PID控制器
```cpp
namespace Control {
    class PID {
    public:
        struct Parameters {
            double Kp{0.0};           // 比例系数
            double Ki{0.0};           // 积分系数
            double Kd{0.0};           // 微分系数
            
            double limitP{100.0};     // 比例项限幅
            double limitI{100.0};     // 积分项限幅
            double limitD{100.0};     // 微分项限幅
            double limitOutput{100.0}; // 输出总限幅
            
            double limitIMin{0.0};    // 积分下限（用于抗积分饱和）
            bool enableAntiWindup{true}; // 使能抗积分饱和
        };
        
        // 构造函数
        PID();
        explicit PID(const Parameters& params);
        
        // 核心方法
        double calculate(double setpoint, double feedback);
        double calculate(double setpoint, double feedback, double dt);
        void reset();
        void setParameters(const Parameters& params);
        
        // 状态查询
        const Parameters& getParameters() const;
        double getError() const;
        double getIntegral() const;
        void setIntegral(double integral);
        bool isInitialized() const;
    };
}
```

#### 2. `Control::PIDWithDeadband` - 带死区处理的PID控制器
```cpp
class PIDWithDeadband : public PID {
public:
    PIDWithDeadband(const Parameters& params, double deadband);
    double calculate(double setpoint, double feedback) override;
    double calculate(double setpoint, double feedback, double dt) override;
};
```

#### 3. `Control::PIDWithFeedforward` - 带前馈控制的PID控制器
```cpp
class PIDWithFeedforward : public PID {
public:
    PIDWithFeedforward(const Parameters& params, double ffGain = 1.0);
    double calculate(double setpoint, double feedback) override;
    double calculate(double setpoint, double feedback, double dt) override;
    void setFeedforwardGain(double gain);
};
```

## 快速开始

### 1. 包含头文件
```cpp
#include "PID.hpp"
using namespace Control;
```

### 2. 创建PID控制器
```cpp
// 方法1：使用默认参数创建，稍后设置参数
PID pid;

// 方法2：创建时指定参数
PID::Parameters params;
params.Kp = 1.0;
params.Ki = 0.1;
params.Kd = 0.05;
params.limitOutput = 100.0;

PID pid(params);
```

### 3. 使用PID控制器
```cpp
double setpoint = 100.0;    // 目标值
double feedback = 0.0;      // 当前测量值
double output;

// 在控制循环中调用
while (true) {
    // 获取当前反馈值（例如从传感器读取）
    feedback = readSensor();
    
    // 计算控制输出
    output = pid.calculate(setpoint, feedback);
    
    // 应用控制输出（例如设置电机PWM）
    applyControl(output);
    
    // 等待下一个控制周期
    delay(controlPeriod);
}
```

## 详细使用说明

### 参数配置

#### PID参数结构体
```cpp
PID::Parameters params;

// 基本PID参数
params.Kp = 1.0;    // 比例系数 - 影响系统响应速度
params.Ki = 0.1;    // 积分系数 - 消除稳态误差
params.Kd = 0.05;   // 微分系数 - 抑制超调和振荡

// 限幅参数
params.limitP = 50.0;       // 比例项最大输出
params.limitI = 30.0;       // 积分项最大输出  
params.limitD = 20.0;       // 微分项最大输出
params.limitOutput = 100.0; // 总输出最大限制

// 高级参数
params.limitIMin = 0.0;     // 积分项最小值（0表示不限制）
params.enableAntiWindup = true; // 启用抗积分饱和
```

#### 参数设置方法
```cpp
// 创建后设置参数
PID pid;
PID::Parameters params;
// ... 配置params ...
pid.setParameters(params);

// 运行时动态调整参数
pid.setParameters(newParams);
```

### 控制计算

#### 自动时间计算模式
```cpp
// 自动计算时间间隔（推荐用于大多数应用）
double output = pid.calculate(setpoint, feedback);
```

#### 手动时间计算模式
```cpp
// 手动指定时间间隔（适用于固定采样率的系统）
double dt = 0.01; // 10ms采样周期
double output = pid.calculate(setpoint, feedback, dt);
```

### 状态管理

#### 重置控制器
```cpp
// 重置所有内部状态（误差、积分值等）
pid.reset();
```

#### 查询状态
```cpp
// 获取当前误差
double currentError = pid.getError();

// 获取当前积分值
double currentIntegral = pid.getIntegral();

// 设置积分值（用于特殊场景）
pid.setIntegral(desiredIntegral);

// 检查控制器是否已初始化
bool initialized = pid.isInitialized();
```

### 扩展控制器使用

#### 带死区处理的PID控制器
```cpp
// 创建带死区的PID控制器
PID::Parameters params;
// ... 配置参数 ...
double deadband = 2.0; // 死区大小
PIDWithDeadband pidDeadband(params, deadband);

// 使用方式与基础PID相同
// 当 |setpoint - feedback| < deadband 时，输出为0
```

#### 带前馈控制的PID控制器
```cpp
// 创建带前馈的PID控制器
PID::Parameters params;
// ... 配置参数 ...
double ffGain = 0.5; // 前馈增益
PIDWithFeedforward pidFF(params, ffGain);

// 运行时调整前馈增益
pidFF.setFeedforwardGain(0.8);

// 使用方式与基础PID相同
// 输出 = PID输出 + ffGain * setpoint
```

## 参数调优指南

### 调优步骤

#### 步骤1：仅使用比例控制（P）
1. 设置 `Ki = 0`, `Kd = 0`
2. 逐渐增加 `Kp` 直到系统开始振荡
3. 将 `Kp` 设置为振荡临界值的50-60%

#### 步骤2：加入积分控制（I）
1. 保持上一步的 `Kp`
2. 逐渐增加 `Ki` 直到稳态误差被消除
3. 注意观察是否出现超调或振荡

#### 步骤3：加入微分控制（D）
1. 保持上一步的 `Kp` 和 `Ki`
2. 逐渐增加 `Kd` 以减少超调和振荡
3. 注意微分项可能放大噪声

### 参数初始值建议

| 系统类型 | Kp | Ki | Kd | 备注 |
|---------|----|----|----|------|
| 慢响应系统 | 0.5-2.0 | 0.01-0.1 | 0.0-0.05 | 温度控制、液位控制 |
| 中速响应系统 | 1.0-5.0 | 0.1-0.5 | 0.05-0.2 | 电机速度控制 |
| 快速响应系统 | 5.0-20.0 | 0.5-2.0 | 0.1-0.5 | 位置伺服、无人机姿态 |

### 限幅设置建议

1. **比例项限幅**：通常设置为总输出限幅的50-80%
2. **积分项限幅**：设置为总输出限幅的30-50%
3. **微分项限幅**：设置为总输出限幅的20-40%
4. **总输出限幅**：根据执行器能力设置（如PWM最大值）

## 示例代码

### 示例1：电机速度控制
```cpp
#include "PID.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace Control;

class MotorController {
private:
    PID speedPID;
    double currentSpeed;
    
public:
    MotorController() {
        // 配置PID参数
        PID::Parameters params;
        params.Kp = 2.5;
        params.Ki = 0.8;
        params.Kd = 0.15;
        params.limitOutput = 100.0; // PWM占空比限制
        params.limitI = 30.0;
        params.enableAntiWindup = true;
        
        speedPID.setParameters(params);
    }
    
    void controlLoop(double targetSpeed) {
        // 模拟控制周期
        const double controlPeriod = 0.01; // 10ms
        
        while (true) {
            // 读取当前速度（模拟）
            currentSpeed = readMotorSpeed();
            
            // 计算控制输出
            double pwmDuty = speedPID.calculate(targetSpeed, currentSpeed, controlPeriod);
            
            // 应用控制输出
            setMotorPWM(pwmDuty);
            
            // 打印状态
            std::cout << "Target: " << targetSpeed 
                      << ", Current: " << currentSpeed
                      << ", PWM: " << pwmDuty 
                      << ", Error: " << speedPID.getError() << std::endl;
            
            // 等待下一个控制周期
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
private:
    double readMotorSpeed() {
        // 模拟读取电机速度
        // 实际应用中应读取编码器数据
        return currentSpeed + (rand() % 10 - 5) * 0.1; // 添加噪声
    }
    
    void setMotorPWM(double duty) {
        // 模拟设置PWM
        // 实际应用中应设置硬件PWM
        if (duty > 0) {
            currentSpeed += duty * 0.1; // 简化模型
        }
    }
};
```

### 示例2：温度控制系统
```cpp
#include "PID.hpp"
#include <iostream>

using namespace Control;

class TemperatureController {
private:
    PIDWithDeadband tempPID;
    double currentTemp;
    
public:
    TemperatureController() : tempPID(createParams(), 0.5) {
        // 死区设置为0.5°C，减少加热器频繁开关
    }
    
    PID::Parameters createParams() {
        PID::Parameters params;
        params.Kp = 3.0;    // 温度响应较慢，需要较大的P
        params.Ki = 0.05;   // 较小的I，避免积分饱和
        params.Kd = 0.5;    // 适当的D抑制超调
        params.limitOutput = 100.0; // 加热器功率限制
        params.limitI = 20.0;
        params.enableAntiWindup = true;
        return params;
    }
    
    double controlStep(double targetTemp, double measuredTemp) {
        // 使用固定采样时间
        const double dt = 1.0; // 1秒采样周期
        
        return tempPID.calculate(targetTemp, measuredTemp, dt);
    }
};
```

### 示例3：机器人位置控制（带前馈）
```cpp
#include "PID.hpp"
#include <iostream>

using namespace Control;

class PositionController {
private:
    PIDWithFeedforward posPID;
    double currentPosition;
    
public:
    PositionController() : posPID(createParams(), 0.3) {
        // 前馈增益0.3，提高对目标位置变化的响应
    }
    
    PID::Parameters createParams() {
        PID::Parameters params;
        params.Kp = 8.0;    // 位置控制需要快速响应
        params.Ki = 0.5;    // 消除位置误差
        params.Kd = 1.2;    // 抑制超调和振荡
        params.limitOutput = 100.0; // 电机扭矩限制
        params.limitP = 60.0;
        params.limitI = 40.0;
        params.limitD = 30.0;
        params.enableAntiWindup = true;
        return params;
    }
    
    void setPosition(double targetPos) {
        // 重置控制器状态
        posPID.reset();
        
        while (abs(currentPosition - targetPos) > 0.01) {
            // 读取当前位置
            currentPosition = readEncoder();
            
            // 计算控制输出（带前馈）
            double torque = posPID.calculate(targetPos, currentPosition);
            
            // 应用扭矩
            applyTorque(torque);
            
            // 短暂延迟
            delay(0.005); // 5ms控制周期
        }
    }
};
```

## 常见问题

### Q1: 如何选择合适的采样时间？
**A:** 采样时间应远小于系统的时间常数。一般建议：
- 快速系统（电机、伺服）：1-10ms
- 中速系统（温度、压力）：100ms-1s
- 慢速系统（液位、pH值）：1-10s

### Q2: 积分项饱和怎么办？
**A:** 本库已内置抗积分饱和机制。确保：
1. 启用 `enableAntiWindup = true`
2. 设置合理的 `limitI` 值
3. 考虑使用积分下限 `limitIMin`

### Q3: 微分项放大噪声怎么办？
**A:** 可以：
1. 降低 `Kd` 值
2. 对反馈信号进行滤波
3. 使用带滤波的微分项（本库已实现基本滤波）

### Q4: 如何调试PID参数？
**A:** 建议步骤：
1. 先调P，使系统有响应但不振荡
2. 再调I，消除稳态误差
3. 最后调D，抑制超调
4. 使用阶跃响应观察系统行为

### Q5: 什么时候使用带死区的PID？
**A:** 当系统存在以下情况时：
1. 执行器有死区（如继电器的吸合电压）
2. 希望减少小误差时的频繁调整
3. 系统存在测量噪声，希望忽略小波动

### Q6: 什么时候使用带前馈的PID？
**A:** 当系统需要：
1. 对目标值变化快速响应
2. 减少跟踪误差
3. 提高系统带宽

## 集成到项目

### CMake集成
```cmake
# 在CMakeLists.txt中添加
add_library(PID STATIC src/PID.cpp)
target_include_directories(PID PUBLIC include)

# 在其他目标中链接
target_link_libraries(your_target PID)
```

### 项目结构建议
```
your_project/
├── include/
│   └── PID.hpp
├── src/
│   └── PID.cpp
├── examples/
│   ├── motor_control.cpp
│   └── temperature_control.cpp
└── CMakeLists.txt
```

## 性能考虑

### 计算复杂度
- 每次 `calculate()` 调用执行约20-30次浮点运算
- 内存占用：约100字节（取决于平台）
- 适合嵌入式系统和实时控制系统

### 线程安全
- 本库不是线程安全的
- 如果需要在多线程环境中使用，需要外部同步
- 建议每个线程使用独立的PID实例

## 版本历史

### v1.0.0 (当前版本)
- 基础PID控制器实现
- 抗积分饱和机制
- 多重限幅功能
- 带死区处理的PID扩展
- 带前馈控制的PID扩展
- 完整的文档和示例

## 许可证

本项目使用MIT许可证。详见LICENSE文件。

## 技术支持

如有问题或建议，请：
1. 查看本文档的常见问题部分
2. 检查示例代码
3. 在项目issue中提出问题

---

*文档最后更新：2025年2月24日*
*版本：1.0.0*