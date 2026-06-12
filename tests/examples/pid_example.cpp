/**
 * PID库使用示例
 * 
 * 这个示例展示了如何使用Control::PID库进行控制系统设计
 * 包含基础PID、带死区PID和带前馈PID的使用方法
 */

#include "PID.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

using namespace Control;
using namespace std;

// 模拟系统模型
class SimulatedSystem {
private:
    double state;
    double timeConstant;
    
public:
    SimulatedSystem(double initialState = 0.0, double tc = 1.0) 
        : state(initialState), timeConstant(tc) {}
    
    // 更新系统状态（一阶惯性系统）
    double update(double input, double dt) {
        state += (input - state) * dt / timeConstant;
        return state;
    }
    
    double getState() const {
        return state;
    }
    
    void reset(double newState = 0.0) {
        state = newState;
    }
};

// 示例1：基础PID控制
void exampleBasicPID() {
    cout << "=== 示例1：基础PID控制 ===" << endl;
    
    // 创建PID控制器
    PID::Parameters params;
    params.Kp = 2.0;
    params.Ki = 0.5;
    params.Kd = 0.1;
    params.limitOutput = 100.0;
    params.limitI = 30.0;
    params.enableAntiWindup = true;
    
    PID pid(params);
    
    // 创建模拟系统
    SimulatedSystem system(0.0, 0.5);
    
    // 控制参数
    double setpoint = 50.0;
    double dt = 0.01; // 10ms控制周期
    int steps = 500;  // 模拟500步
    
    cout << "目标值: " << setpoint << endl;
    cout << "控制周期: " << dt * 1000 << "ms" << endl;
    cout << "模拟步数: " << steps << endl << endl;
    
    // 控制循环
    for (int i = 0; i < steps; i++) {
        // 获取当前状态
        double feedback = system.getState();
        
        // 计算控制输出
        double output = pid.calculate(setpoint, feedback, dt);
        
        // 更新系统
        system.update(output, dt);
        
        // 每50步打印一次状态
        if (i % 50 == 0) {
            cout << "步数: " << i 
                 << ", 目标: " << setpoint
                 << ", 当前: " << feedback
                 << ", 输出: " << output
                 << ", 误差: " << pid.getError() << endl;
        }
        
        // 模拟时间延迟
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    
    cout << endl << "最终状态: " << system.getState() 
         << ", 稳态误差: " << pid.getError() << endl;
    cout << "==========================" << endl << endl;
}

// 示例2：带死区的PID控制
void examplePIDWithDeadband() {
    cout << "=== 示例2：带死区的PID控制 ===" << endl;
    
    // 创建PID参数
    PID::Parameters params;
    params.Kp = 1.5;
    params.Ki = 0.3;
    params.Kd = 0.05;
    params.limitOutput = 80.0;
    params.limitI = 20.0;
    
    // 创建带死区的PID控制器（死区大小为2.0）
    double deadband = 2.0;
    PIDWithDeadband pid(params, deadband);
    
    // 创建模拟系统
    SimulatedSystem system(0.0, 0.8);
    
    // 控制参数
    double setpoint = 30.0;
    double dt = 0.02; // 20ms控制周期
    int steps = 300;
    
    cout << "目标值: " << setpoint << endl;
    cout << "死区大小: " << deadband << endl;
    cout << "控制周期: " << dt * 1000 << "ms" << endl << endl;
    
    // 控制循环
    for (int i = 0; i < steps; i++) {
        double feedback = system.getState();
        double output = pid.calculate(setpoint, feedback, dt);
        system.update(output, dt);
        
        if (i % 30 == 0) {
            cout << "步数: " << i 
                 << ", 当前: " << feedback
                 << ", 输出: " << output
                 << ", 误差: " << pid.getError();
            
            // 显示是否在死区内
            if (abs(setpoint - feedback) < deadband) {
                cout << " [在死区内]" << endl;
            } else {
                cout << " [在死区外]" << endl;
            }
        }
        
        this_thread::sleep_for(chrono::milliseconds(2));
    }
    
    cout << endl << "最终状态: " << system.getState() << endl;
    cout << "==========================" << endl << endl;
}

// 示例3：带前馈的PID控制
void examplePIDWithFeedforward() {
    cout << "=== 示例3：带前馈的PID控制 ===" << endl;
    
    // 创建PID参数
    PID::Parameters params;
    params.Kp = 3.0;
    params.Ki = 0.8;
    params.Kd = 0.2;
    params.limitOutput = 100.0;
    params.limitP = 60.0;
    params.limitI = 40.0;
    params.limitD = 20.0;
    
    // 创建带前馈的PID控制器（前馈增益0.4）
    double ffGain = 0.4;
    PIDWithFeedforward pid(params, ffGain);
    
    // 创建模拟系统
    SimulatedSystem system(0.0, 0.3);
    
    // 控制参数 - 使用变化的设定点
    double dt = 0.005; // 5ms控制周期
    int steps = 600;
    
    cout << "前馈增益: " << ffGain << endl;
    cout << "控制周期: " << dt * 1000 << "ms" << endl;
    cout << "模拟步数: " << steps << endl << endl;
    
    // 控制循环 - 跟踪变化的设定点
    for (int i = 0; i < steps; i++) {
        // 创建变化的设定点（方波信号）
        double setpoint;
        if (i < 200) {
            setpoint = 20.0;
        } else if (i < 400) {
            setpoint = 40.0;
        } else {
            setpoint = 60.0;
        }
        
        double feedback = system.getState();
        double output = pid.calculate(setpoint, feedback, dt);
        system.update(output, dt);
        
        if (i % 50 == 0) {
            cout << "步数: " << i 
                 << ", 目标: " << setpoint
                 << ", 当前: " << feedback
                 << ", 输出: " << output
                 << ", 误差: " << pid.getError() << endl;
        }
        
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    
    cout << endl << "最终状态: " << system.getState() << endl;
    cout << "==========================" << endl << endl;
}

// 示例4：动态参数调整
void exampleDynamicParameterAdjustment() {
    cout << "=== 示例4：动态参数调整 ===" << endl;
    
    // 创建PID控制器
    PID pid;
    
    // 初始参数
    PID::Parameters initialParams;
    initialParams.Kp = 1.0;
    initialParams.Ki = 0.2;
    initialParams.Kd = 0.05;
    initialParams.limitOutput = 100.0;
    
    pid.setParameters(initialParams);
    
    // 创建模拟系统
    SimulatedSystem system(0.0, 0.6);
    
    double setpoint = 40.0;
    double dt = 0.01;
    int steps = 400;
    
    cout << "初始参数: Kp=" << initialParams.Kp 
         << ", Ki=" << initialParams.Ki 
         << ", Kd=" << initialParams.Kd << endl;
    cout << "目标值: " << setpoint << endl << endl;
    
    // 控制循环
    for (int i = 0; i < steps; i++) {
        double feedback = system.getState();
        double output = pid.calculate(setpoint, feedback, dt);
        system.update(output, dt);
        
        // 在第200步时动态调整参数
        if (i == 200) {
            PID::Parameters newParams = pid.getParameters();
            newParams.Kp = 2.0;  // 增加比例增益
            newParams.Ki = 0.4;  // 增加积分增益
            pid.setParameters(newParams);
            
            cout << "第200步：动态调整参数" << endl;
            cout << "新参数: Kp=" << newParams.Kp 
                 << ", Ki=" << newParams.Ki 
                 << ", Kd=" << newParams.Kd << endl;
        }
        
        if (i % 40 == 0) {
            cout << "步数: " << i 
                 << ", 当前: " << feedback
                 << ", 输出: " << output
                 << ", 误差: " << pid.getError() << endl;
        }
        
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    
    cout << endl << "最终状态: " << system.getState() << endl;
    cout << "==========================" << endl << endl;
}

// 示例5：重置和状态管理
void exampleResetAndStateManagement() {
    cout << "=== 示例5：重置和状态管理 ===" << endl;
    
    PID::Parameters params;
    params.Kp = 2.5;
    params.Ki = 0.6;
    params.Kd = 0.15;
    params.limitOutput = 100.0;
    params.limitI = 25.0;
    
    PID pid(params);
    SimulatedSystem system(0.0, 0.4);
    
    double setpoint = 50.0;
    double dt = 0.01;
    
    cout << "演示PID控制器的状态管理功能" << endl << endl;
    
    // 第一阶段：正常控制
    cout << "第一阶段：正常控制" << endl;
    for (int i = 0; i < 100; i++) {
        double feedback = system.getState();
        pid.calculate(setpoint, feedback, dt);
        system.update(pid.getError() > 0 ? 30.0 : -30.0, dt);
    }
    
    cout << "控制后状态:" << endl;
    cout << "  当前误差: " << pid.getError() << endl;
    cout << "  积分值: " << pid.getIntegral() << endl;
    cout << "  已初始化: " << (pid.isInitialized() ? "是" : "否") << endl;
    
    // 重置控制器
    cout << endl << "重置控制器..." << endl;
    pid.reset();
    system.reset(10.0); // 同时重置系统状态
    
    cout << "重置后状态:" << endl;
    cout << "  当前误差: " << pid.getError() << endl;
    cout << "  积分值: " << pid.getIntegral() << endl;
    cout << "  已初始化: " << (pid.isInitialized() ? "是" : "否") << endl;
    
    // 第二阶段：手动设置积分值
    cout << endl << "手动设置积分值为10.0..." << endl;
    pid.setIntegral(10.0);
    
    // 继续控制
    cout << "继续控制..." << endl;
    for (int i = 0; i < 50; i++) {
        double feedback = system.getState();
        double output = pid.calculate(setpoint, feedback, dt);
        system.update(output, dt);
    }
    
    cout << "最终状态:" << endl;
    cout << "  系统状态: " << system.getState() << endl;
    cout << "  控制器误差: " << pid.getError() << endl;
    cout << "  控制器积分值: " << pid.getIntegral() << endl;
    
    cout << "==========================" << endl << endl;
}

int main() {
    cout << "PID控制器库使用示例程序" << endl;
    cout << "========================" << endl << endl;
    
    // 运行所有示例
    exampleBasicPID();
    examplePIDWithDeadband();
    examplePIDWithFeedforward();
    exampleDynamicParameterAdjustment();
    exampleResetAndStateManagement();
    
    cout << "所有示例运行完成！" << endl;
    cout << "更多详细信息请参考PID_使用文档.md" << endl;
    
    return 0;
}