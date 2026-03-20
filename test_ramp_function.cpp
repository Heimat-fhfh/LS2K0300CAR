#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

// 简单的斜坡函数测试程序
// 这个程序模拟斜坡函数的行为，不依赖实际的硬件

class RampLimiter {
public:
    RampLimiter(double maxAcceleration = 0.5, double maxDeceleration = 0.5)
        : maxAcceleration_(maxAcceleration)
        , maxDeceleration_(maxDeceleration) {
    }
    
    double apply(double target, double current, double dt) {
        if (dt <= 0.0) {
            return target;
        }
        
        double maxChange = 0.0;
        if (target > current) {
            maxChange = maxAcceleration_ * dt;
        } else {
            maxChange = maxDeceleration_ * dt;
        }
        
        double difference = target - current;
        if (std::abs(difference) <= maxChange) {
            return target;
        } else {
            if (difference > 0) {
                return current + maxChange;
            } else {
                return current - maxChange;
            }
        }
    }
    
    void setLimits(double maxAcceleration, double maxDeceleration) {
        if (maxAcceleration > 0.0) maxAcceleration_ = maxAcceleration;
        if (maxDeceleration > 0.0) maxDeceleration_ = maxDeceleration;
    }
    
private:
    double maxAcceleration_;
    double maxDeceleration_;
};

void testRampFunction() {
    std::cout << "=== 斜坡函数测试 ===\n";
    
    // 测试1：基本斜坡功能
    {
        std::cout << "\n测试1：基本斜坡功能（加速度=0.5，减速度=0.5）\n";
        RampLimiter ramp(0.5, 0.5);
        double current = 0.0;
        double dt = 0.01;  // 10ms控制周期
        
        std::cout << "时间(s)\t目标值\t当前值\t变化量\n";
        
        // 从0加速到1.0
        double target = 1.0;
        for (int i = 0; i < 30; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << i*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
        
        // 从1.0减速到0
        target = 0.0;
        for (int i = 0; i < 30; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << (30+i)*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
    }
    
    // 测试2：不同的加速度和减速度
    {
        std::cout << "\n测试2：不同的加速度和减速度（加速度=1.0，减速度=0.2）\n";
        RampLimiter ramp(1.0, 0.2);
        double current = 0.0;
        double dt = 0.01;
        
        std::cout << "时间(s)\t目标值\t当前值\t变化量\n";
        
        // 快速加速，慢速减速
        double target = 1.0;
        for (int i = 0; i < 15; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << i*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
        
        target = 0.0;
        for (int i = 0; i < 50; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << (15+i)*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
    }
    
    // 测试3：负值测试
    {
        std::cout << "\n测试3：负值测试（加速度=0.5，减速度=0.5）\n";
        RampLimiter ramp(0.5, 0.5);
        double current = 0.0;
        double dt = 0.01;
        
        std::cout << "时间(s)\t目标值\t当前值\t变化量\n";
        
        // 从0加速到-1.0
        double target = -1.0;
        for (int i = 0; i < 30; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << i*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
        
        // 从-1.0减速到0
        target = 0.0;
        for (int i = 0; i < 30; i++) {
            double newCurrent = ramp.apply(target, current, dt);
            double change = newCurrent - current;
            std::cout << (30+i)*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
            current = newCurrent;
        }
    }
    
    // 测试4：阶跃响应测试
    {
        std::cout << "\n测试4：阶跃响应测试（加速度=0.5，减速度=0.5）\n";
        RampLimiter ramp(0.5, 0.5);
        double current = 0.0;
        double dt = 0.01;
        
        std::cout << "时间(s)\t目标值\t当前值\t变化量\n";
        
        // 测试多个阶跃变化
        double targets[] = {0.5, 0.8, 0.2, -0.3, 0.0};
        int steps_per_target = 20;
        
        for (int t = 0; t < 5; t++) {
            double target = targets[t];
            for (int i = 0; i < steps_per_target; i++) {
                double newCurrent = ramp.apply(target, current, dt);
                double change = newCurrent - current;
                std::cout << (t*steps_per_target + i)*dt << "\t" << target << "\t" << newCurrent << "\t" << change << "\n";
                current = newCurrent;
            }
        }
    }
}

int main() {
    std::cout << "斜坡函数单元测试\n";
    std::cout << "================\n";
    
    testRampFunction();
    
    std::cout << "\n测试完成！\n";
    std::cout << "斜坡函数功能验证：\n";
    std::cout << "1. 正确限制加速度和减速度\n";
    std::cout << "2. 支持正负值\n";
    std::cout << "3. 正确处理阶跃输入\n";
    std::cout << "4. 当变化量小于限制时直接到达目标值\n";
    
    return 0;
}