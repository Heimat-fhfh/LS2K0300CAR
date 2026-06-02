#pragma once

#include <chrono>
#include <algorithm>
#include <limits>

namespace Control {

/**
 * @brief 数字PID控制器类
 * 
 * 支持比例(P)、积分(I)、微分(D)控制，带有限幅功能
 * 使用C++11/14特性，类型安全，易于使用
 */
class PID {
public:
    /**
     * @brief PID控制器参数配置
     */
    struct Parameters {
        double Kp{0.0};           // 比例系数
        double Ki{0.0};           // 积分系数
        double Kd{0.0};           // 微分系数
        
        double limitP{100.0};      // 比例项限幅
        double limitI{100.0};      // 积分项限幅
        double limitD{100.0};      // 微分项限幅
        double limitOutput{100.0};  // 输出总限幅
        
        double limitIMin{0.0};      // 积分下限（用于抗积分饱和）
        bool enableAntiWindup{true}; // 使能抗积分饱和
        
        Parameters() = default;
        
        /**
         * @brief 带参数的构造函数
         */
        Parameters(double kp, double ki, double kd, 
                   double outLimit = 100.0, 
                   double iLimit = 100.0);
    };

public:
    /**
     * @brief 默认构造函数（使用默认参数）
     */
    PID();
    
    /**
     * @brief 带参数的构造函数
     * @param params PID参数配置
     */
    explicit PID(const Parameters& params);
    
    /**
     * @brief 虚析构函数（用于多态）
     */
    virtual ~PID() = default;
    
    /**
     * @brief 重置PID控制器状态
     */
    void reset();
    
    /**
     * @brief 更新PID计算（使用当前时间戳）
     * @param setpoint 目标值
     * @param feedback 反馈值（当前测量值）
     * @return 控制输出
     */
    virtual double calculate(double setpoint, double feedback);
    
    /**
     * @brief 更新PID计算（使用指定的时间间隔）
     * @param setpoint 目标值
     * @param feedback 反馈值
     * @param dt 时间间隔（秒）
     * @return 控制输出
     */
    virtual double calculate(double setpoint, double feedback, double dt);
    
    /**
     * @brief 更新PID参数
     * @param params 新的参数
     */
    void setParameters(const Parameters& params);
    
    /**
     * @brief 获取当前PID参数
     * @return 当前参数
     */
    const Parameters& getParameters() const;
    
    /**
     * @brief 获取当前误差
     * @return 当前误差
     */
    double getError() const;
    
    /**
     * @brief 获取当前积分值
     * @return 积分累积值
     */
    double getIntegral() const;
    
    /**
     * @brief 设置积分值（用于特殊情况）
     * @param integral 新的积分值
     */
    void setIntegral(double integral);
    
    /**
     * @brief 检查控制器是否已初始化
     */
    bool isInitialized() const;

protected:
    /**
     * @brief 钳位值到指定范围内
     * @param value 要钳位的值
     * @param limit 限幅值（对称）
     */
    static void clampValue(double& value, double limit);
    
protected:
    Parameters m_params;           // PID参数
    
    double m_error{0.0};           // 当前误差
    double m_prevError{0.0};        // 上一次误差
    double m_integral{0.0};         // 积分累积值
    double m_prevOutput{0.0};       // 上一次输出
    
    std::chrono::steady_clock::time_point m_lastTime;  // 上次计算时间
    bool m_firstCall{true};         // 是否是第一次调用
};

/**
 * @brief 带死区处理的PID控制器
 */
class PIDWithDeadband : public PID {
public:
    PIDWithDeadband(const Parameters& params, double deadband);
    
    double calculate(double setpoint, double feedback) override;
    double calculate(double setpoint, double feedback, double dt) override;
    
private:
    double m_deadband{0.0};
};

/**
 * @brief 带前馈控制的PID控制器
 */
class PIDWithFeedforward : public PID {
public:
    PIDWithFeedforward(const Parameters& params, double ffGain = 1.0);
    
    double calculate(double setpoint, double feedback) override;
    double calculate(double setpoint, double feedback, double dt) override;
    void setFeedforwardGain(double gain);
    
private:
    double m_ffGain{1.0};
};

/**
 * @brief 增量式PID控制器
 *
 * 内部完成累加和限幅，返回完整的 u(k)：
 *   Δu(k) = Kp*(e_k - e_{k-1}) + Ki*e_k*dt + Kd*(e_k - 2*e_{k-1} + e_{k-2})/dt
 *   u(k) = clamp(u(k-1) + Δu(k), ±limitOutput)
 */
class IncrementalPID {
public:
    struct Parameters {
        double Kp{0.0};
        double Ki{0.0};
        double Kd{0.0};
        double limitOutput{1.0};

        Parameters() = default;
        Parameters(double kp, double ki, double kd, double outLimit);
    };

    explicit IncrementalPID(const Parameters& params);
    void reset();
    double calculate(double setpoint, double feedback, double dt);
    double getOutput() const { return m_accumOutput; }
    void setOutput(double val) { m_accumOutput = val; }

private:
    Parameters m_params;
    double m_accumOutput{0.0};
    double m_prevError{0.0};
    double m_prevPrevError{0.0};
    bool m_firstCall{true};
};

} // namespace Control