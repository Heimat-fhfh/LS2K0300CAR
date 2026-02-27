

#include "PID.hpp"
#include <cmath>
#include <iostream>

using namespace std;


namespace Control {

//=============================================================================
// PID::Parameters 构造函数
//=============================================================================
PID::Parameters::Parameters(double kp, double ki, double kd, 
                           double outLimit, double iLimit)
    : Kp(kp), Ki(ki), Kd(kd)
    , limitP(outLimit), limitI(iLimit)
    , limitD(outLimit), limitOutput(outLimit) {}

//=============================================================================
// PID 默认构造函数
//=============================================================================
PID::PID() {
    reset();
}

//=============================================================================
// PID 带参数构造函数
//=============================================================================
PID::PID(const Parameters& params) 
    : m_params(params) {
    reset();
}

//=============================================================================
// PID::reset
//=============================================================================
void PID::reset() {
    m_error = 0.0;
    m_prevError = 0.0;
    m_integral = 0.0;
    m_prevOutput = 0.0;
    m_lastTime = std::chrono::steady_clock::time_point{};
    m_firstCall = true;
}

//=============================================================================
// PID::calculate（使用当前时间戳）
//=============================================================================
double PID::calculate(double setpoint, double feedback) {
    auto now = std::chrono::steady_clock::now();
    
    // 如果是第一次调用，只记录时间，不进行计算
    if (m_firstCall) {
        m_lastTime = now;
        m_firstCall = false;
        m_prevError = setpoint - feedback;
        return 0.0;
    }
    
    // 计算时间差（秒）
    double dt = std::chrono::duration<double>(now - m_lastTime).count();
    m_lastTime = now;
    
    return calculate(setpoint, feedback, dt);
}

//=============================================================================
// PID::calculate（使用指定时间间隔）
//=============================================================================
double PID::calculate(double setpoint, double feedback, double dt) {
    // 计算误差
    m_error = setpoint - feedback;
    
    // 计算比例项
    double pTerm = m_params.Kp * m_error;
    clampValue(pTerm, m_params.limitP);
    
    // 计算积分项（带抗积分饱和）
    if (m_params.Ki != 0.0 && dt > 0.0) {
        m_integral += m_error * dt;
        
        // 抗积分饱和
        if (m_params.enableAntiWindup) {
            // 检查输出是否饱和，如果饱和则停止积分累积
            double tempOutput = pTerm + m_params.Ki * m_integral + 
                                m_params.Kd * (m_error - m_prevError) / dt;
            
            if ((tempOutput >= m_params.limitOutput && m_error > 0) ||
                (tempOutput <= -m_params.limitOutput && m_error < 0)) {
                // 输出饱和且误差方向相同，不累积积分
                m_integral -= m_error * dt;
            }
        }
        
        // 积分限幅
        clampValue(m_integral, m_params.limitI / m_params.Ki);
        
        // 积分下限限制（可选）
        if (m_params.limitIMin > 0.0) {
            m_integral = std::max(m_integral, -m_params.limitIMin);
        }
    }
    
    double iTerm = m_params.Ki * m_integral;
    clampValue(iTerm, m_params.limitI);
    
    // 计算微分项（带滤波）
    double dTerm = 0.0;
    if (m_params.Kd != 0.0 && dt > 0.0) {
        dTerm = m_params.Kd * (m_error - m_prevError) / dt;
        clampValue(dTerm, m_params.limitD);
    }
    
    // 计算总输出
    double output = pTerm + iTerm + dTerm;
    // cout << "PID Debug - P: " << pTerm << ", I: " << iTerm << ", D: " << dTerm << ", Output before clamp: " << output << endl;
    clampValue(output, m_params.limitOutput);
    
    // 更新状态
    m_prevError = m_error;
    m_prevOutput = output;
    
    return output;
}

//=============================================================================
// PID::setParameters
//=============================================================================
void PID::setParameters(const Parameters& params) {
    m_params = params;
}

//=============================================================================
// PID::getParameters
//=============================================================================
const PID::Parameters& PID::getParameters() const {
    return m_params;
}

//=============================================================================
// PID::getError
//=============================================================================
double PID::getError() const {
    return m_error;
}

//=============================================================================
// PID::getIntegral
//=============================================================================
double PID::getIntegral() const {
    return m_integral;
}

//=============================================================================
// PID::setIntegral
//=============================================================================
void PID::setIntegral(double integral) {
    m_integral = integral;
}

//=============================================================================
// PID::isInitialized
//=============================================================================
bool PID::isInitialized() const {
    return !m_firstCall;
}

//=============================================================================
// PID::clampValue（私有静态方法）
//=============================================================================
void PID::clampValue(double& value, double limit) {
    if (limit > 0.0) {
        // 兼容C++11的clamp实现
        if (value > limit) {
            value = limit;
        } else if (value < -limit) {
            value = -limit;
        }
    }
}

//=============================================================================
// PIDWithDeadband 实现
//=============================================================================
PIDWithDeadband::PIDWithDeadband(const Parameters& params, double deadband)
    : PID(params), m_deadband(std::abs(deadband)) {}

double PIDWithDeadband::calculate(double setpoint, double feedback, double dt) {
    // 如果误差在死区内，不进行控制
    if (std::abs(setpoint - feedback) < m_deadband) {
        return 0.0;
    }
    return PID::calculate(setpoint, feedback, dt);
}

double PIDWithDeadband::calculate(double setpoint, double feedback) {
    if (std::abs(setpoint - feedback) < m_deadband) {
        return 0.0;
    }
    return PID::calculate(setpoint, feedback);
}

//=============================================================================
// PIDWithFeedforward 实现
//=============================================================================
PIDWithFeedforward::PIDWithFeedforward(const Parameters& params, double ffGain)
    : PID(params), m_ffGain(ffGain) {}

double PIDWithFeedforward::calculate(double setpoint, double feedback) {
    double pidOutput = PID::calculate(setpoint, feedback);
    // 简单前馈：根据目标值直接贡献输出
    double feedforward = m_ffGain * setpoint;
    return pidOutput + feedforward;
}

double PIDWithFeedforward::calculate(double setpoint, double feedback, double dt) {
    double pidOutput = PID::calculate(setpoint, feedback, dt);
    // 简单前馈：根据目标值直接贡献输出
    double feedforward = m_ffGain * setpoint;
    return pidOutput + feedforward;
}

void PIDWithFeedforward::setFeedforwardGain(double gain) {
    m_ffGain = gain;
}

} // namespace Control

