#include "MotorControlTask.hpp"
#include <pthread.h>
#include <cmath>
#include <cstring>


MotorControlTask::MotorControlTask(
    const Control::PID::Parameters& leftParams,
    const Control::PID::Parameters& rightParams,
    DualMotorController* motors,
    Encoder* leftEncoder,
    Encoder* rightEncoder,
    double controlPeriod
)
    : leftTargetSpeed(0.0)
    , rightTargetSpeed(0.0)
    , targetAngularVelocity(0.0)
    , baseSpeed(0.0)
    , angularVelocityControlEnabled(false)
    , leftParams(leftParams)
    , rightParams(rightParams)
    , motors(motors)
    , leftEncoder(leftEncoder)
    , rightEncoder(rightEncoder)
    , imu(nullptr)
    , wheelbase(0.158)      // 默认轮距15.8cm = 0.158m
    , wheelRadius(0.0325)   // 默认车轮半径3.25cm = 0.0325m
    , controlPeriod(controlPeriod)
    , running(false)
    , taskError(false)
    , workerThread(nullptr)
    , rtPriority(50)
    , rtPolicy(SCHED_FIFO) {
    
    if (!motors || !leftEncoder || !rightEncoder) {
        throw std::invalid_argument("MotorControlTask: Null pointer provided");
    }
    
    if (controlPeriod <= 0.0) {
        throw std::invalid_argument("MotorControlTask: Invalid control period");
    }
    
    // 初始化角速度PID参数
    angularVelocityParams.Kp = 0.65;      // 适中的反应速度
    angularVelocityParams.Ki = 0.25;      // 较温和的误差消除
    angularVelocityParams.Kd = 0.015;     // 微弱的阻尼，防止超调
    
    angularVelocityParams.limitP = 40.0;  // 允许比例项产生较大的修正
    angularVelocityParams.limitI = 15.0;  // 限制积分项，防止严重超调
    angularVelocityParams.limitD = 10.0;  // 限制微分震荡
    
    // 总输出限幅：
    // 如果偏差很大，允许 PID 在目标值基础上最多补偿 ±50°/s
    angularVelocityParams.limitOutput = 50.0; 
    
    angularVelocityParams.limitIMin = -15.0;
    angularVelocityParams.enableAntiWindup = true; // 必须开启
}

MotorControlTask::MotorControlTask(
    const Control::PID::Parameters& leftParams,
    const Control::PID::Parameters& rightParams,
    DualMotorController* motors,
    Encoder* leftEncoder,
    Encoder* rightEncoder,
    IMUDevice* imu,
    double controlPeriod
)
    : leftTargetSpeed(0.0)
    , rightTargetSpeed(0.0)
    , targetAngularVelocity(0.0)
    , baseSpeed(0.0)
    , angularVelocityControlEnabled(false)
    , leftParams(leftParams)
    , rightParams(rightParams)
    , motors(motors)
    , leftEncoder(leftEncoder)
    , rightEncoder(rightEncoder)
    , imu(imu)
    , wheelbase(0.158)      // 默认轮距15.8cm = 0.158m
    , wheelRadius(0.0325)   // 默认车轮半径3.25cm = 0.0325m
    , controlPeriod(controlPeriod)
    , running(false)
    , taskError(false)
    , workerThread(nullptr)
    , rtPriority(50)
    , rtPolicy(SCHED_FIFO) {
    
    if (!motors || !leftEncoder || !rightEncoder) {
        throw std::invalid_argument("MotorControlTask: Null pointer provided");
    }
    
    if (controlPeriod <= 0.0) {
        throw std::invalid_argument("MotorControlTask: Invalid control period");
    }
    
    // 初始化角速度PID参数
    angularVelocityParams.Kp = 0.2;      // 适中的反应速度
    angularVelocityParams.Ki = 0.05;      // 较温和的误差消除
    angularVelocityParams.Kd = 0.0;     // 微弱的阻尼，防止超调
    
    angularVelocityParams.limitP = 40.0;  // 允许比例项产生较大的修正
    angularVelocityParams.limitI = 15.0;  // 限制积分项，防止严重超调
    angularVelocityParams.limitD = 10.0;  // 限制微分震荡
    
    // 总输出限幅：
    // 如果偏差很大，允许 PID 在目标值基础上最多补偿 ±50°/s
    angularVelocityParams.limitOutput = 30.0; 
    
    angularVelocityParams.limitIMin = -15.0;
    angularVelocityParams.enableAntiWindup = true; // 必须开启
}

MotorControlTask::~MotorControlTask() {
    stop();
}

bool MotorControlTask::start() {
    if (workerThread && workerThread->joinable()) {
        printf("Motor control task already running\n");
        return false;
    }
    
    running = true;
    taskError = false;
    
    workerThread = std::make_unique<std::thread>(&MotorControlTask::run, this);
    
    // 设置实时优先级
    pthread_t native_handle = workerThread->native_handle();
    struct sched_param param;
    param.sched_priority = rtPriority;
    
    if (pthread_setschedparam(native_handle, rtPolicy, &param) != 0) {
        printf("Warning: Failed to set realtime priority for motor control thread\n");
    }
    
    printf("Motor control task started with period %.3f s\n", controlPeriod);
    return true;
}

void MotorControlTask::stop() {
    if (workerThread && workerThread->joinable()) {
        running = false;
        workerThread->join();
        workerThread.reset();
        printf("Motor control task stopped\n");
    }
}

void MotorControlTask::setTargetSpeed(double leftSpeed, double rightSpeed) {
    leftTargetSpeed.store(leftSpeed);
    rightTargetSpeed.store(rightSpeed);
    printf("Target speeds updated - Left: %.3f m/s, Right: %.3f m/s\n", leftSpeed, rightSpeed);
}

void MotorControlTask::setLeftTargetSpeed(double speed) {
    leftTargetSpeed.store(speed);
    printf("Left target speed updated to: %.3f m/s\n", speed);
}

void MotorControlTask::setRightTargetSpeed(double speed) {
    rightTargetSpeed.store(speed);
    printf("Right target speed updated to: %.3f m/s\n", speed);
}

double MotorControlTask::getLeftTargetSpeed() const {
    return leftTargetSpeed.load();
}

double MotorControlTask::getRightTargetSpeed() const {
    return rightTargetSpeed.load();
}

std::pair<double, double> MotorControlTask::getTargetSpeeds() const {
    return {leftTargetSpeed.load(), rightTargetSpeed.load()};
}

bool MotorControlTask::isRunning() const {
    return running.load() && workerThread && workerThread->joinable();
}

bool MotorControlTask::hasError() const {
    return taskError.load();
}

void MotorControlTask::setRealtimePriority(int priority, int policy) {
    rtPriority = priority;
    rtPolicy = policy;
}

bool MotorControlTask::isValidSpeed(double speed) const {
    return !std::isnan(speed) && !std::isinf(speed) && speed >= -10.0 && speed <= 10.0;
}

void MotorControlTask::run() {
    enum class ControlState {
        INIT,
        RUNNING,
        ERROR,
        STOPPED
    };
    
    ControlState state = ControlState::INIT;
    
    // 创建左右轮PID控制器
    Control::PID leftPID(leftParams);
    Control::PID rightPID(rightParams);
    
    // 创建角速度PID控制器（如果需要）
    Control::PID angularVelocityPID(angularVelocityParams);
    
    // 设置电机限制
    motors->setMaxDutyLimits(50.0f);
    
    // 监控变量
    const int maxErrorCount = 10;
    int leftErrorCount = 0;
    int rightErrorCount = 0;
    int imuErrorCount = 0;
    double lastValidLeftSpeed = 0.0;
    double lastValidRightSpeed = 0.0;
    double lastValidAngularVelocity = 0.0;
    
    printf("Motor control task initialized\n");
    state = ControlState::RUNNING;
    
    // 精确计时
    auto next_time = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(static_cast<int>(controlPeriod * 1000000));
    
    while (state != ControlState::STOPPED && running.load()) {
        try {
            next_time += period;
            
            // 读取左右轮速度
            double leftSpeed = leftEncoder->readSpeed();
            double rightSpeed = rightEncoder->readSpeed();
            
            // 左轮速度有效性检查
            if (!isValidSpeed(leftSpeed)) {
                leftErrorCount++;
                printf("Warning: Invalid left speed: %.3f (error %d/%d)\n", 
                       leftSpeed, leftErrorCount, maxErrorCount);
                leftSpeed = lastValidLeftSpeed;
                
                if (leftErrorCount >= maxErrorCount) {
                    printf("ERROR: Too many invalid left speed readings\n");
                    state = ControlState::ERROR;
                    taskError = true;
                    break;
                }
            } else {
                leftErrorCount = 0;
                lastValidLeftSpeed = leftSpeed;
            }
            
            // 右轮速度有效性检查
            if (!isValidSpeed(rightSpeed)) {
                rightErrorCount++;
                printf("Warning: Invalid right speed: %.3f (error %d/%d)\n", 
                       rightSpeed, rightErrorCount, maxErrorCount);
                rightSpeed = lastValidRightSpeed;
                
                if (rightErrorCount >= maxErrorCount) {
                    printf("ERROR: Too many invalid right speed readings\n");
                    state = ControlState::ERROR;
                    taskError = true;
                    break;
                }
            } else {
                rightErrorCount = 0;
                lastValidRightSpeed = rightSpeed;
            }
            
            // 获取当前目标速度
            double currentLeftTarget = leftTargetSpeed.load();
            double currentRightTarget = rightTargetSpeed.load();
            
            // 角速度控制逻辑
            bool angularControlEnabled = angularVelocityControlEnabled.load();
            if (angularControlEnabled && imu != nullptr) {
                // 读取IMU数据获取实际角速度
                double actualAngularVelocity = 0.0;
                if (imu->update_all_data()) {
                    const imu_unit_data_t& unit_data = imu->get_unit_data();
                    actualAngularVelocity = unit_data.gyro_z;  // Z轴角速度（°/s）
                    
                    // 角速度有效性检查
                    if (!isValidAngularVelocity(actualAngularVelocity)) {
                        imuErrorCount++;
                        printf("Warning: Invalid angular velocity: %.3f °/s (error %d/%d)\n", 
                               actualAngularVelocity, imuErrorCount, maxErrorCount);
                        actualAngularVelocity = lastValidAngularVelocity;
                        
                        if (imuErrorCount >= maxErrorCount) {
                            printf("ERROR: Too many invalid angular velocity readings\n");
                            state = ControlState::ERROR;
                            taskError = true;
                            break;
                        }
                    } else {
                        imuErrorCount = 0;
                        lastValidAngularVelocity = actualAngularVelocity;
                    }
                } else {
                    printf("Warning: Failed to update IMU data\n");
                }
                
                // 获取目标角速度和基础速度
                double targetAngVel = targetAngularVelocity.load();
                double currentBaseSpeed = baseSpeed.load();
                
                // 角速度PID控制
                double angularVelocityError = targetAngVel - actualAngularVelocity;
                double angularControlOutput = angularVelocityPID.calculate(targetAngVel, actualAngularVelocity, controlPeriod);
                printf("%.2f,%.2f,%.2f\n",actualAngularVelocity,angularVelocityError,angularControlOutput);
                
                // 将角速度控制输出转换为速度差
                // 注意：angularControlOutput是角速度误差的修正量（°/s）
                double correctedAngularVelocity = targetAngVel + angularControlOutput;
                
                // 单位转换：°/s -> rad/s
                double correctedAngularVelocityRad = degToRad(correctedAngularVelocity);
                
                // 运动学分解：计算左右轮目标速度
                auto [leftTarget, rightTarget] = kinematicsDecomposition(currentBaseSpeed, correctedAngularVelocityRad);
                
                // 更新目标速度
                currentLeftTarget = leftTarget;
                currentRightTarget = rightTarget;
                
                // 更新原子变量（用于显示）
                leftTargetSpeed.store(leftTarget);
                rightTargetSpeed.store(rightTarget);
            }
            
            // PID计算
            double leftOutput = leftPID.calculate(currentLeftTarget, leftSpeed, controlPeriod);
            double rightOutput = rightPID.calculate(currentRightTarget, rightSpeed, controlPeriod);
            
            // 应用控制输出
            motors->setSpeeds(static_cast<float>(leftOutput), static_cast<float>(rightOutput));
            
            // 周期性状态输出（每秒一次）
            static int cycleCount = 0;
            if (++cycleCount >= static_cast<int>(1.0 / controlPeriod)) {
                bool angularControlEnabled = angularVelocityControlEnabled.load();
                
                if (angularControlEnabled && imu != nullptr) {
                    double targetAngVel = targetAngularVelocity.load();
                    double currentBaseSpeed = baseSpeed.load();
                    
                    printf("Status - Base:%.3f m/s, AngTarget:%.3f °/s | "
                           "Left: Target=%.3f, Current=%.3f, Output=%.3f | "
                           "Right: Target=%.3f, Current=%.3f, Output=%.3f\n",
                           currentBaseSpeed, targetAngVel,
                           currentLeftTarget, leftSpeed, leftOutput,
                           currentRightTarget, rightSpeed, rightOutput);
                    
                    // UDP发送扩展的PID状态数据（包含角速度信息）
                    std::array<float, 12> data = {
                        (float)currentBaseSpeed,
                        (float)targetAngVel,
                        (float)angularVelocityPID.getError(),
                        (float)currentLeftTarget,
                        (float)leftSpeed,
                        (float)leftOutput,
                        (float)leftPID.getError(),
                        (float)currentRightTarget,
                        (float)rightSpeed,
                        (float)rightOutput,
                        (float)rightPID.getError(),
                        (float)angularVelocityPID.getIntegral()
                    };
                    
                    send_udp_data("motor_angular_status", data.data(), data.size());
                } else {
                    printf("Status - Left: Target=%.3f, Current=%.3f, Output=%.3f | "
                           "Right: Target=%.3f, Current=%.3f, Output=%.3f\n",
                           currentLeftTarget, leftSpeed, leftOutput,
                           currentRightTarget, rightSpeed, rightOutput);
                    
                    // UDP发送PID状态数据
                    std::array<float, 8> data = {
                        (float)currentLeftTarget,
                        (float)leftSpeed,
                        (float)leftOutput,
                        (float)leftPID.getError(),
                        (float)leftPID.getIntegral(),
                        (float)currentRightTarget,
                        (float)rightSpeed,
                        (float)rightOutput
                    };
                    
                    send_udp_data("motor_status", data.data(), data.size());
                }
                
                cycleCount = 0;
            }
            
        } catch (const std::exception& e) {
            printf("Exception in control loop: %s\n", e.what());
            motors->setSpeeds(0.0f, 0.0f);
            state = ControlState::ERROR;
            taskError = true;
            break;
        }
        
        // 等待到下一个控制周期
        std::this_thread::sleep_until(next_time);
    }
    
    // 确保停止电机
    motors->setSpeeds(0.0f, 0.0f);
    
    if (state == ControlState::ERROR) {
        printf("Motor control task terminated due to errors.\n");
    } else {
        printf("Motor control task stopped normally.\n");
    }
}

// ==================== 角速度控制相关方法实现 ====================

void MotorControlTask::setTargetAngularVelocity(double angularVelocity) {
    if (!isValidAngularVelocity(angularVelocity)) {
        printf("Warning: Invalid angular velocity: %.3f °/s\n", angularVelocity);
        return;
    }
    
    targetAngularVelocity.store(angularVelocity);
    printf("Target angular velocity updated to: %.3f °/s\n", angularVelocity);
}

double MotorControlTask::getTargetAngularVelocity() const {
    return targetAngularVelocity.load();
}

void MotorControlTask::enableAngularVelocityControl(bool enable) {
    bool wasEnabled = angularVelocityControlEnabled.load();
    angularVelocityControlEnabled.store(enable);
    
    if (enable && !wasEnabled) {
        printf("Angular velocity control enabled\n");
        if (imu == nullptr) {
            printf("Warning: IMU device not available for angular velocity control\n");
        }
    } else if (!enable && wasEnabled) {
        printf("Angular velocity control disabled\n");
    }
}

bool MotorControlTask::isAngularVelocityControlEnabled() const {
    return angularVelocityControlEnabled.load();
}

void MotorControlTask::setBaseSpeed(double baseSpeed) {
    if (!isValidSpeed(baseSpeed)) {
        printf("Warning: Invalid base speed: %.3f m/s\n", baseSpeed);
        return;
    }
    
    this->baseSpeed.store(baseSpeed);
    printf("Base speed updated to: %.3f m/s\n", baseSpeed);
}

double MotorControlTask::getBaseSpeed() const {
    return baseSpeed.load();
}

void MotorControlTask::setWheelbase(double wheelbase) {
    if (wheelbase <= 0.0) {
        printf("Warning: Invalid wheelbase: %.3f m\n", wheelbase);
        return;
    }
    
    this->wheelbase = wheelbase;
    printf("Wheelbase updated to: %.3f m\n", wheelbase);
}

void MotorControlTask::setWheelRadius(double wheelRadius) {
    if (wheelRadius <= 0.0) {
        printf("Warning: Invalid wheel radius: %.3f m\n", wheelRadius);
        return;
    }
    
    this->wheelRadius = wheelRadius;
    printf("Wheel radius updated to: %.3f m\n", wheelRadius);
}

bool MotorControlTask::isValidAngularVelocity(double angularVelocity) const {
    return !std::isnan(angularVelocity) && !std::isinf(angularVelocity) && 
           angularVelocity >= -1000.0 && angularVelocity <= 1000.0;
}

std::pair<double, double> MotorControlTask::kinematicsDecomposition(double baseSpeed, double angularVelocityRad) const {
    // 计算速度差：Δv = ω * L / 2
    double deltaV = angularVelocityRad * wheelbase / 2.0;
    
    // 计算左右轮速度
    double leftSpeed = baseSpeed - deltaV;
    double rightSpeed = baseSpeed + deltaV;
    
    return {leftSpeed, rightSpeed};
}

double MotorControlTask::degToRad(double deg) const {
    return deg * M_PI / 180.0;
}

double MotorControlTask::radToDeg(double rad) const {
    return rad * 180.0 / M_PI;
}
