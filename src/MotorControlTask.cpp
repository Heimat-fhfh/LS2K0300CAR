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
    IMUDevice* imu,
    double controlPeriod
)
    : leftTargetSpeed(0.0)
    , rightTargetSpeed(0.0)
    , targetAngularVelocity(0.0)
    , baseSpeed(0.0)
    , lastActualAngularVelocity(0.0)
    , lastAngularVelocityPidOutput(0.0)
    , angularVelocityControlEnabled(false)
    , lowPassFilterEnabled(true)          // 默认启用低通滤波
    , speedFilterTau(0.02)
    , angularFilterTau(0.02)
    , rampLimitingEnabled(false)          // 默认禁用斜坡限制
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
    , rtPolicy(SCHED_FIFO)
    , leftSpeedFilter(0.02, controlPeriod)
    , rightSpeedFilter(0.02, controlPeriod)
    , angularVelocityFilter(0.02, controlPeriod)
    , leftRampLimiter(0.5, 0.5)
    , rightRampLimiter(0.5, 0.5)
    , lastLeftOutput_(0.0)
    , lastRightOutput_(0.0)
    , motorMaxDuty(50.0f)
    , motorMinSpeed(0.0)
    , collisionProtectEnabled(false)
    , collisionImuJerkThreshold(3.0)
    , collisionStallDutyThreshold(0.2)
    , collisionStallSpeedThreshold(0.01)
    , collisionStallCycles(20)
    , collisionResetKey(0)
    , collisionBumperKey(-1)
    , collisionDetected(false)
    , stallCounter_(0) {
    
    if (!motors || !leftEncoder || !rightEncoder) {
        throw std::invalid_argument("MotorControlTask: Null pointer provided");
    }
    
    if (controlPeriod <= 0.0) {
        throw std::invalid_argument("MotorControlTask: Invalid control period");
    }
    
    // 角速度PID参数由 setAngularVelocityParams() 设置
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
        COLLISION,
        ERROR,
        STOPPED
    };
    
    ControlState state = ControlState::INIT;
    
    Control::PID leftPID(leftParams);leftPID.setIntegral(0); // 设置初始积分值，帮助快速响应
    Control::PID rightPID(rightParams);rightPID.setIntegral(0); // 设置初始积分值，帮助快速响应

    // 创建角速度PID控制器（如果需要）
    Control::PID angularVelocityPID(angularVelocityParams);
    
    // 设置电机限制
    motors->setMaxDutyLimits(static_cast<float>(motorMaxDuty));
    
    // 监控变量
    const int maxErrorCount = 10;
    int leftErrorCount = 0;
    int rightErrorCount = 0;
    int imuErrorCount = 0;
    double lastValidLeftSpeed = 0.0;
    double lastValidRightSpeed = 0.0;
    double lastValidAngularVelocity = 0.0;
    int intervalCounter = 0;
    
    printf("Motor control task initialized\n");
    state = ControlState::RUNNING;
    
    // 精确计时
    auto next_time = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(static_cast<int>(controlPeriod * 1000000));
    
    while (state != ControlState::STOPPED && running.load()) {
        try {
            next_time += period;
            
            // ============================================================
            // 碰撞状态：电机停转，等待手动按键复位
            // ============================================================
            if (state == ControlState::COLLISION) {
                if (intervalCounter % 100 == 0) {
                    printf("[COLLISION] Press KEY%d to reset\n", collisionResetKey.load());
                }
                if (checkCollisionReset()) {
                    resetCollisionState();
                    state = ControlState::RUNNING;
                    leftPID.reset(); leftPID.setIntegral(0);
                    rightPID.reset(); rightPID.setIntegral(0);
                    if (angularVelocityControlEnabled.load())
                        angularVelocityPID.reset();
                    printf("Collision reset - resuming control\n");
                }
                intervalCounter++;
                std::this_thread::sleep_until(next_time);
                continue;
            }
            
            // ============================================================
            // 传感器采集
            // ============================================================
            
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
            
            // 一阶低通滤波
            if (lowPassFilterEnabled.load()) {
                leftSpeed = leftSpeedFilter.apply(leftSpeed);
                rightSpeed = rightSpeedFilter.apply(rightSpeed);
            }
            
            // 获取当前目标速度
            double currentLeftTarget = leftTargetSpeed.load();
            double currentRightTarget = rightTargetSpeed.load();

            if (speedPidIntegralResetRequested.exchange(false)) {
                double integralValue = speedPidIntegralValue.load();
                leftPID.setIntegral(integralValue);
                rightPID.setIntegral(integralValue);
            }
            
            // 读取IMU数据（每控制周期一次，碰撞检测与角速度PID共享）
            bool imuValid = false;
            imu_unit_data_t imuData{};
            if (imu != nullptr) {
                imuValid = imu->update_all_data();
                if (imuValid) {
                    imuData = imu->get_unit_data();
                }
            }
            
            // ============================================================
            // 碰撞检测（PID计算前：IMU冲击 + GPIO开关）
            // ============================================================
            if (collisionProtectEnabled.load()) {
                if (imuValid && detectImuCollision(imuData)) {
                    printf("COLLISION: IMU jerk detected (acc_x=%.2fg acc_y=%.2fg acc_z=%.2fg)\n",
                           imuData.acc_x, imuData.acc_y, imuData.acc_z);
                    handleCollision();
                    state = ControlState::COLLISION;
                    continue;
                }
                if (detectGpioCollision()) {
                    handleCollision();
                    state = ControlState::COLLISION;
                    continue;
                }
            }
            
            // ============================================================
            // 角速度控制 + 运动学分解
            // ============================================================
            bool angularControlEnabled = angularVelocityControlEnabled.load();
            if (angularControlEnabled && imuValid) {
                double actualAngularVelocity = imuData.gyro_z;
                
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
            
                // 一阶低通滤波
                if (lowPassFilterEnabled.load()) {
                    actualAngularVelocity = angularVelocityFilter.apply(actualAngularVelocity);
                }
                
                // 保存实际角速度，供外部读取
                lastActualAngularVelocity.store(actualAngularVelocity);
                
                // 获取目标角速度和基础速度
                double targetAngVel = targetAngularVelocity.load();
                double currentBaseSpeed = baseSpeed.load();
                
                // 角速度PID控制
                double angularVelocityError = targetAngVel - actualAngularVelocity;
                double angularControlOutput = angularVelocityPID.calculate(targetAngVel, actualAngularVelocity, controlPeriod);
                lastAngularVelocityPidOutput.store(angularControlOutput);
                
                // 将角速度控制输出转换为速度差
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
            } else if (angularControlEnabled && !imuValid) {
                printf("Warning: Failed to update IMU data\n");
            } else {
                currentLeftTarget = baseSpeed.load();
                currentRightTarget = baseSpeed.load();
                leftTargetSpeed.store(currentLeftTarget);
                rightTargetSpeed.store(currentRightTarget);
            }
            
            // 最小速度死区重分配（慢轮钳位到 minSpeed，丢速补到快轮，保持差速不变）
            double mMinSpeed = motorMinSpeed.load();
            if (mMinSpeed > 0.0) {
                auto [adjustedLeft, adjustedRight] = applySpeedRedistribution(currentLeftTarget, currentRightTarget);
                currentLeftTarget = adjustedLeft;
                currentRightTarget = adjustedRight;
            }
            
            // PID计算
            double leftOutput = leftPID.calculate(currentLeftTarget, leftSpeed);
            double rightOutput = rightPID.calculate(currentRightTarget, rightSpeed);
            
            // ============================================================
            // 堵转检测（PID计算后：有大输出但轮子不转）
            // ============================================================
            if (collisionProtectEnabled.load()) {
                if (detectStallCollision(leftOutput, rightOutput, leftSpeed, rightSpeed)) {
                    handleCollision();
                    state = ControlState::COLLISION;
                    continue;
                }
            }

            // 应用斜坡限制（如果启用）
            bool rampEnabled = rampLimitingEnabled.load();
            if (rampEnabled) {
                leftOutput = leftRampLimiter.apply(leftOutput, lastLeftOutput_, controlPeriod);
                rightOutput = rightRampLimiter.apply(rightOutput, lastRightOutput_, controlPeriod);
                lastLeftOutput_ = leftOutput;
                lastRightOutput_ = rightOutput;
            }
            
            std::array<float, 12> data = {
                (float)currentLeftTarget,           // 0
                (float)currentRightTarget,          // 1
                (float)leftSpeed,                   // 2
                (float)rightSpeed,                  // 3
                (float)leftOutput,                  // 4
                (float)rightOutput,                 // 5
                (float)leftPID.getIntegral(),       // 6
                (float)rightPID.getIntegral(),      // 7
            };
            send_udp_data("status", data.data(), data.size());

            // 应用控制输出
            motors->setSpeeds(static_cast<float>(leftOutput), static_cast<float>(rightOutput));
            
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
    } else if (state == ControlState::COLLISION) {
        printf("Motor control task stopped - collision detected.\n");
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
}

void MotorControlTask::setSpeedPidIntegral(double integral) {
    speedPidIntegralValue.store(integral);
    speedPidIntegralResetRequested.store(true);
}

double MotorControlTask::getBaseSpeed() const {
    return baseSpeed.load();
}

double MotorControlTask::getActualAngularVelocity() const {
    return lastActualAngularVelocity.load();
}

double MotorControlTask::getAngularVelocityPidOutput() const {
    return lastAngularVelocityPidOutput.load();
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

void MotorControlTask::setMotorMaxDuty(double duty) {
    if (duty <= 0.0 || duty > 100.0) {
        printf("Warning: Invalid motor max duty: %.3f\n", duty);
        return;
    }
    
    motorMaxDuty = duty;
    printf("Motor max duty updated to: %.3f\n", duty);
}

bool MotorControlTask::isValidAngularVelocity(double angularVelocity) const {
    return !std::isnan(angularVelocity) && !std::isinf(angularVelocity) && 
           angularVelocity >= -1000.0 && angularVelocity <= 1000.0;
}

std::pair<double, double> MotorControlTask::kinematicsDecomposition(double baseSpeed, double angularVelocityRad) const {
    // 计算速度差：Δv = ω * L / 2
    double deltaV = angularVelocityRad * wheelbase / 2.0;;
    
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

// ==================== LowPassFilter类实现 ====================

MotorControlTask::LowPassFilter::LowPassFilter(double tau, double dt)
    : tau_(tau)
    , dt_(dt)
    , alpha_(dt / (dt + tau))
    , lastValue_(0.0)
    , initialized_(false) {
}

double MotorControlTask::LowPassFilter::apply(double raw) {
    if (!initialized_) {
        lastValue_ = raw;
        initialized_ = true;
        return raw;
    }
    lastValue_ = alpha_ * raw + (1.0 - alpha_) * lastValue_;
    return lastValue_;
}

void MotorControlTask::LowPassFilter::reset(double value) {
    lastValue_ = value;
    initialized_ = true;
}

void MotorControlTask::LowPassFilter::setTimeConstant(double tau) {
    if (tau < 0.0) return;
    tau_ = tau;
    alpha_ = dt_ / (dt_ + tau_);
    if (alpha_ < 0.0) alpha_ = 0.0;
    if (alpha_ > 1.0) alpha_ = 1.0;
}

// ==================== RampLimiter类实现 ====================

MotorControlTask::RampLimiter::RampLimiter(double maxAcceleration, double maxDeceleration)
    : maxAcceleration_(maxAcceleration)
    , maxDeceleration_(maxDeceleration) {
}

double MotorControlTask::RampLimiter::apply(double target, double current, double dt) {
    if (dt <= 0.0) {
        return target;  // 无效的时间间隔，直接返回目标值
    }
    
    // 计算允许的最大变化量
    double maxChange = 0.0;
    if (target > current) {
        // 加速情况
        maxChange = maxAcceleration_ * dt;
    } else {
        // 减速情况
        maxChange = maxDeceleration_ * dt;
    }
    
    // 限制变化量
    double difference = target - current;
    if (std::abs(difference) <= maxChange) {
        // 变化量在允许范围内，直接到达目标值
        return target;
    } else {
        // 限制变化量
        if (difference > 0) {
            return current + maxChange;
        } else {
            return current - maxChange;
        }
    }
}

void MotorControlTask::RampLimiter::reset(double value) {
    // 这个实现不需要重置内部状态，因为RampLimiter是无状态的
    // 状态由外部管理（lastLeftOutput_和lastRightOutput_）
}

void MotorControlTask::RampLimiter::setLimits(double maxAcceleration, double maxDeceleration) {
    if (maxAcceleration > 0.0) {
        maxAcceleration_ = maxAcceleration;
    }
    if (maxDeceleration > 0.0) {
        maxDeceleration_ = maxDeceleration;
    }
}

std::pair<double, double> MotorControlTask::RampLimiter::getLimits() const {
    return {maxAcceleration_, maxDeceleration_};
}

void MotorControlTask::setMotorMinSpeed(double minSpeed) {
    if (minSpeed < 0.0) {
        printf("Warning: Invalid motor min speed: %.4f m/s\n", minSpeed);
        return;
    }
    motorMinSpeed.store(minSpeed);
    printf("Motor min speed set to: %.4f m/s\n", minSpeed);
}

double MotorControlTask::getMotorMinSpeed() const {
    return motorMinSpeed.load();
}

std::pair<double, double> MotorControlTask::applySpeedRedistribution(double leftTarget, double rightTarget) const {
    double minSpeed = motorMinSpeed.load();
    if (minSpeed <= 0.0) {
        return {leftTarget, rightTarget};
    }
    
    double absL = std::abs(leftTarget);
    double absR = std::abs(rightTarget);
    
    // 两轮均低于最小速度 → 停车
    if (absL < minSpeed && absR < minSpeed) {
        return {0.0, 0.0};
    }
    
    // 两轮均在最小速度以上 → 无需调整
    if (absL >= minSpeed && absR >= minSpeed) {
        return {leftTarget, rightTarget};
    }
    
    // 原始转速差
    double originalDiff = rightTarget - leftTarget;
    
    // 慢轮钳位到最小速度，丢速叠加到快轮以保持转速差
    if (absL < minSpeed) {
        double clampedLeft = std::copysign(minSpeed, leftTarget);
        double compensatedRight = clampedLeft + originalDiff;
        return {clampedLeft, compensatedRight};
    }
    
    // absR < minSpeed
    double clampedRight = std::copysign(minSpeed, rightTarget);
    double compensatedLeft = clampedRight - originalDiff;
    return {compensatedLeft, clampedRight};
}

void MotorControlTask::setAngularVelocityParams(const Control::PID::Parameters& params) {
    angularVelocityParams = params;
    printf("Angular velocity PID params updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
           params.Kp, params.Ki, params.Kd);
}

// ==================== 低通滤波控制相关方法实现 ====================

void MotorControlTask::enableLowPassFilter(bool enable) {
    bool wasEnabled = lowPassFilterEnabled.load();
    lowPassFilterEnabled.store(enable);
    
    if (enable && !wasEnabled) {
        leftSpeedFilter.reset(leftEncoder->readSpeed());
        rightSpeedFilter.reset(rightEncoder->readSpeed());
        if (imu != nullptr) {
            imu->update_all_data();
            angularVelocityFilter.reset(imu->get_unit_data().gyro_z);
        }
        printf("Low-pass filter enabled\n");
    } else if (!enable && wasEnabled) {
        printf("Low-pass filter disabled\n");
    }
}

bool MotorControlTask::isLowPassFilterEnabled() const {
    return lowPassFilterEnabled.load();
}

void MotorControlTask::setSpeedFilterTimeConstant(double tau) {
    if (tau < 0.0) {
        printf("Warning: Invalid speed filter time constant: %.4f\n", tau);
        return;
    }
    leftSpeedFilter.setTimeConstant(tau);
    rightSpeedFilter.setTimeConstant(tau);
    speedFilterTau.store(tau);
    printf("Speed low-pass filter time constant set to: %.4f s (alpha=%.4f)\n", 
           tau, leftSpeedFilter.getAlpha());
}

void MotorControlTask::setAngularFilterTimeConstant(double tau) {
    if (tau < 0.0) {
        printf("Warning: Invalid angular filter time constant: %.4f\n", tau);
        return;
    }
    angularVelocityFilter.setTimeConstant(tau);
    angularFilterTau.store(tau);
    printf("Angular velocity low-pass filter time constant set to: %.4f s (alpha=%.4f)\n",
           tau, angularVelocityFilter.getAlpha());
}

void MotorControlTask::resetFilters(double leftValue, double rightValue, double angularValue) {
    leftSpeedFilter.reset(leftValue);
    rightSpeedFilter.reset(rightValue);
    angularVelocityFilter.reset(angularValue);
}

// ==================== 斜坡控制相关方法实现 ====================

void MotorControlTask::enableRampLimiting(bool enable) {
    bool wasEnabled = rampLimitingEnabled.load();
    rampLimitingEnabled.store(enable);
    
    if (enable && !wasEnabled) {
        printf("Ramp limiting enabled\n");
        // 重置上一次的输出值为当前电机速度（如果有的话）
        // 这里可以添加获取当前速度的逻辑，但为了简单起见，我们重置为0
        lastLeftOutput_ = 0.0;
        lastRightOutput_ = 0.0;
    } else if (!enable && wasEnabled) {
        printf("Ramp limiting disabled\n");
    }
}

bool MotorControlTask::isRampLimitingEnabled() const {
    return rampLimitingEnabled.load();
}

void MotorControlTask::setRampLimits(double maxAcceleration, double maxDeceleration) {
    if (maxAcceleration <= 0.0 || maxDeceleration <= 0.0) {
        printf("Warning: Invalid ramp limits - acceleration: %.3f, deceleration: %.3f\n", 
               maxAcceleration, maxDeceleration);
        return;
    }
    
    leftRampLimiter.setLimits(maxAcceleration, maxDeceleration);
    rightRampLimiter.setLimits(maxAcceleration, maxDeceleration);
    
    printf("Ramp limits updated - acceleration: %.3f, deceleration: %.3f (duty/s)\n", 
           maxAcceleration, maxDeceleration);
}

std::pair<double, double> MotorControlTask::getRampLimits() const {
    return leftRampLimiter.getLimits();  // 左右限制器参数相同
}

void MotorControlTask::resetRampLimiters(double leftValue, double rightValue) {
    leftRampLimiter.reset(leftValue);
    rightRampLimiter.reset(rightValue);
    lastLeftOutput_ = leftValue;
    lastRightOutput_ = rightValue;
    
    printf("Ramp limiters reset - left: %.3f, right: %.3f\n", leftValue, rightValue);
}

// ==================== 碰撞保护方法实现 ====================

void MotorControlTask::enableCollisionProtection(bool enable) {
    collisionProtectEnabled.store(enable);
    printf("Collision protection %s\n", enable ? "enabled" : "disabled");
}

bool MotorControlTask::isCollisionProtectionEnabled() const {
    return collisionProtectEnabled.load();
}

void MotorControlTask::setCollisionImuJerkThreshold(double threshold) {
    if (threshold <= 0.0) {
        printf("Warning: Invalid IMU jerk threshold: %.2f g\n", threshold);
        return;
    }
    collisionImuJerkThreshold.store(threshold);
    printf("Collision IMU jerk threshold set to: %.2f g\n", threshold);
}

void MotorControlTask::setCollisionStallThresholds(double dutyThreshold, double speedThreshold, int cycles) {
    if (dutyThreshold < 0.0 || dutyThreshold > 1.0 || speedThreshold < 0.0 || cycles < 1) {
        printf("Warning: Invalid stall thresholds\n");
        return;
    }
    collisionStallDutyThreshold.store(dutyThreshold);
    collisionStallSpeedThreshold.store(speedThreshold);
    collisionStallCycles.store(cycles);
    stallCounter_ = 0;
    printf("Collision stall: duty>%.2f, speed<%.3f m/s, cycles=%d\n",
           dutyThreshold, speedThreshold, cycles);
}

void MotorControlTask::setCollisionKeyConfig(int resetKeyIndex, int bumperKeyIndex) {
    collisionResetKey.store(resetKeyIndex);
    collisionBumperKey.store(bumperKeyIndex);
    printf("Collision keys: reset=KEY%d, bumper=KEY%d\n", resetKeyIndex, bumperKeyIndex);
}

void MotorControlTask::configureCollisionGpio(const std::array<std::string, 4>& keyPaths) {
    collisionKeyPaths_ = keyPaths;
    printf("Collision GPIO paths configured\n");
}

void MotorControlTask::resetCollisionState() {
    collisionDetected.store(false);
    stallCounter_ = 0;
    printf("Collision state reset\n");
}

bool MotorControlTask::isCollisionDetected() const {
    return collisionDetected.load();
}

bool MotorControlTask::detectImuCollision(const imu_unit_data_t& imuData) const {
    double threshold = collisionImuJerkThreshold.load();
    if (threshold <= 0.0) return false;
    
    double horizontalAcc = std::sqrt(imuData.acc_x * imuData.acc_x +
                                      imuData.acc_y * imuData.acc_y);
    return horizontalAcc > threshold;
}

bool MotorControlTask::detectStallCollision(double leftOutput, double rightOutput,
                                             double leftSpeed, double rightSpeed) {
    double dutyThr = collisionStallDutyThreshold.load();
    double speedThr = collisionStallSpeedThreshold.load();
    int cycles = collisionStallCycles.load();
    
    if (dutyThr <= 0.0 || cycles <= 0) return false;
    
    double absLeftOut = std::abs(leftOutput);
    double absRightOut = std::abs(rightOutput);
    double absLeftSpeed = std::abs(leftSpeed);
    double absRightSpeed = std::abs(rightSpeed);
    
    bool stalld = (absLeftOut > dutyThr && absLeftSpeed < speedThr) ||
                  (absRightOut > dutyThr && absRightSpeed < speedThr);
    
    if (stalld) {
        stallCounter_++;
        if (stallCounter_ >= cycles) {
            printf("Stall detected: L_out=%.3f L_spd=%.3f R_out=%.3f R_spd=%.3f (%d cycles)\n",
                   leftOutput, leftSpeed, rightOutput, rightSpeed, stallCounter_);
            stallCounter_ = 0;
            return true;
        }
    } else {
        stallCounter_ = 0;
    }
    return false;
}

bool MotorControlTask::detectGpioCollision() const {
    int bumperKey = collisionBumperKey.load();
    if (bumperKey < 0 || bumperKey >= 4) return false;
    
    const char* path = getKeyPath(bumperKey);
    if (path == nullptr) return false;
    
    uint8_t level = gpio_get_level(path);
    if (level == 0) {
        printf("GPIO collision detected on KEY%d (level=%d)\n", bumperKey, level);
        return true;
    }
    return false;
}

void MotorControlTask::handleCollision() {
    collisionDetected.store(true);
    motors->setSpeeds(0.0f, 0.0f);
    printf("*** COLLISION PROTECTION ACTIVATED - Motors stopped ***\n");
}

bool MotorControlTask::checkCollisionReset() const {
    int resetKey = collisionResetKey.load();
    if (resetKey < 0 || resetKey >= 4) return true;
    
    const char* path = getKeyPath(resetKey);
    if (path == nullptr) return true;
    
    uint8_t level = gpio_get_level(path);
    return (level == 0);
}

const char* MotorControlTask::getKeyPath(int keyIndex) const {
    if (keyIndex < 0 || keyIndex >= 4) return nullptr;
    if (collisionKeyPaths_[keyIndex].empty()) return nullptr;
    return collisionKeyPaths_[keyIndex].c_str();
}
