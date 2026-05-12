#include "MotorControlTask.hpp"
#include <pthread.h>
#include <cmath>
#include <cstring>
#include <algorithm>

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
    , rampLimitingEnabled(false)          // 默认禁用斜坡限制
    , leftParams(leftParams)
    , rightParams(rightParams)
    , leftPID(leftParams)
    , rightPID(rightParams)
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
    , leftRampLimiter(0.8, 0.2)          // 默认加速度=0.5，减速度=0.5
    , rightRampLimiter(0.8, 0.2)         // 默认加速度=0.5，减速度=0.5
    , lastLeftOutput_(0.0)
    , lastRightOutput_(0.0)
    , startupRampEnabled(false)          // 默认禁用开机缓升
    , startupRampDurationMs(1000)        // 默认缓升时间1秒
    , startupRampInitialized(false) {
    
    if (!motors || !leftEncoder || !rightEncoder) {
        throw std::invalid_argument("MotorControlTask: Null pointer provided");
    }
    
    if (controlPeriod <= 0.0) {
        throw std::invalid_argument("MotorControlTask: Invalid control period");
    }
    
    // 初始化角速度PID参数
    angularVelocityParams.Kp = 1.0;      // 适中的反应速度
    angularVelocityParams.Ki = 0.5;      // 较温和的误差消除
    angularVelocityParams.Kd = 0.05;     // 微弱的阻尼，防止超调
    
    angularVelocityParams.limitP = 80.0;  // 允许比例项产生较大的修正
    angularVelocityParams.limitI = 100.0;  // 限制积分项，防止严重超调
    angularVelocityParams.limitD = 30.0;  // 限制微分震荡
    
    // 总输出限幅：
    // 如果偏差很大，允许 PID 在目标值基础上最多补偿 ±50°/s
    angularVelocityParams.limitOutput = 120.0; 
    angularVelocityParams.limitIMin = -120.0;
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
    
    // printf("Target speeds updated - Left: %.3f m/s, Right: %.3f m/s\n", leftSpeed, rightSpeed);
}

void MotorControlTask::setLeftTargetSpeed(double speed) {
    leftTargetSpeed.store(speed);
   // printf("Left target speed updated to: %.3f m/s\n", speed);
}

void MotorControlTask::setRightTargetSpeed(double speed) {
    rightTargetSpeed.store(speed);
   // printf("Right target speed updated to: %.3f m/s\n", speed);
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
    
    // PID 预热阶段：空跑 10 帧，让 PID 建立微分历史，避免第一帧微分冲击导致速度暴涨
    printf("PID warmup: running %d cycles with zero target...\n", 10);
    for (int warmup = 0; warmup < 10; warmup++) {
        next_time += period;
        double leftSpeed = leftEncoder->readSpeed();
        double rightSpeed = rightEncoder->readSpeed();
        // 用 0 目标跑 PID，让 m_prevError 和 m_integral 有初始值
        leftPID.calculate(0.0, leftSpeed, controlPeriod);
        rightPID.calculate(0.0, rightSpeed, controlPeriod);
        std::this_thread::sleep_until(next_time);
    }
    printf("PID warmup done.\n");
    
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
            
            // ==================== 开机速度缓升 ====================
            // 原理：在缓升持续时间内，将目标速度乘以一个从0线性增加到1的系数
            // 这样即使外部突然设置一个很大的目标速度，实际给到PID的目标也是从0逐渐增大
            bool rampUp = startupRampEnabled.load();
            if (rampUp) {
                // 首次进入缓升时记录开始时间
                if (!startupRampInitialized) {
                    startupStartTime = std::chrono::steady_clock::now();
                    startupRampInitialized = true;
                    printf("[StartupRamp] 开机速度缓升已激活，持续时间 %d ms\n", startupRampDurationMs.load());
                }
                
                int durationMs = startupRampDurationMs.load();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startupStartTime).count();
                
                double rampFactor = 1.0;
                if (durationMs > 0 && elapsed < durationMs) {
                    rampFactor = static_cast<double>(elapsed) / static_cast<double>(durationMs);
                    rampFactor = std::clamp(rampFactor, 0.0, 1.0);
                } else {
                    // 缓升结束，自动禁用
                    startupRampEnabled.store(false);
                    startupRampInitialized = false;
                    printf("[StartupRamp] 开机速度缓升完成 (elapsed=%ld ms)\n", elapsed);
                }
                
                // 应用缓升系数：目标速度乘以 rampFactor
                currentLeftTarget *= rampFactor;
                currentRightTarget *= rampFactor;
            } else {
                startupRampInitialized = false;
            }
            
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
                // printf("当前角速度: %.2f, 误差: %.2f, 角速度控制输出: %.2f", 
                //     actualAngularVelocity, angularVelocityError, angularControlOutput);
                
                // 更新原子变量（用于显示）
                leftTargetSpeed.store(leftTarget);
                rightTargetSpeed.store(rightTarget);
            }
            
            // PID计算（输出单位：m/s）
            double leftOutput = leftPID.calculate(currentLeftTarget, leftSpeed);
            double rightOutput = rightPID.calculate(currentRightTarget, rightSpeed);

            // 应用斜坡限制（如果启用）
            bool rampEnabled = rampLimitingEnabled.load();
            if (rampEnabled) {
                leftOutput = leftRampLimiter.apply(leftOutput, lastLeftOutput_, controlPeriod);
                rightOutput = rightRampLimiter.apply(rightOutput, lastRightOutput_, controlPeriod);
                lastLeftOutput_ = leftOutput;
                lastRightOutput_ = rightOutput;
            }

            // printf(",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            //     currentLeftTarget,currentRightTarget,leftSpeed,rightSpeed,leftOutput,rightOutput);
            
            // // 添加斜坡状态输出
            // if (rampEnabled) {
            //     printf(",RAMP");
            // } else {
            //     printf(",NORAMP");
            // }
            // printf("\n");
            
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

void MotorControlTask::steerByCenterlineError(double trackCenterX,
                                               double imageCenterX,
                                               double baseSpeedMps,
                                               double kpDegPerPixel,
                                               double maxYawRateDeg)
{
    const double errorPx = trackCenterX - imageCenterX;
    const double targetYaw = std::clamp(errorPx * kpDegPerPixel, -maxYawRateDeg, maxYawRateDeg);
    const double clampedBaseSpeed = std::clamp(baseSpeedMps, -10.0, 10.0);

    setTargetAngularVelocity(targetYaw);//
    setBaseSpeed(clampedBaseSpeed);

    const auto [leftSpeed, rightSpeed] = kinematicsDecomposition(clampedBaseSpeed, degToRad(targetYaw));
    if (!isAngularVelocityControlEnabled()) {
        setTargetSpeed(leftSpeed, rightSpeed);
    }

    fprintf(stderr,
            "[MotorControlTask] steer error=%.2f px, targetYaw=%.2f deg/s, left=%.3f m/s, right=%.3f m/s\n",
            errorPx, targetYaw, leftSpeed, rightSpeed);
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

// ==================== 开机速度缓升相关方法实现 ====================

void MotorControlTask::enableStartupRamp(bool enable) {
    startupRampEnabled.store(enable);
    if (enable) {
        startupRampInitialized = false;  // 下次进入 run() 循环时会重新初始化
        printf("[StartupRamp] 开机速度缓升已%s\n", enable ? "启用" : "禁用");
    } else {
        printf("[StartupRamp] 开机速度缓升已禁用\n");
    }
}

bool MotorControlTask::isStartupRampEnabled() const {
    return startupRampEnabled.load();
}

void MotorControlTask::setStartupRampDuration(int durationMs) {
    if (durationMs > 0) {
        startupRampDurationMs.store(durationMs);
        printf("[StartupRamp] 缓升时间设置为 %d ms\n", durationMs);
    } else {
        printf("[StartupRamp] 无效的缓升时间: %d ms，使用默认值 1000 ms\n", durationMs);
        startupRampDurationMs.store(1000);
    }
}
