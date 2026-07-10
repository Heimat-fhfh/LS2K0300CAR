#include "control/MotorControlTask.hpp"
#include "control/MotorCommandAllocator.hpp"
#include <pthread.h>
#include <cmath>
#include <cstring>
#include <algorithm>

MotorControlTask::MotorControlTask(
    const Control::PID::Parameters& diffOuterParams,
    const Control::PID::Parameters& diffInnerParams,
    const Control::IncrementalPID::Parameters& speedIncrParams,
    DualMotorController* motors,
    Encoder* leftEncoder,
    Encoder* rightEncoder,
    IMUDevice* imu,
    double controlPeriod
)
    : steerError(0.0)
    , targetSpeed(0.0)
    , diffOuterParams(diffOuterParams)
    , diffInnerParams(diffInnerParams)
    , speedIncrParams(speedIncrParams)
    , motors(motors)
    , leftEncoder(leftEncoder)
    , rightEncoder(rightEncoder)
    , imu(imu)
    , motorMaxDuty(50.0)
    , collisionProtectEnabled(false)
    , collisionImuJerkThreshold(3.0)
    , collisionStallDutyThreshold(0.2)
    , collisionStallSpeedThreshold(0.01)
    , collisionStallCycles(20)
    , collisionResetKey(0)
    , collisionBumperKey(-1)
    , collisionDetected(false)
    , stallCounter_(0)
    , lowPassFilterEnabled(true)
    , speedFilterTau(0.02)
    , angularFilterTau(0.02)
    , steerFilterTau(0.02)
    , leftSpeedFilter(0.02, controlPeriod)
    , rightSpeedFilter(0.02, controlPeriod)
    , angularVelocityFilter(0.02, controlPeriod)
    , steerErrorFilter(0.02, controlPeriod)
    , rampControlEnabled(true)
    , leftRampLimiter(50.0, 100.0, controlPeriod)
    , rightRampLimiter(50.0, 100.0, controlPeriod)
    , diffOutputRampEnabled(true)
    , diffOutputRampLimiter(15.0, 30.0, controlPeriod)
    , pwmDeadZoneLeft_(0.001)
    , pwmDeadZoneRight_(0.001)
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

    pthread_t native_handle = workerThread->native_handle();
    struct sched_param param;
    param.sched_priority = rtPriority;

    if (pthread_setschedparam(native_handle, rtPolicy, &param) != 0) {
        printf("Warning: Failed to set realtime priority for motor control thread\n");
    }

    printf("[电机] 控制任务启动, 周期 %.3fs\n", controlPeriod);
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

bool MotorControlTask::isValidAngularVelocity(double angularVelocity) const {
    return !std::isnan(angularVelocity) && !std::isinf(angularVelocity) &&
           angularVelocity >= -20.0 && angularVelocity <= 20.0;
}

// ==================== 串级差速环 API 实现 ====================

void MotorControlTask::setSteerError(double error) {
    steerError.store(std::max(-1.0, std::min(1.0, error)));
}

void MotorControlTask::setTargetSpeed(double speed) {
    if (!isValidSpeed(speed)) {
        printf("Warning: Invalid target speed: %.3f m/s\n", speed);
        return;
    }
    targetSpeed.store(speed);
}

double MotorControlTask::getTargetSpeed() const {
    return targetSpeed.load();
}

void MotorControlTask::setDiffOuterParams(const Control::PID::Parameters& params) {
    const_cast<Control::PID::Parameters&>(diffOuterParams) = params;
    printf("Diff outer PD params updated: Kp=%.3f, Kd=%.3f\n",
           params.Kp, params.Kd);
}

void MotorControlTask::setDiffInnerParams(const Control::PID::Parameters& params) {
    const_cast<Control::PID::Parameters&>(diffInnerParams) = params;
    printf("Diff inner PI params updated: Kp=%.3f, Ki=%.3f\n",
           params.Kp, params.Ki);
}

void MotorControlTask::setSpeedIncrementalParams(const Control::IncrementalPID::Parameters& params) {
    const_cast<Control::IncrementalPID::Parameters&>(speedIncrParams) = params;
    printf("Speed incremental PID params updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\n",
           params.Kp, params.Ki, params.Kd);
}

void MotorControlTask::setMotorMaxDuty(double duty) {
    if (duty <= 0.0 || duty > 100.0) {
        printf("Warning: Invalid motor max duty: %.3f\n", duty);
        return;
    }
    motorMaxDuty = duty;

}

void MotorControlTask::setCurvatureSpeedGain(double gain) {
    if (gain < 0.0 || gain > 1.0) {
        printf("Warning: Invalid curvature speed gain: %.3f, clamped to [0,1]\n", gain);
        gain = std::max(0.0, std::min(1.0, gain));
    }
    curvatureSpeedGain.store(gain);
    printf("Curvature speed gain set: %.3f\n", gain);
}

double MotorControlTask::getCurvatureSpeedGain() const {
    return curvatureSpeedGain.load();
}

void MotorControlTask::setCurvatureSpeedMin(double minSpeed) {
    if (minSpeed < 0.0 || minSpeed > 1.0) {
        printf("Warning: Invalid curvature speed min: %.3f, clamped to [0,1]\n", minSpeed);
        minSpeed = std::max(0.0, std::min(1.0, minSpeed));
    }
    curvatureSpeedMin.store(minSpeed);
    printf("Curvature speed min set: %.3f\n", minSpeed);
}

double MotorControlTask::getCurvatureSpeedMin() const {
    return curvatureSpeedMin.load();
}

void MotorControlTask::setDiffOuterKp2(double kp2) {
    if (kp2 < 0.0) {
        printf("Warning: Invalid diff outer KP2: %.3f, clamped to 0\n", kp2);
        kp2 = 0.0;
    }
    diffOuterKp2_.store(kp2);
    printf("Diff outer KP2 set: %.3f\n", kp2);
}

void MotorControlTask::setDiffInnerGkd(double gkd) {
    diffInnerGkd_.store(gkd);
    printf("Diff inner GKD set: %.3f\n", gkd);
}

void MotorControlTask::setDiffInnerGkdLimit(double limit) {
    if (limit < 0.0) {
        printf("Warning: Invalid GKD limit: %.3f, clamped to 0\n", limit);
        limit = 0.0;
    }
    diffInnerGkdLimit_.store(limit);
    printf("Diff inner GKD limit set: %.3f\n", limit);
}

void MotorControlTask::setDeadZones(double leftDeadZone, double rightDeadZone) {
    if (leftDeadZone < 0.0) leftDeadZone = 0.0;
    if (leftDeadZone >= 1.0) leftDeadZone = 0.999;
    if (rightDeadZone < 0.0) rightDeadZone = 0.0;
    if (rightDeadZone >= 1.0) rightDeadZone = 0.999;
    pwmDeadZoneLeft_.store(leftDeadZone);
    pwmDeadZoneRight_.store(rightDeadZone);
    printf("Dead zones set: left=%.4f right=%.4f\n", leftDeadZone, rightDeadZone);
}

double MotorControlTask::applyDeadZoneRemap(double speed, double deadZone) {
    double absSpeed = std::abs(speed);
    if (absSpeed <= deadZone) {
        return 0.0;
    }
    double remapped = (absSpeed - deadZone) / (1.0 - deadZone);
    return std::copysign(remapped, speed);
}

// ==================== 主控制循环 ====================

void MotorControlTask::run() {
    enum class ControlState {
        INIT,
        RUNNING,
        COLLISION,
        ERROR,
        STOPPED
    };

    ControlState state = ControlState::INIT;

    Control::PID diffOuterPID(diffOuterParams);
    Control::PID diffInnerPID(diffInnerParams);
    Control::IncrementalPID speedPID(speedIncrParams);

    motors->setMaxDutyLimits(static_cast<float>(motorMaxDuty));

    const int maxErrorCount = 10;
    int leftErrorCount = 0;
    int rightErrorCount = 0;
    int imuErrorCount = 0;
    double lastValidLeftSpeed = 0.0;
    double lastValidRightSpeed = 0.0;
    double lastValidAngularVelocity = 0.0;

    int intervalCounter = 0;


    leftRampLimiter.reset(0.0);
    rightRampLimiter.reset(0.0);
    diffOutputRampLimiter.reset(0.0);
    state = ControlState::RUNNING;

    auto next_time = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(static_cast<int>(controlPeriod * 1000000));

    while (state != ControlState::STOPPED && running.load()) {
        try {
            next_time += period;

            // ============================================================
            // 碰撞状态
            // ============================================================
            if (state == ControlState::COLLISION) {
                if (intervalCounter % 100 == 0) {
                    printf("[COLLISION] Press KEY%d to reset\n", collisionResetKey.load());
                }
                if (checkCollisionReset()) {
                    resetCollisionState();
                    leftRampLimiter.reset(0.0);
                    rightRampLimiter.reset(0.0);
                    diffOutputRampLimiter.reset(0.0);
                    state = ControlState::RUNNING;
                    diffOuterPID.reset();
                    diffInnerPID.reset();
                    speedPID.reset();
                    printf("Collision reset - resuming control\n");
                }
                intervalCounter++;
                std::this_thread::sleep_until(next_time);
                continue;
            }

            // ============================================================
            // 紧急停机状态（出界保护）
            // ============================================================
            if (emergencyStopActive.load()) {
                motors->setSpeeds(0.0f, 0.0f);
                if (pidResetRequested.load()) {
                    diffOuterPID.reset();
                    diffInnerPID.reset();
                    speedPID.reset();
                    leftRampLimiter.reset(0.0);
                    rightRampLimiter.reset(0.0);
                    diffOutputRampLimiter.reset(0.0);
                    steerErrorFilter.reset(0.0);
                    pidResetRequested.store(false);
                }
                intervalCounter++;
                std::this_thread::sleep_until(next_time);
                continue;
            }

            // PID 复位请求（恢复时清除积分）
            if (pidResetRequested.load()) {
                diffOuterPID.reset();
                diffInnerPID.reset();
                speedPID.reset();
                leftRampLimiter.reset(0.0);
                rightRampLimiter.reset(0.0);
                diffOutputRampLimiter.reset(0.0);
                steerErrorFilter.reset(0.0);
                pidResetRequested.store(false);
                printf("PID controllers reset\n");
            }

            // ============================================================
            // 1. 传感器采集
            // ============================================================
            double leftSpeed = leftEncoder->readSpeed();
            double rightSpeed = rightEncoder->readSpeed();

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

            // ============================================================
            // 2. 低通滤波 (编码器速度)
            // ============================================================
            if (lowPassFilterEnabled.load()) {
                leftSpeed = leftSpeedFilter.apply(leftSpeed);
                rightSpeed = rightSpeedFilter.apply(rightSpeed);
            }

            // ============================================================
            // 3. IMU 数据读取
            // ============================================================
            bool imuValid = false;
            imu_unit_data_t imuData{};
            double gyroZ = 0.0;
            if (imu != nullptr) {
                imuValid = imu->update_all_data();
                if (imuValid) {
                    imuData = imu->get_unit_data();
                    gyroZ = imuData.gyro_z;
                }
            }

            // ============================================================
            // 4. 碰撞检测
            // ============================================================
            if (collisionProtectEnabled.load()) {
                if (imuValid && detectImuCollision(imuData)) {
                    printf("碰撞: 检测到IMU抖动 (acc_x=%.2fg acc_y=%.2fg acc_z=%.2fg)\n",
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
            // 5. 读取原子变量 (视觉反馈)
            // ============================================================
            double currentError = steerError.load();
            double desiredSpeed = targetSpeed.load();

            // ============================================================
            // 6. 低通滤波 (归一化偏差 + 陀螺仪)
            // ============================================================
            if (lowPassFilterEnabled.load()) {
                currentError = steerErrorFilter.apply(currentError);
            }

            // ============================================================
            // 6.5 曲率自适应降速：弯越大，目标速度越低
            //     实现摩擦圆约束：横向力大时降低纵向力，防止轮胎突破抓地力极限
            // ============================================================
            {
                double cg = curvatureSpeedGain.load();
                if (cg > 0.0) {
                    double turnFactor = 1.0 - cg * std::abs(currentError);
                    double minFactor = curvatureSpeedMin.load();
                    if (turnFactor < minFactor) turnFactor = minFactor;
                    desiredSpeed *= turnFactor;
                }
            }

            // ============================================================
            // 7. 低通滤波 (陀螺仪)
            // ============================================================
            if (imuValid) {
                if (!isValidAngularVelocity(gyroZ)) {
                    imuErrorCount++;
                    printf("Warning: Invalid angular velocity: %.3f rad/s (error %d/%d)\n",
                           gyroZ, imuErrorCount, maxErrorCount);
                    gyroZ = lastValidAngularVelocity;
                    if (imuErrorCount >= maxErrorCount) {
                        printf("ERROR: Too many invalid angular velocity readings\n");
                        state = ControlState::ERROR;
                        taskError = true;
                        break;
                    }
                } else {
                    imuErrorCount = 0;
                    lastValidAngularVelocity = gyroZ;
                }

                if (lowPassFilterEnabled.load()) {
                    gyroZ = angularVelocityFilter.apply(gyroZ);
                }
            }

            // ============================================================
            // 8. 外环差速PD
            //    setpoint=0, feedback=currentError (归一化偏差)
            // ============================================================
            double desiredDiffSpeed = diffOuterPID.calculate(0.0, currentError, controlPeriod);

            {
                double kp2 = diffOuterKp2_.load();
                if (kp2 != 0.0) {
                    double outerError = -currentError;
                    desiredDiffSpeed += outerError * std::abs(outerError) * kp2;
                    desiredDiffSpeed = std::max(-diffOuterParams.limitOutput,
                                                 std::min(diffOuterParams.limitOutput, desiredDiffSpeed));
                }
            }

            if (diffOutputRampEnabled.load()) {
                desiredDiffSpeed = diffOutputRampLimiter.apply(desiredDiffSpeed);
            }

            // ============================================================
            // 9. 内环角速度PI
            //    setpoint=desiredDiffSpeed, feedback=gyroZ
            // ============================================================
            double diffOutput;
            if (imuValid) {
                diffOutput = diffInnerPID.calculate(desiredDiffSpeed, gyroZ, controlPeriod);

                double gkd = diffInnerGkd_.load();
                if (gkd != 0.0) {
                    double gkdTerm = gyroZ * gkd;
                    double gkdLimit = diffInnerGkdLimit_.load();
                    gkdTerm = std::max(-gkdLimit, std::min(gkdLimit, gkdTerm));
                    diffOutput += gkdTerm;
                }
            } else {
                diffOutput = desiredDiffSpeed;
            }

            // ============================================================
            // 10. 速度环增量式PID
            //    Δu = Kp*(e_k - e_{k-1}) + Ki*e_k*dt + Kd*(e_k - 2*e_{k-1} + e_{k-2})/dt
            //    u_k = clamp(u_{k-1} + Δu, ±limitOutput)
            // ============================================================
            double avgSpeed = (leftSpeed + rightSpeed) / 2.0;
            double speedOutput = speedPID.calculate(desiredSpeed, avgSpeed, controlPeriod);

            
            // ============================================================
            // 11. 组合输出
            //     左电机 = 速度输出 - 差速输出
            //     右电机 = 速度输出 + 差速输出
            // ============================================================
            // double leftCmd = std::max(0.0, speedOutput - diffOutput);
            // double rightCmd = std::max(0.0, speedOutput + diffOutput);
            double leftCmd = speedOutput - diffOutput;
            double rightCmd = speedOutput + diffOutput;

            // ============================================================
            // 11. 斜坡控制 (限制速度输出变化率)
            // ============================================================
            if (rampControlEnabled.load()) {
                leftCmd = leftRampLimiter.apply(leftCmd);
                rightCmd = rightRampLimiter.apply(rightCmd);
            }

            leftCmd = applyDeadZoneRemap(leftCmd, pwmDeadZoneLeft_.load());
            rightCmd = applyDeadZoneRemap(rightCmd, pwmDeadZoneRight_.load());

            // UDP遥测
            std::array<float, 10> data = {
                (float)desiredSpeed,           // 0: 期望速度
                (float)avgSpeed,               // 1: 平均速度
                (float)currentError,           // 2: 归一化偏差
                (float)desiredDiffSpeed,       // 3: 外环PD输出(期望差速)
                (float)gyroZ,                  // 4: 陀螺仪角速度
                (float)diffOutput,             // 5: 内环PI输出(最终差速)
                (float)speedOutput,            // 6: 速度环输出
                (float)leftCmd,                // 7: 左电机命令
                (float)rightCmd,               // 8: 右电机命令
                (float)diffInnerPID.getIntegral() // 9: 内环积分
            };
            send_udp_data("status", data.data(), data.size());

            // ============================================================
            // 12. 输出到电机
            // ============================================================
            motors->setSpeeds(static_cast<float>(leftCmd), static_cast<float>(rightCmd));

            intervalCounter++;

        } catch (const std::exception& e) {
            printf("Exception in control loop: %s\n", e.what());
            motors->setSpeeds(0.0f, 0.0f);
            leftRampLimiter.reset(0.0);
            rightRampLimiter.reset(0.0);
            diffOutputRampLimiter.reset(0.0);
            state = ControlState::ERROR;
            taskError = true;
            break;
        }

        std::this_thread::sleep_until(next_time);
    }

    motors->setSpeeds(0.0f, 0.0f);

    if (state == ControlState::ERROR) {
        printf("Motor control task terminated due to errors.\n");
    } else if (state == ControlState::COLLISION) {
        printf("Motor control task stopped - collision detected.\n");
    } else {
        printf("Motor control task stopped normally.\n");
    }
}

// ==================== 低通滤波实现 ====================

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

// ==================== 斜坡限制器实现 ====================

MotorControlTask::RampLimiter::RampLimiter(double accelRate, double decelRate, double dt)
    : accelRate_(accelRate)
    , decelRate_(decelRate)
    , dt_(dt)
    , lastValue_(0.0)
    , initialized_(false) {
}

double MotorControlTask::RampLimiter::apply(double target) {
    if (!initialized_) {
        lastValue_ = target;
        initialized_ = true;
        return target;
    }

    double delta = target - lastValue_;
    double maxDelta;

    if (delta > 0.0) {
        maxDelta = accelRate_ * dt_;
        if (delta > maxDelta) delta = maxDelta;
    } else {
        maxDelta = decelRate_ * dt_;
        if (delta < -maxDelta) delta = -maxDelta;
    }

    lastValue_ = lastValue_ + delta;
    return lastValue_;
}

void MotorControlTask::RampLimiter::reset(double value) {
    lastValue_ = value;
    initialized_ = true;
}

void MotorControlTask::RampLimiter::setAccelRate(double rate) {
    if (rate > 0.0) accelRate_ = rate;
}

void MotorControlTask::RampLimiter::setDecelRate(double rate) {
    if (rate > 0.0) decelRate_ = rate;
}

// ==================== 低通滤波控制 API ====================

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

}

void MotorControlTask::setAngularFilterTimeConstant(double tau) {
    if (tau < 0.0) {
        printf("Warning: Invalid angular filter time constant: %.4f\n", tau);
        return;
    }
    angularVelocityFilter.setTimeConstant(tau);
    angularFilterTau.store(tau);

}

void MotorControlTask::resetFilters(double leftValue, double rightValue, double angularValue) {
    leftSpeedFilter.reset(leftValue);
    rightSpeedFilter.reset(rightValue);
    angularVelocityFilter.reset(angularValue);
    steerErrorFilter.reset(0.0);
}

void MotorControlTask::setSteerErrorFilterTimeConstant(double tau) {
    if (tau < 0.0) {
        printf("Warning: Invalid steer error filter time constant: %.4f\n", tau);
        return;
    }
    steerErrorFilter.setTimeConstant(tau);
    steerFilterTau.store(tau);

}

// ==================== 斜坡控制 API ====================

void MotorControlTask::enableRampControl(bool enable) {
    bool wasEnabled = rampControlEnabled.load();
    rampControlEnabled.store(enable);

    if (enable && !wasEnabled) {
        leftRampLimiter.reset(0.0);
        rightRampLimiter.reset(0.0);
        printf("Ramp control enabled\n");
    } else if (!enable && wasEnabled) {
        printf("Ramp control disabled\n");
    }
}

bool MotorControlTask::isRampControlEnabled() const {
    return rampControlEnabled.load();
}

void MotorControlTask::setRampRates(double accelRate, double decelRate) {
    if (accelRate <= 0.0 || decelRate <= 0.0) {
        printf("Warning: Invalid ramp rates (accel=%.1f, decel=%.1f)\n", accelRate, decelRate);
        return;
    }
    leftRampLimiter.setAccelRate(accelRate);
    leftRampLimiter.setDecelRate(decelRate);
    rightRampLimiter.setAccelRate(accelRate);
    rightRampLimiter.setDecelRate(decelRate);

}

void MotorControlTask::resetRampState() {
    leftRampLimiter.reset(0.0);
    rightRampLimiter.reset(0.0);
    printf("Ramp state reset\n");
}

// ==================== 外环PD输出斜坡控制 API ====================

void MotorControlTask::enableDiffOutputRamp(bool enable) {
    bool wasEnabled = diffOutputRampEnabled.load();
    diffOutputRampEnabled.store(enable);

    if (enable && !wasEnabled) {
        diffOutputRampLimiter.reset(0.0);
        printf("Diff output ramp control enabled\n");
    } else if (!enable && wasEnabled) {
        printf("Diff output ramp control disabled\n");
    }
}

bool MotorControlTask::isDiffOutputRampEnabled() const {
    return diffOutputRampEnabled.load();
}

void MotorControlTask::setDiffOutputRampRates(double accelRate, double decelRate) {
    if (accelRate <= 0.0 || decelRate <= 0.0) {
        printf("Warning: Invalid diff output ramp rates (accel=%.1f, decel=%.1f)\n",
               accelRate, decelRate);
        return;
    }
    diffOutputRampLimiter.setAccelRate(accelRate);
    diffOutputRampLimiter.setDecelRate(decelRate);
    printf("Diff output ramp rates set: accel=%.1f rad/s², decel=%.1f rad/s²\n",
           accelRate, decelRate);
}

// ==================== 紧急停机/出界保护实现 ====================

void MotorControlTask::emergencyStop() {
    emergencyStopActive.store(true);
    motors->setSpeeds(0.0f, 0.0f);
    leftRampLimiter.reset(0.0);
    rightRampLimiter.reset(0.0);
    printf("*** EMERGENCY STOP - Motors zeroed ***\n");
}

void MotorControlTask::clearEmergencyStop() {
    pidResetRequested.store(true);
    emergencyStopActive.store(false);
    printf("Emergency stop cleared - PID integrals will be reset\n");
}

bool MotorControlTask::isEmergencyStopActive() const {
    return emergencyStopActive.load();
}

// ==================== 碰撞保护实现 ====================

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

}

void MotorControlTask::setCollisionKeyConfig(int resetKeyIndex, int bumperKeyIndex) {
    collisionResetKey.store(resetKeyIndex);
    collisionBumperKey.store(bumperKeyIndex);

}

void MotorControlTask::configureCollisionGpio(const std::array<std::string, 4>& keyPaths) {
    collisionKeyPaths_ = keyPaths;

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
    leftRampLimiter.reset(0.0);
    rightRampLimiter.reset(0.0);
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
