#include "MotorControlTask.hpp"
#include <pthread.h>
#include <cmath>
#include <cstring>

// UDP发送函数的实现（如果未在其他地方定义，这里可以放空实现或链接外部函数）
void send_udp_data(const char* topic, const float* data, size_t size) {
    // 实际实现应该在别处，这里只是一个占位
    // 如果需要，可以在这里添加实际的UDP发送代码
}

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
    , leftParams(leftParams)
    , rightParams(rightParams)
    , motors(motors)
    , leftEncoder(leftEncoder)
    , rightEncoder(rightEncoder)
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
    
    // 设置电机限制
    motors->setMaxDutyLimits(50.0f);
    
    // 监控变量
    const int maxErrorCount = 10;
    int leftErrorCount = 0;
    int rightErrorCount = 0;
    double lastValidLeftSpeed = 0.0;
    double lastValidRightSpeed = 0.0;
    
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
            
            // PID计算
            double leftOutput = leftPID.calculate(currentLeftTarget, leftSpeed, controlPeriod);
            double rightOutput = rightPID.calculate(currentRightTarget, rightSpeed, controlPeriod);
            
            // 应用控制输出
            motors->setSpeeds(static_cast<float>(leftOutput), static_cast<float>(rightOutput));
            
            // 周期性状态输出（每秒一次）
            static int cycleCount = 0;
            if (++cycleCount >= static_cast<int>(1.0 / controlPeriod)) {
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