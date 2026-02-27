#include "main.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

zf_driver_udp udp_dev;
IMUDevice imu;
std::unique_ptr<DualMotorController> motors;
Encoder encoder_left("/dev/zf_encoder_1");
Encoder encoder_right("/dev/zf_encoder_2", true); // 右轮编码器取反

int main() {
    if (main_init_task() == EXIT_SUCCESS) { 
        cout << "初始化成功" << endl; 
    } else { 
        cout << "初始化失败" << endl; return EXIT_FAILURE; 
    }
    motor_control_task();
    // VideoCapture Camera; CameraInit(Camera,CameraKind::VIDEO_0,320,240,120);
    


    // Camera.release();
    return 0;
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    motors->stopAll();
    exit(0);
}

void cleanup()
{
    motors->stopAll();
    printf("程序退出，执行清理操作\n");
}

int main_init_task()
{
    // 任务初始化代码
    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    setbuf(stdout, NULL);
    ips200_init("/dev/fb0");
    
    // 显示IP地址
    display_ip_address(0, 181);
    printf("IP address displayed on screen.\n");

    //==================================== UDP_TEST ====================================

    if(udp_dev.init(SERVER_IP, PORT) == 0){printf("tcp_client ok\r\n");}
    else{printf("tcp_client error\r\n");return -1;}
    
    uint8 temp_str[] = "UDP IS READY.\r\n";
    udp_dev.send_data(temp_str, sizeof(temp_str));

    //==================================== IMU_TEST ====================================
    
    if (!imu.initialize()) {
        printf("Failed to initialize IMU device\n");
        return EXIT_FAILURE;
    }

    imu_device_type_t type = imu.get_device_type();
    printf("IMU Device Type: %d\n", type);
    
    for (int i = 0; i < 3; i++) {
        // 更新所有数据
        auto start_time = steady_clock::now();
        if (imu.update_all_data()) {
            // 获取完整数据
            const imu_raw_data_t& data = imu.get_raw_data();

            auto end_time = steady_clock::now();
            auto duration = duration_cast<microseconds>(end_time - start_time);
            printf("Sample %d took %ld microseconds\n", i + 1, duration.count());

            printf("Sample %d:\n", i + 1);
            printf("  Acc: X=%6d, Y=%6d, Z=%6d\n", data.acc_x, data.acc_y, data.acc_z);
            printf("  Gyro: X=%6d, Y=%6d, Z=%6d\n", data.gyro_x, data.gyro_y, data.gyro_z);
            
            printf("\n");
        }
    }

    //=================================== MOTOR_TEST ===================================
    try {
        cout << "初始化双电机控制器..." << endl;
        // 创建双电机控制器
        motors = std::make_unique<DualMotorController>();
        
        // 设置最大占空比限制为30%
        motors->setMaxDutyLimits(30.0f);
        
        // 示例1：向前移动
        std::cout << "向前移动..." << std::endl;
        motors->setSpeeds(0.5f, 0.5f);  // 50%速度向前
        usleep(500000);
        
        // 示例2：停止
        std::cout << "停止..." << std::endl;
        motors->stopAll();
        usleep(500000);
        
        // 示例3：向后移动
        std::cout << "向后移动..." << std::endl;
        motors->setSpeeds(-0.3f, -0.3f);  // 30%速度向后
        usleep(500000);
        
        // 示例4：转弯
        std::cout << "直行..." << std::endl;
        motors->setSpeeds(0.3f, 0.3f);
        usleep(500000);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    //================================== ENCODER_TEST ==================================
    try {
        // 验证设备是否可用
        if (!encoder_left.isValid()) {
            std::cerr << "Warning: Left encoder device not accessible\n";
        }
        if (!encoder_right.isValid()) {
            std::cerr << "Warning: Right encoder device not accessible\n";
        }
        
        std::cout << "Encoder reading started. Device paths:\n"
                  << "  Left:  " << encoder_left.devicePath() << "\n"
                  << "  Right: " << encoder_right.devicePath() << "\n\n";
        
        // 主循环 - 直接读取两个编码器
        std::cout << "Conversion factor: " << encoder_left.conversionFactor() << std::endl;
        for (int i = 0; i < 3; i++) {
            try {
                // int16_t left_value = encoder_left.readCount();
                // int16_t right_value = encoder_right.readCount();
                
                // std::cout << "Encoder values - Left: " << left_value 
                //          << ", Right: " << right_value 
                //          << std::endl;
            
                // 直接读取速度值(m/s)
                auto left_speed = encoder_left.readSpeed();
                auto right_speed = encoder_right.readSpeed();
                std::cout << "Speed - Left: " << left_speed << " m/s, Right: " << right_speed << " m/s" << std::endl;
                
            } catch (const EncoderException& e) {
                std::cerr << "Read error: " << e.what() << std::endl;
                // 出错时短暂延时后继续
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "自检完成..." << std::endl;
    motors->stopAll();

    return EXIT_SUCCESS;
}

int motor_control_task()
{
    // PID控制器状态枚举
    enum class ControlState {
        INIT,           // 初始化
        RUNNING,        // 运行中
        ERROR,          // 错误状态
        STOPPED         // 停止
    };
    
    ControlState state = ControlState::INIT;
    
    // 创建PID控制器
    Control::PID::Parameters params;
    params.Kp = 0.3;           // 比之前稍大一点，提供足够响应
    params.Ki = 0.1;           // 中等积分，消除静差
    params.Kd = 0.0;            // 依然保持为0，除非有特殊需求
    params.limitP = 1.0;        // 比例项限幅（通常不需要限制）
    params.limitI = 0.8;        // **减小积分限幅**，防止积分饱和过冲
    params.limitD = 0.0;        // 保留
    params.limitIMin = -0.8;    // **减小负积分限幅**，保持对称
    params.limitOutput = 1.0;   // 总输出限幅 ±100%
    params.enableAntiWindup = true;  // 保持开启

    
    Control::PID speedPID(params);
    
    // 设置电机限制
    motors->setMaxDutyLimits(50.0f);
    
    // 控制参数
    double targetSpeed = 3.0;
    const double controlPeriod = 0.01;
    
    // 监控变量
    int errorCount = 0;
    const int maxErrorCount = 10;
    double lastValidSpeed = 0.0;
    
    printf("Motor speed control initialized. Target: %.2f m/s\n", targetSpeed);
    state = ControlState::RUNNING;
    
    while (state != ControlState::STOPPED) {
        try {
            // 读取当前速度
            double currentSpeed = encoder_left.readSpeed();
            
            // 速度值有效性检查
            if (std::isnan(currentSpeed) || std::isinf(currentSpeed) || currentSpeed < -10.0 || currentSpeed > 10.0) {
                errorCount++;
                printf("Warning: Invalid speed reading: %.3f (error %d/%d)\n", 
                       currentSpeed, errorCount, maxErrorCount);
                currentSpeed = lastValidSpeed;  // 使用上次有效值
                
                if (errorCount >= maxErrorCount) {
                    printf("ERROR: Too many invalid speed readings. Stopping motor.\n");
                    motors->setSpeeds(0.0f, 0.0f);
                    state = ControlState::ERROR;
                    break;
                }
            } else {
                errorCount = 0;
                lastValidSpeed = currentSpeed;
            }
            
            // PID计算
            double controlOutput = speedPID.calculate(targetSpeed, currentSpeed, controlPeriod);
            
            // 应用控制输出
            motors->setSpeeds(static_cast<float>(controlOutput), 0.0f);  // 只控制左轮，右轮保持不动
            
            // 周期性状态输出（每秒一次）
            static int cycleCount = 0;
            if (++cycleCount >= static_cast<int>(1.0 / controlPeriod)) {
                printf("Status - Target:%.3f, Current:%.3f, Output:%.3f, Error:%.3f\n",
                       targetSpeed, currentSpeed, controlOutput, 
                       targetSpeed - currentSpeed);
                
                // UDP发送PID状态数据

                std::array<float, 5> data = {
                (float)targetSpeed,
                (float)currentSpeed,
                (float)controlOutput,
                (float)speedPID.getError(),
                (float)speedPID.getIntegral()
                };

                send_udp_data("motor_status", data.data(), data.size());

                cycleCount = 0;
            }
            
        } catch (const std::exception& e) {
            printf("Exception in control loop: %s\n", e.what());
            motors->setSpeeds(0.0f, 0.0f);
            state = ControlState::ERROR;
            break;
        }
        
        // 等待下一个控制周期
        usleep(controlPeriod * 100000);
    }
    
    // 错误处理
    if (state == ControlState::ERROR) {
        printf("Motor control task terminated due to errors.\n");
        return -1;
    }
    
    return 0;
}

int posture_control_task()
{
    // 姿态控制任务代码

    // 获取目标速度

    // 获取目标转向角速度

    // 速度滤波控制算法

    // 转向角速度滤波控制算法

    // 运动学分解

    // 陀螺仪数据读取

    // 编码器数据读取

    // 左右轮PID计算

    // 左轮实际速度

    // 右轮实际速度

    // 左右轮PWM输出


    return 1;
}

void send_udp_data(const char* channelName, const float* dataArray, uint32 dataCount) {
    // 定义发送缓冲区，根据最大可能需求设置大小
    // 计算公式：通道名称长度 + 冒号 + 每个数据最多15字符(含小数点) + 逗号 + 换行符 + 结束符
    uint8 send_buf[256];  
    int len = 0;
    int offset = 0;
    
    // 1. 添加通道名称和冒号
    if (channelName != NULL && strlen(channelName) > 0) {
        offset += snprintf((char*)send_buf, sizeof(send_buf), "%s:", channelName);
    }
    
    // 2. 循环添加所有数据
    for (uint32 i = 0; i < dataCount; i++) {
        if (i == 0) {
            // 第一个数据，前面不加逗号
            offset += snprintf((char*)send_buf + offset, sizeof(send_buf) - offset, 
                              "%.3f", dataArray[i]);
        } else {
            // 后续数据，前面加逗号
            offset += snprintf((char*)send_buf + offset, sizeof(send_buf) - offset, 
                              ",%.3f", dataArray[i]);
        }
        
        // 检查缓冲区是否溢出
        if (offset >= sizeof(send_buf) - 2) {
            // 缓冲区快满了，强制添加换行符后退出
            snprintf((char*)send_buf + offset, sizeof(send_buf) - offset, "\n");
            break;
        }
    }
    
    // 3. 添加换行符（如果还没有添加的话）
    if (offset < sizeof(send_buf) - 1 && send_buf[offset-1] != '\n') {
        offset += snprintf((char*)send_buf + offset, sizeof(send_buf) - offset, "\n");
    }
    
    // 4. 发送数据
    len = offset;
    if (len > 0 && len < sizeof(send_buf)) {
        udp_dev.send_data(send_buf, (uint32)len);
    }
}

void pit_callback()
{
    encoder_get_count(ENCODER_1);
    encoder_get_count(ENCODER_2);
}


