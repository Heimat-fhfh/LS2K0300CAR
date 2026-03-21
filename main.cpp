#include "main.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::this_thread;



IMUDevice imu;
std::unique_ptr<DualMotorController> motors;
Encoder encoder_left("/dev/zf_encoder_1");
Encoder encoder_right("/dev/zf_encoder_2", true); // 右轮编码器取反
Control::PID::Parameters leftParams,rightParams;
std::unique_ptr<MotorControlTask> motorTask;
Buzzer buzzer;
MainTestConfig test_config;

int main() {
    argument_config();
    if (main_init_task() == EXIT_SUCCESS) { cout << "初始化成功" << endl; } else { cout << "初始化失败" << endl; return EXIT_FAILURE; }
    
    if (main_test_task(test_config) != EXIT_SUCCESS) {
        cout << "功能测试失败" << endl;
        return EXIT_FAILURE;
    }


    // VideoCapture Camera; CameraInit(Camera,CameraKind::VIDEO_0,320,240,120);



    
    // Camera.release();
    return 0;
}

void argument_config(void)
{
    test_config.buzzer_test = false;
    test_config.imu_test = false;
    test_config.motor_test = false;
    test_config.angular_velocity_test = false;
    test_config.encoder_test = false;

    buzzer.setShortDuration(60)
            .setLongDuration(300)
            .setIntervalDuration(120);

    motors = std::make_unique<DualMotorController>();

    leftParams.Kp = 1.0;
    leftParams.Ki = 0.4;
    leftParams.Kd = 0.0;
    leftParams.limitP = 0.9;
    leftParams.limitI = 0.7;
    leftParams.limitD = 0.0;
    leftParams.limitIMin = -0.7;
    leftParams.limitOutput = 0.7;
    leftParams.enableAntiWindup = true;

    rightParams = leftParams;

    // 使用带IMU的构造函数创建电机控制任务
    motorTask = std::make_unique<MotorControlTask>(
            leftParams,
            rightParams,
            motors.get(),
            &encoder_left,
            &encoder_right,
            &imu,           // 传入IMU设备指针
            0.01            // 10ms控制周期
        );
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

    if(udp_dev.init(SERVER_IP, PORT) == 0){printf("tcp_client ok\r\n");}
    else{printf("tcp_client error\r\n");return -1;}
    
    uint8 temp_str[] = "UDP IS READY.\r\n";
    udp_dev.send_data(temp_str, sizeof(temp_str));

    if (!imu.initialize()) {
        printf("Failed to initialize IMU device\n");
        return EXIT_FAILURE;
    }else {
        printf("IMU device initialized successfully\n");
    }

    return EXIT_SUCCESS;
}

int main_test_task(const MainTestConfig& test_config)
{
    //================================== BUZZER_TEST ==================================//
    if (test_config.buzzer_test) {
        try {
            std::cout << "短鸣 1 次" << std::endl;
            buzzer.shortBeep();
            std::this_thread::sleep_for(std::chrono::milliseconds(600));

            std::cout << "双短鸣" << std::endl;
            buzzer.patternDoubleShort();
            std::this_thread::sleep_for(std::chrono::milliseconds(900));

            std::cout << "一长一短" << std::endl;
            buzzer.patternLongShort();
            std::this_thread::sleep_for(std::chrono::milliseconds(1200));

            std::cout << "自定义模式（300ms 响 / 100ms 停，重复 3 次）" << std::endl;
            buzzer.customPattern({300}, {100}, 3);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));

            std::cout << "连续鸣叫 2 秒" << std::endl;
            buzzer.patternContinuous();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            buzzer.stop();
        } catch (const std::exception& e) {
            std::cerr << "Buzzer example failed: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    //==================================== IMU_TEST ====================================
    if (test_config.imu_test) {
        imu_device_type_t type = imu.get_device_type();
        printf("IMU Device Type: %d\n", type);

        // 示例：测量零漂
        printf("\n=== Zero Drift Calibration Example ===\n");
        if (imu.measure_zero_drift()) {
            printf("Zero drift calibration successful!\n");
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒，确保数据稳定
            printf("Bias values stored in IMU device.\n");
            
            // 示例：获取补偿后的数据
            printf("\n=== Compensated Data Example ===\n");
            for (int i = 0; i < 3; i++) {
                if (imu.update_all_data()) {
                    imu.apply_zero_drift_compensation();
                    const imu_unit_data_t& raw_data = imu.get_unit_data();
                    const imu_unit_data_t& comp_data = imu.get_compensated_unit_data();
                    
                    printf("Sample %d:\n", i + 1);
                    printf("  Raw Gyro: X=%.2f, Y=%.2f, Z=%.2f °/s\n", 
                           raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z);
                    printf("  Comp Gyro: X=%.2f, Y=%.2f, Z=%.2f °/s\n", 
                           comp_data.gyro_x, comp_data.gyro_y, comp_data.gyro_z);
                    printf("\n");
                }
                sleep_for(milliseconds(100));
            }
        } else {
            printf("Zero drift calibration failed or skipped.\n");
        }
        
        for (int i = 0; i < 3; i++) {
            // 更新所有数据
            auto start_time = steady_clock::now();
            if (imu.update_all_data()) {
                // 获取完整数据
                const imu_raw_data_t& data = imu.get_raw_data();
                const imu_unit_data_t& unit_data = imu.get_unit_data();

                auto end_time = steady_clock::now();
                auto duration = duration_cast<microseconds>(end_time - start_time);
                printf("Sample %d took %ld microseconds\n", i + 1, duration.count());

                printf("Sample %d:\n", i + 1);
                printf("  Acc: X=%6d(%.2fg), Y=%6d(%.2fg), Z=%6d(%.2fg)\n", data.acc_x, unit_data.acc_x, data.acc_y, unit_data.acc_y, data.acc_z, unit_data.acc_z);
                printf("  Gyro: X=%6d(%.2f°/s), Y=%6d(%.2f°/s), Z=%6d(%.2f°/s)\n", data.gyro_x, unit_data.gyro_x, data.gyro_y, unit_data.gyro_y, data.gyro_z, unit_data.gyro_z);
                
                printf("\n");

                sleep_for(milliseconds(20));
            }
        }
    }

    //=================================== MOTOR_TEST ===================================
    if (test_config.motor_test) {
        motorTask->start();
        motorTask->enableRampLimiting(true);
        motorTask->setRampLimits(0.8, 0.2);  // 设置较小的加速度限制
        try {
            // 测试1：无斜坡限制的基本测试
            printf("\n=== 测试1：无斜坡限制的基本测试 ===\n");
            motorTask->setTargetSpeed(1, 4);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            motorTask->setTargetSpeed(4, 1);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            motorTask->setTargetSpeed(0, 0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 禁用斜坡限制
            motorTask->enableRampLimiting(false);
            motorTask->stop();
            printf("\n斜坡限制测试完成\n");

            std::this_thread::sleep_for(std::chrono::seconds(1));
            
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    //=================================== ANGULAR_VELOCITY_CONTROL_TEST ===================================
    if (test_config.angular_velocity_test) {
        printf("\n=== Angular Velocity Control Test ===\n");
        try {
            // 启用角速度控制
            motorTask->enableAngularVelocityControl(true);
            motorTask->enableRampLimiting(true);
            motorTask->setRampLimits(0.8, 0.2);  // 设置较小的加速度限制
            motorTask->start();
            
            // 设置基础速度
            motorTask->setBaseSpeed(0.5);  // 基础速度
            
            // 测试1：顺时针旋转（正角速度）
            printf("\nTest 1: Clockwise rotation (+90°/s)\n");
            motorTask->setTargetAngularVelocity(90.0);  // 90°/s 顺时针
            std::this_thread::sleep_for(std::chrono::seconds(15));
            
            // 测试2：逆时针旋转（负角速度）
            printf("\nTest 2: Counter-clockwise rotation (-90°/s)\n");
            motorTask->setTargetAngularVelocity(-90.0);  // -90°/s 逆时针
            std::this_thread::sleep_for(std::chrono::seconds(30));
            
            // 测试3：快速旋转
            printf("\nTest 3: Fast rotation (+180°/s)\n");
            motorTask->setTargetAngularVelocity(180.0);  // 180°/s 顺时针
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // 测试4：慢速旋转
            printf("\nTest 4: Slow rotation (+45°/s)\n");
            motorTask->setTargetAngularVelocity(45.0);  // 45°/s 顺时针
            std::this_thread::sleep_for(std::chrono::seconds(4));
            
            // 测试5：停止旋转，只保持基础速度
            printf("\nTest 5: Straight line (0°/s)\n");
            motorTask->setTargetAngularVelocity(0.0);  // 0°/s 直行
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // 测试6：改变基础速度
            printf("\nTest 6: Change base speed to 0.3 m/s with +60°/s\n");
            motorTask->setBaseSpeed(0.3);  // 降低基础速度
            motorTask->setTargetAngularVelocity(60.0);  // 60°/s 顺时针
            std::this_thread::sleep_for(std::chrono::seconds(4));
            
            // 禁用角速度控制
            motorTask->stop();
            motorTask->enableAngularVelocityControl(false);
            motorTask->enableRampLimiting(false);
            printf("\nAngular velocity control test completed.\n");
            
        } catch (const std::exception& e) {
            std::cerr << "Angular velocity control test error: " << e.what() << std::endl;
            motorTask->enableAngularVelocityControl(false);
            return EXIT_FAILURE;
        }
    }

    //================================== ENCODER_TEST ==================================
    if (test_config.encoder_test) {
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
    }

    std::cout << "自检完成..." << std::endl;
    motorTask->stop();

    return EXIT_SUCCESS;
}

void pit_callback()
{
    encoder_get_count(ENCODER_1);
    encoder_get_count(ENCODER_2);
}


