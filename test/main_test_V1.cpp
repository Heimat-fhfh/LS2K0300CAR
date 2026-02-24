#include "main.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;


IMUDevice imu;
std::unique_ptr<DualMotorController> motors;
Encoder encoder_left("/dev/zf_encoder_1");
Encoder encoder_right("/dev/zf_encoder_2");

int main() {
    if (main_init_task() == EXIT_SUCCESS) { 
        cout << "初始化成功" << endl; 
    } else { 
        cout << "初始化失败" << endl; return EXIT_FAILURE; 
    }
    VideoCapture Camera; CameraInit(Camera,CameraKind::VIDEO_0,320,240,120);
    


    // Camera.release();
    return 0;
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    
    exit(0);
}

void cleanup()
{
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

    if (!imu.initialize()) {
        printf("Failed to initialize IMU device\n");
        return EXIT_FAILURE;
    }

    imu_device_type_t type = imu.get_device_type();
    printf("IMU Device Type: %d\n", type);

    //==================================== IMU_TEST ====================================
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
        usleep(2000000);
        
        // 示例2：停止
        std::cout << "停止..." << std::endl;
        motors->stopAll();
        usleep(1000000);
        
        // 示例3：向后移动
        std::cout << "向后移动..." << std::endl;
        motors->setSpeeds(-0.3f, -0.3f);  // 30%速度向后
        usleep(2000000);
        
        // 示例4：转弯
        std::cout << "右转..." << std::endl;
        motors->setSpeeds(0.8f, 0.8f);  // 左轮40%，右轮20%
        usleep(1500000);
        
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
        for (int i = 0; i < 3; i++) {
            try {
                int16_t left_value = encoder_left.readCount();
                int16_t right_value = encoder_right.readCount();
                
                std::cout << "Encoder values - Left: " << left_value 
                         << " (" << std::dec << static_cast<int>(left_value) << ")"
                         << ", Right: " << right_value 
                         << " (" << std::dec << static_cast<int>(right_value) << ")"
                         << std::endl;
                
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

void pit_callback()
{
    encoder_get_count(ENCODER_1);
    encoder_get_count(ENCODER_2);
}
