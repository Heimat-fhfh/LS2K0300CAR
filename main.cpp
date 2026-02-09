#include "main.hpp"

using namespace std;
using namespace cv;

IMUDevice imu;


int main() {
    if (main_init_task() == 1) { 
        cout << "初始化成功" << endl; 
    } else { 
        cout << "初始化失败" << endl; return -1; 
    }
    // VideoCapture Camera; CameraInit(Camera,CameraKind::VIDEO_0,320,240,60);
    


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
        return -1;
    }

    imu_device_type_t type = imu.get_device_type();
    printf("IMU Device Type: %d\n", type);

    for (int i = 0; i < 10; i++) {
        // 更新所有数据
        if (imu.update_all_data()) {
            // 获取完整数据
            const imu_raw_data_t& data = imu.get_raw_data();
            
            printf("Sample %d:\n", i + 1);
            printf("  Acc: X=%6d, Y=%6d, Z=%6d\n", data.acc_x, data.acc_y, data.acc_z);
            printf("  Gyro: X=%6d, Y=%6d, Z=%6d\n", data.gyro_x, data.gyro_y, data.gyro_z);
            
            printf("\n");
        }
        
        // 延时100ms
        usleep(100000);
    }

    return 1;
}

void pit_callback()
{
    encoder_get_count(ENCODER_1);
    encoder_get_count(ENCODER_2);
}
