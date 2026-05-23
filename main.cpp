#define MAKE_MAIN_CPP

#include "main.hpp"
#include "camera_calibration.h"
#include "AAAtools.h"

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::this_thread;

// 全局变量定义
IMUDevice imu;
std::unique_ptr<DualMotorController> motors;
Encoder encoder_left("/dev/zf_encoder_1");
Encoder encoder_right("/dev/zf_encoder_2", true);
Control::PID::Parameters leftParams, rightParams;
std::unique_ptr<MotorControlTask> motorTask;
Buzzer buzzer;
MainTestConfig test_config;
std::atomic<bool> g_running(true);
bool g_runtime_config_ok = false;
CameraKind g_camera_kind = CameraKind::VIDEO_0;
bool g_calibration_enabled = false;
bool g_simple_tracking_enabled = false;

JSON_PIDConfigData JSON_PIDConfigData_s;
JSON_SpeedPIDConfigData JSON_LeftSpeedPIDConfigData_s;
JSON_SpeedPIDConfigData JSON_RightSpeedPIDConfigData_s;
JSON_AngularVelocityPIDConfigData JSON_AngularVelocityPIDConfigData_s;
JSON_VehicleConfigData JSON_VehicleConfigData_s;
Function_EN Function_EN_s;
Data_Path Data_Path_s;

ImgProcess imgProcess;
Judge judge;
SYNC Sync;
CameraCalibrationCorrector g_calibration_corrector;


void argument_config(void);
void sigint_handler(int signum);
void cleanup();
int main_init_task();
int main_test_task(const MainTestConfig& test_config);

int main()
{
    const bool runCameraOnlyTest = false;
    if (runCameraOnlyTest)
    {
        std::cout << "[CameraTest] 仅摄像头初始化与采集测试启动" << std::endl;

        VideoCapture camera;
        CameraInit(camera, CameraKind::VIDEO_0, 320, 240, 120);

        Img_Store imgStore;
        std::thread captureThread;
        CameraCaptureThreadStart(camera, &imgStore, captureThread);

        uint64_t frameCount = 0;
        uint64_t emptyFrameCount = 0;
        auto windowStart = std::chrono::steady_clock::now();

        while (1)
        {
            const auto frameStart = std::chrono::steady_clock::now();
            CameraImgGet(&imgStore);
            const auto captureCostUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::steady_clock::now() - frameStart)
                                           .count();

            if (imgStore.Img_Color.empty())
            {
                ++emptyFrameCount;
            }

            ++frameCount;
            if (frameCount % 30 == 0)
            {
                const auto now = std::chrono::steady_clock::now();
                const auto elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(now - windowStart).count();
                const double fps = (elapsedUs > 0) ? (30.0 * 1000000.0 / static_cast<double>(elapsedUs)) : 0.0;

                std::cout << "[CameraTest] frame=" << frameCount
                          << " capture_us=" << captureCostUs
                          << " empty=" << emptyFrameCount
                          << " fps=" << fps << std::endl;
                windowStart = now;
            }
        }

        CameraCaptureThreadStop(&imgStore, captureThread);
        camera.release();
        return EXIT_SUCCESS;
    }

    // 1. 配置参数初始化
    argument_config();
    if (!g_runtime_config_ok)
    {
        cout << "配置同步失败" << endl;
        return EXIT_FAILURE;
    }

    // 2. 硬件设备初始化
    if (main_init_task() == EXIT_SUCCESS)
    {
        cout << "初始化成功" << endl;
    }
    else
    {
        cout << "初始化失败" << endl;
        return EXIT_FAILURE;
    }

    // 3. 功能测试
    if (main_test_task(test_config) != EXIT_SUCCESS)
    {
        cout << "功能测试失败" << endl;
        return EXIT_FAILURE;
    }

    // 4. 启动电机控制任务
    if (!motorTask)
    {
        std::cerr << "[Motor] motorTask 未创建，无法启动主循环" << std::endl;
        return EXIT_FAILURE;
    }

    // 从配置读取角速度PID参数
    {
        Control::PID::Parameters angularVelParams;
        angularVelParams.Kp = JSON_AngularVelocityPIDConfigData_s.Kp;
        angularVelParams.Ki = JSON_AngularVelocityPIDConfigData_s.Ki;
        angularVelParams.Kd = JSON_AngularVelocityPIDConfigData_s.Kd;
        angularVelParams.limitP = JSON_AngularVelocityPIDConfigData_s.limitP;
        angularVelParams.limitI = JSON_AngularVelocityPIDConfigData_s.limitI;
        angularVelParams.limitD = JSON_AngularVelocityPIDConfigData_s.limitD;
        angularVelParams.limitOutput = JSON_AngularVelocityPIDConfigData_s.limitOutput;
        angularVelParams.limitIMin = JSON_AngularVelocityPIDConfigData_s.limitIMin;
        angularVelParams.enableAntiWindup = JSON_AngularVelocityPIDConfigData_s.enableAntiWindup;
        motorTask->setAngularVelocityParams(angularVelParams);
    }

    motorTask->enableAngularVelocityControl(true);
    motorTask->enableRampLimiting(true);
    motorTask->setRampLimits(JSON_VehicleConfigData_s.rampMaxAccel, JSON_VehicleConfigData_s.rampMaxDecel);
    motorTask->setWheelbase(JSON_VehicleConfigData_s.wheelbase);
    motorTask->setWheelRadius(JSON_VehicleConfigData_s.wheelRadius);
    motorTask->setMotorMaxDuty(JSON_VehicleConfigData_s.motorMaxDuty);
    motorTask->start();

    // 5. 初始化摄像头并启动图像采集线程    
    VideoCapture Camera;
    CameraInit(Camera, g_camera_kind, 320, 240, 120);
    Img_Store Img_Store_s;
    std::thread captureThread;
    CameraCaptureThreadStart(Camera, &Img_Store_s, captureThread);

    TempCaptureSession tempCapture(false);
    PerfWindowRecorder perfRecorder(30, false);

    // 6. 主循环：图像处理 -> 赛道识别 -> 电机控制
    while (g_running.load() && Function_EN_s.Game_EN)
    {
        const auto frameStart = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration captureCost = std::chrono::steady_clock::duration::zero();
        std::chrono::steady_clock::duration undistortCost = std::chrono::steady_clock::duration::zero();
        bool undistortExecuted = false;

        if (!tempCapture.handleKeyEvent())
        {
            break;
        }

        const auto captureStart = std::chrono::steady_clock::now();
        CameraImgGet(&Img_Store_s);
        captureCost = std::chrono::steady_clock::now() - captureStart;

        if (!g_running.load())
        {
            printf("退出信号已接收，正在停止摄像头捕获线程...\n");
            break;
        }

        if (Img_Store_s.Img_Color.empty())
        {
            printf("Warning: Captured image is empty, skipping this frame.\n");
            continue;
        }

        tempCapture.saveFrameIfNeeded(Img_Store_s.Img_Color);

        ProcessTrackTaskPerFrame(&Img_Store_s, &Data_Path_s, &Function_EN_s, &imgProcess, &judge);

        
        // perfRecorder.record(std::chrono::steady_clock::now() - frameStart,
        //                     captureCost,
        //                     undistortCost,
        //                     undistortExecuted);
    }

    perfRecorder.flush();
    tempCapture.printSummary();

    CameraCaptureThreadStop(&Img_Store_s, captureThread);
    Camera.release();

    if (motorTask)
    {
        motorTask->setTargetAngularVelocity(0.0);
        motorTask->setBaseSpeed(0.0);
        motorTask->stop();
    }

    return 0;
}

void argument_config(void)
{
    // 1. 测试配置初始化
    test_config.buzzer_test = true;
    test_config.imu_test = false;
    test_config.motor_test = false;
    test_config.angular_velocity_test = false;
    test_config.encoder_test = false;

    // 2. 蜂鸣器参数设置
    buzzer.setShortDuration(60)
            .setLongDuration(300)
            .setIntervalDuration(120);

    // 3. 电机控制器创建
    motors = std::make_unique<DualMotorController>();

    // 4. 配置文件同步
    Sync.ConfigData_SYNC(&Data_Path_s,&Function_EN_s,&JSON_PIDConfigData_s);
    g_runtime_config_ok = !(Function_EN_s.JSON_FunctionConfigData_v.empty() || Data_Path_s.JSON_TrackConfigData_v.empty());
    if (g_runtime_config_ok)
    {
        std::string calibrationError;
        const std::string calibrationJsonPath = "config/calibration.json";
        const std::string calibrationYamlPath = "config/calibration.yaml";

        if (g_calibration_enabled)
        {
            g_calibration_enabled = g_calibration_corrector.load(calibrationYamlPath, &calibrationError);
            if (!g_calibration_enabled)
            {
                std::cerr << "[Calibration] YAML 加载失败: " << calibrationError << std::endl;
            }
            else
            {
                std::cout << "[Calibration] 已加载 YAML 标定参数: " << calibrationYamlPath << std::endl;
            }
        }
        else
        {
            std::cout << "[Calibration] 图像标定使能关闭: " << calibrationJsonPath << std::endl;
        }

        // 从配置读取轮速PID参数
        JSON_LeftSpeedPIDConfigData_s = Data_Path_s.JSON_SpeedPIDConfigData_v[0];
        JSON_RightSpeedPIDConfigData_s = Data_Path_s.JSON_SpeedPIDConfigData_v[1];
        JSON_AngularVelocityPIDConfigData_s = Data_Path_s.JSON_AngularVelocityPIDConfigData_v[0];
        JSON_VehicleConfigData_s = Data_Path_s.JSON_VehicleConfigData_v[0];

        leftParams.Kp = JSON_LeftSpeedPIDConfigData_s.Kp;
        leftParams.Ki = JSON_LeftSpeedPIDConfigData_s.Ki;
        leftParams.Kd = JSON_LeftSpeedPIDConfigData_s.Kd;
        leftParams.limitP = JSON_LeftSpeedPIDConfigData_s.limitP;
        leftParams.limitI = JSON_LeftSpeedPIDConfigData_s.limitI;
        leftParams.limitD = JSON_LeftSpeedPIDConfigData_s.limitD;
        leftParams.limitOutput = JSON_LeftSpeedPIDConfigData_s.limitOutput;
        leftParams.limitIMin = JSON_LeftSpeedPIDConfigData_s.limitIMin;
        leftParams.enableAntiWindup = JSON_LeftSpeedPIDConfigData_s.enableAntiWindup;

        rightParams.Kp = JSON_RightSpeedPIDConfigData_s.Kp;
        rightParams.Ki = JSON_RightSpeedPIDConfigData_s.Ki;
        rightParams.Kd = JSON_RightSpeedPIDConfigData_s.Kd;
        rightParams.limitP = JSON_RightSpeedPIDConfigData_s.limitP;
        rightParams.limitI = JSON_RightSpeedPIDConfigData_s.limitI;
        rightParams.limitD = JSON_RightSpeedPIDConfigData_s.limitD;
        rightParams.limitOutput = JSON_RightSpeedPIDConfigData_s.limitOutput;
        rightParams.limitIMin = JSON_RightSpeedPIDConfigData_s.limitIMin;
        rightParams.enableAntiWindup = JSON_RightSpeedPIDConfigData_s.enableAntiWindup;

        g_camera_kind = Function_EN_s.JSON_FunctionConfigData_v[0].Camera_EN;
        Function_EN_s.Game_EN = true;
        // 默认从图像循环开始，等待第一帧完成赛道状态判定后再切换到对应任务。
        Data_Path_s.Loop_Kind = CAMERA_CATCH_LOOP;
    }
    else
    {
        std::cerr << "[Config] 配置同步失败" << std::endl;
        std::cerr << "[Config] Function 配置数量: " << Function_EN_s.JSON_FunctionConfigData_v.size()
                  << ", Track 配置数量: " << Data_Path_s.JSON_TrackConfigData_v.size() << std::endl;
        if (Function_EN_s.JSON_FunctionConfigData_v.empty())
        {
            std::cerr << "[Config] JSON_FunctionConfigData_v 为空，请检查功能配置文件" << std::endl;
        }
        if (Data_Path_s.JSON_TrackConfigData_v.empty())
        {
            std::cerr << "[Config] JSON_TrackConfigData_v 为空，请检查赛道配置文件" << std::endl;
        }
        Function_EN_s.Game_EN = false;
    }

    // 6. 电机控制任务创建
    {
        double ctrlPeriod = g_runtime_config_ok ? JSON_VehicleConfigData_s.controlPeriod : 0.01;
        motorTask = std::make_unique<MotorControlTask>(
                leftParams,
                rightParams,
                motors.get(),
                &encoder_left,
                &encoder_right,
                &imu,
                ctrlPeriod
            );
    }
}

void sigint_handler(int signum)
{
    (void)signum;
    g_running.store(false);
    if (motorTask)
    {
        motorTask->stop();
    }
    if (motors)
    {
        motors->stopAll();
    }
    exit(EXIT_SUCCESS);
}

void cleanup()
{
    printf("程序退出，执行清理操作\n");
    g_running.store(false);
    if (motorTask)
    {
        motorTask->stop();
    }
    if (motors)
    {
        motors->stopAll();
    }
    exit(EXIT_SUCCESS);
}

int main_init_task()
{
    // 1. 设置程序退出清理函数
    atexit(cleanup);

    // 2. 设置信号处理函数（Ctrl+C）
    signal(SIGINT, sigint_handler);

    // 3. 初始化显示屏
    setbuf(stdout, NULL);
    ips200_init("/dev/fb0");

    // 4. 显示IP地址
    display_ip_address(0, 181);
    printf("IP address displayed on screen.\n");

    // 5. 初始化UDP通信
    if (udp_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    uint8 temp_str[] = "UDP IS READY.\r\n";
    udp_dev.send_data(temp_str, sizeof(temp_str));

    // 6. 初始化IMU设备
    if (!imu.initialize())
    {
        printf("Failed to initialize IMU device\n");
        return EXIT_FAILURE;
    }
    else
    {
        printf("IMU device initialized successfully\n");
    }

    return EXIT_SUCCESS;
}

int main_test_task(const MainTestConfig& test_config)
{
    if (test_config.buzzer_test)
    {
        try
        {
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
        }
        catch (const std::exception& e)
        {
            std::cerr << "Buzzer example failed: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (test_config.imu_test)
    {
        imu_device_type_t type = imu.get_device_type();
        printf("IMU Device Type: %d\n", type);

        printf("\n=== Zero Drift Calibration Example ===\n");
        if (imu.measure_zero_drift())
        {
            printf("Zero drift calibration successful!\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            printf("Bias values stored in IMU device.\n");

            printf("\n=== Compensated Data Example ===\n");
            for (int i = 0; i < 3; i++)
            {
                if (imu.update_all_data())
                {
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
        }
        else
        {
            printf("Zero drift calibration failed or skipped.\n");
        }

        for (int i = 0; i < 3; i++)
        {
            auto start_time = steady_clock::now();
            if (imu.update_all_data())
            {
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

    if (test_config.motor_test)
    {
        motorTask->start();
        motorTask->enableRampLimiting(true);
        motorTask->setRampLimits(JSON_VehicleConfigData_s.rampMaxAccel, JSON_VehicleConfigData_s.rampMaxDecel);
        try
        {
            printf("\n=== 测试1：无斜坡限制的基本测试 ===\n");
            motorTask->setTargetSpeed(1, 4);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            motorTask->setTargetSpeed(4, 1);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            motorTask->setTargetSpeed(0, 0);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            motorTask->enableRampLimiting(false);
            motorTask->stop();
            printf("\n斜坡限制测试完成\n");

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        catch (const std::exception& e)
        {
            std::cerr << "错误: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (test_config.angular_velocity_test)
    {
        printf("\n=== Angular Velocity Control Test ===\n");
        try
        {
            motorTask->enableAngularVelocityControl(true);
            motorTask->enableRampLimiting(true);
            motorTask->setRampLimits(JSON_VehicleConfigData_s.rampMaxAccel, JSON_VehicleConfigData_s.rampMaxDecel);
            motorTask->start();

            motorTask->setBaseSpeed(0.5);
            printf("\nTest 1: Clockwise rotation (+90°/s)\n");
            motorTask->setTargetAngularVelocity(90.0);
            std::this_thread::sleep_for(std::chrono::seconds(15));

            printf("\nTest 2: Counter-clockwise rotation (-90°/s)\n");
            motorTask->setTargetAngularVelocity(-90.0);
            std::this_thread::sleep_for(std::chrono::seconds(30));

            printf("\nTest 3: Fast rotation (+180°/s)\n");
            motorTask->setTargetAngularVelocity(180.0);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            printf("\nTest 4: Slow rotation (+45°/s)\n");
            motorTask->setTargetAngularVelocity(45.0);
            std::this_thread::sleep_for(std::chrono::seconds(4));

            printf("\nTest 5: Straight line (0°/s)\n");
            motorTask->setTargetAngularVelocity(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            printf("\nTest 6: Change base speed to 0.3 m/s with +60°/s\n");
            motorTask->setBaseSpeed(0.3);
            motorTask->setTargetAngularVelocity(60.0);
            std::this_thread::sleep_for(std::chrono::seconds(4));

            motorTask->stop();
            motorTask->enableAngularVelocityControl(false);
            motorTask->enableRampLimiting(false);
            printf("\nAngular velocity control test completed.\n");
        }
        catch (const std::exception& e)
        {
            std::cerr << "Angular velocity control test error: " << e.what() << std::endl;
            motorTask->enableAngularVelocityControl(false);
            return EXIT_FAILURE;
        }
    }

    if (test_config.encoder_test)
    {
        try
        {
            if (!encoder_left.isValid())
            {
                std::cerr << "Warning: Left encoder device not accessible\n";
            }
            if (!encoder_right.isValid())
            {
                std::cerr << "Warning: Right encoder device not accessible\n";
            }

            std::cout << "Encoder reading started. Device paths:\n"
                      << "  Left:  " << encoder_left.devicePath() << "\n"
                      << "  Right: " << encoder_right.devicePath() << "\n\n";

            std::cout << "Conversion factor: " << encoder_left.conversionFactor() << std::endl;
            for (int i = 0; i < 3; i++)
            {
                try
                {
                    auto left_speed = encoder_left.readSpeed();
                    auto right_speed = encoder_right.readSpeed();
                    std::cout << "Speed - Left: " << left_speed << " m/s, Right: " << right_speed << " m/s" << std::endl;
                }
                catch (const EncoderException& e)
                {
                    std::cerr << "Read error: " << e.what() << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Fatal error: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::cout << "自检完成..." << std::endl;
    motorTask->stop();
    return EXIT_SUCCESS;
}

