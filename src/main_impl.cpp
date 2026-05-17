#include "main.hpp"
#include "camera_calibration.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

void ReadInput_CameraCatch(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Function_EN_p;
    Data_Path_p->JSON_TrackConfigData_v[0].Forward = Data_Path_p->JSON_TrackConfigData_v[0].Default_Forward; // 前瞻点初始化
}

// 摄像头捕获任务的算法处理函数
void ProcessAlgo_CameraCatch(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    // imgProcess.imgPreProc(Img_Store_p,Data_Path_p,Function_EN_p); // 图像预处理

    // 仅对二值图去畸变，降低计算开销并直接服务后续寻线。
    // if (g_calibration_enabled && !Img_Store_p->Img_OTSU.empty())
    // {
    //     cv::Mat correctedBinary;
    //     if (g_calibration_corrector.correct(Img_Store_p->Img_OTSU, correctedBinary))
    //     {
    //         Img_Store_p->Img_OTSU = std::move(correctedBinary);
    //     }
    // }

    // if (!Img_Store_p->Img_OTSU.empty())
    // {
    //     memcpy(Img_Store_p->bin_image[0], Img_Store_p->Img_OTSU.data, image_h * image_w * sizeof(uint8));
    //     // 将处理后的二值图像数据复制到二维数组中，供后续算法使用
    // }
    // imgSearch_l_r(Img_Store_p,Data_Path_p);   // 边线八邻域寻线
}

void OutputDisplay_CameraCatch(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
    // imgProcess.ImgLabel(Img_Store_p,Data_Path_p,Function_EN_p);
    // displayMatOnIPS200(Img_Store_p->Img_OTSU);
}

void RunCameraCatchTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    ReadInput_CameraCatch(Img_Store_p,Data_Path_p,Function_EN_p);
    ProcessAlgo_CameraCatch(Img_Store_p,Data_Path_p,Function_EN_p);
    OutputDisplay_CameraCatch(Img_Store_p,Data_Path_p,Function_EN_p);
}

void ReadInput_JudgeTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void ProcessAlgo_JudgeTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    Function_EN_p->Loop_Kind_EN = judge.TrackKind_Judge(Img_Store_p,Data_Path_p,Function_EN_p);  // 切换至赛道循环
}

void OutputDisplay_JudgeTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void RunJudgeTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    ReadInput_JudgeTask(Img_Store_p,Data_Path_p,Function_EN_p);
    ProcessAlgo_JudgeTask(Img_Store_p,Data_Path_p,Function_EN_p);
    OutputDisplay_JudgeTask(Img_Store_p,Data_Path_p,Function_EN_p);
}

void ReadInput_CommonTrackTask(Function_EN *Function_EN_p)
{
    (void)Function_EN_p;
}

void ProcessAlgo_CommonTrackTask(Function_EN *Function_EN_p)
{
    Function_EN_p->Loop_Kind_EN = CAMERA_CATCH_LOOP;
}

void OutputDisplay_CommonTrackTask(Function_EN *Function_EN_p)
{
    (void)Function_EN_p;
}

void RunCommonTrackTask(Function_EN *Function_EN_p)
{
    ReadInput_CommonTrackTask(Function_EN_p);
    ProcessAlgo_CommonTrackTask(Function_EN_p);
    OutputDisplay_CommonTrackTask(Function_EN_p);
}

void ReadInput_CircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void ProcessAlgo_CircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    // 圆环赛道状态机只在本轮周期内完成“补线 + 寻线 + 回到图像循环”的闭环。
    // 其中 Circle_Track_Step 由决策模块提前写入，这里只负责按步骤执行对应补线算法。
    switch(Data_Path_p->Circle_Track_Step)
    {
        case IN_PREPARE:
        {
            // 准备入环：根据当前圆环方向先做预补线，避免进入环口时边线断裂。
            CircleTrack_Step_IN_Prepare(Img_Store_p,Data_Path_p);   // 准备入环补线
            break;
        }
        case IN:
        {
            // 入环：沿已确认的圆环方向继续补线，保证进入环内后仍能稳定寻线。
            CircleTrack_Step_IN(Img_Store_p,Data_Path_p);   // 入环补线
            break;
        }
        case OUT:
        {
            // 出环：完成从环内到环外的补线过渡，给后续恢复普通寻线提供连续边线。
            CircleTrack_Step_OUT(Img_Store_p,Data_Path_p);   // 出环补线
            break;
        }
        default:
        {
            // INIT / OUT_2_STRIGHT 等非补线阶段直接跳过，由普通图像循环接管。
            break;
        }
    }

    // 补线结束后重新寻线，输出给下一帧的决策模块使用。
    imgSearch_l_r(Img_Store_p,Data_Path_p);
    // 圆环任务只占用一个周期，执行完立即切回图像循环，等待下一次状态机判定。
    Function_EN_p->Loop_Kind_EN = CAMERA_CATCH_LOOP; // 切换回图像循环
}

void OutputDisplay_CircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void RunCircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    ReadInput_CircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
    ProcessAlgo_CircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
    OutputDisplay_CircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
}

void ReadInput_AcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void ProcessAlgo_AcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    AcrossTrack(Img_Store_p,Data_Path_p);
    imgSearch_l_r(Img_Store_p,Data_Path_p);
    Function_EN_p->Loop_Kind_EN = CAMERA_CATCH_LOOP; // 切换回图像循环
}

void OutputDisplay_AcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
}

void RunAcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    ReadInput_AcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
    ProcessAlgo_AcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
    OutputDisplay_AcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
}

/**
 * @brief 处理每帧的赛道任务
 *
 * 根据当前循环类型调度不同的赛道处理任务。
 * 这里是主状态机的执行入口：
 * 1. CAMERA_CATCH_LOOP 负责采集后的基础图像处理
 * 2. JUDGE_LOOP 负责识别当前赛道并更新下一阶段状态
 * 3. COMMON_TRACK_LOOP 负责普通赛道的方向/速度计算
 * 4. L_CIRCLE_TRACK_LOOP / R_CIRCLE_TRACK_LOOP 负责圆环补线
 * 5. RIGHT_ACROSS_TRACK_LOOP 负责十字赛道处理
 *
 * @param Img_Store_p 图像存储指针
 * @param Data_Path_p 路径数据指针
 * @param Function_EN_p 功能使能状态指针
 */
void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    switch (Function_EN_p->Loop_Kind_EN)
    {
        case CAMERA_CATCH_LOOP:
        {
            // 图像循环：先完成预处理、二值化和基础寻线，为后续识别提供输入。
            RunCameraCatchTask(Img_Store_p,Data_Path_p,Function_EN_p);
            break;
        }
        case JUDGE_LOOP:
        {
            // 决策循环：根据拐点、弯点和状态位判断当前属于哪类赛道。
            RunJudgeTask(Img_Store_p,Data_Path_p,Function_EN_p);
            break;
        }
        case COMMON_TRACK_LOOP:
        {
            // 普通赛道循环：输出常规路径控制结果，并立即回到图像循环。
            RunCommonTrackTask(Function_EN_p);
            break;
        }
        case L_CIRCLE_TRACK_LOOP:
        case R_CIRCLE_TRACK_LOOP:
        {
            // 圆环循环：根据当前 Circle_Track_Step 执行对应补线策略。
            RunCircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
            break;
        }
        case RIGHT_ACROSS_TRACK_LOOP:
        {
            // 十字循环：执行十字赛道的特殊处理逻辑，然后回到图像循环。
            RunAcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
            break;
        }
        default:
        {
            // 兜底保护：遇到非法状态时回到图像循环，避免状态机卡死。
            Function_EN_p->Loop_Kind_EN = CAMERA_CATCH_LOOP;
            break;
        }
    }
}

/**
 * @brief 应用差速控制
 *
 * 根据视觉处理结果和功能使能状态，计算并设置电机的目标角速度和基础速度
 * 实现上位机控制模式下的差速控制
 *
 * @param Img_Store_p 图像存储指针
 * @param Data_Path_p 路径数据指针
 * @param Function_EN_p 功能使能状态指针
 */
void ApplyDifferentialControl(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    (void)Img_Store_p;
    if (!motorTask || !motorTask->isRunning())
    {
        return;
    }

    // 上位机控制模式：视觉输出 -> 目标角速度差速控制。
    if (Function_EN_p->Control_EN == false)
    {
        judge.ServoDirAngle_Judge(Data_Path_p);
        judge.MotorSpeed_Judge(Img_Store_p,Data_Path_p);
        judge.AngularVelocityTarget_Judge(Data_Path_p);

        motorTask->setBaseSpeed(Data_Path_p->TargetBaseSpeedMps);
        motorTask->setTargetAngularVelocity(Data_Path_p->TargetAngularVelocityDeg);
    }
    else
    {
        // 非上位机控制时，清零角速度目标，防止残留指令。
        motorTask->setTargetAngularVelocity(0.0);
        motorTask->setBaseSpeed(0.0);
    }
}

void FrameTaskAfterRead(Img_Store *Img_Store_p)
{
    if (!Function_EN_s.Game_EN)
    {
        return;
    }
    ProcessTrackTaskPerFrame(Img_Store_p,&Data_Path_s,&Function_EN_s);
    ApplyDifferentialControl(Img_Store_p,&Data_Path_s,&Function_EN_s);
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

    // 4. PID参数设置
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

    // 5. 配置文件同步
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

        g_camera_kind = Function_EN_s.JSON_FunctionConfigData_v[0].Camera_EN;
        Function_EN_s.Game_EN = true;
        // 默认从图像循环开始，等待第一帧完成赛道状态判定后再切换到对应任务。
        Function_EN_s.Loop_Kind_EN = CAMERA_CATCH_LOOP;
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
    motorTask = std::make_unique<MotorControlTask>(
            leftParams,
            rightParams,
            motors.get(),
            &encoder_left,
            &encoder_right,
            &imu,
            0.01
        );
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
        motorTask->setRampLimits(0.8, 0.2);
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
            motorTask->setRampLimits(0.8, 0.2);
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

void pit_callback()
{
    encoder_get_count(ENCODER_1);
    encoder_get_count(ENCODER_2);
}
