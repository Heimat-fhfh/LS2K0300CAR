#define MAKE_MAIN_CPP

#include "main_runtime.hpp"
#include "camera_calibration.h"

#include <fstream>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::this_thread;

IMUDevice imu;
std::unique_ptr<DualMotorController> motors;
Encoder encoder_left("/dev/zf_encoder_1");
Encoder encoder_right("/dev/zf_encoder_2", true);
Control::PID::Parameters diffOuterParams, diffInnerParams;
Control::IncrementalPID::Parameters speedIncrParams;
std::unique_ptr<MotorControlTask> motorTask;
MainTestConfig test_config;
std::atomic<bool> g_running(true);
bool g_runtime_config_ok = false;
CameraKind g_camera_kind = CameraKind::VIDEO_0;
int g_camera_fps = 120;
bool g_calibration_enabled = false;
bool g_simple_tracking_enabled = false;

JSON_PIDConfigData JSON_PIDConfigData_s;
JSON_DifferentialPDConfigData JSON_DifferentialPDConfigData_s;
JSON_AngularVelocityPIDConfigData JSON_AngularVelocityPIDConfigData_s;
JSON_SpeedIncrementalPIConfigData JSON_SpeedIncrementalPIConfigData_s;
JSON_VehicleConfigData JSON_VehicleConfigData_s;
Function_EN Function_EN_s;
Data_Path Data_Path_s;

ImgProcess imgProcess;
Judge judge;
SYNC Sync;
CameraCalibrationCorrector g_calibration_corrector;

Buzzer& GetBuzzer()
{
    static Buzzer buzzer;
    return buzzer;
}

namespace
{
constexpr int kVideoSpeedTestWidth = 320;
constexpr int kVideoSpeedTestHeight = 240;
constexpr int kVideoSpeedTestReportFrames = 30;
constexpr double kVideoSpeedTestAutoExposureMode = 3.0;

bool ParsePositiveInt(const std::string& text, int* value)
{
    if (text.empty() || value == nullptr)
    {
        return false;
    }

    char* end = nullptr;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (*end != '\0' || parsed <= 0 || parsed > 1000)
    {
        return false;
    }

    *value = static_cast<int>(parsed);
    return true;
}

std::string FourccToString(double fourccValue)
{
    const int fourcc = static_cast<int>(fourccValue);
    std::string text(4, ' ');
    text[0] = static_cast<char>(fourcc & 0xFF);
    text[1] = static_cast<char>((fourcc >> 8) & 0xFF);
    text[2] = static_cast<char>((fourcc >> 16) & 0xFF);
    text[3] = static_cast<char>((fourcc >> 24) & 0xFF);
    return text;
}

void VideoSpeedTestSignalHandler(int signum)
{
    (void)signum;
    g_running.store(false);
}

double ReadAverageSpeed(Encoder& encoder, int samples, int delayMs)
{
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < samples; ++i)
    {
        try
        {
            sum += std::abs(encoder.readSpeed());
            ++count;
        }
        catch (const EncoderException&)
        {
            // Ignore single read failures during dead-zone probing.
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
    }
    return (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
}

double FindMotorDeadZone(MotorController& motor,
                         Encoder& encoder,
                         double speedThreshold,
                         double step,
                         int settleMs)
{
    motor.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for (double cmd = step; cmd <= 1.0 + 1e-6; cmd += step)
    {
        motor.setSpeed(static_cast<float>(cmd));
        std::this_thread::sleep_for(std::chrono::milliseconds(settleMs));

        const double avgSpeed = ReadAverageSpeed(encoder, 3, 50);
        if (avgSpeed >= speedThreshold)
        {
            motor.stop();
            return cmd;
        }
    }

    motor.stop();
    return 1.0;
}

bool UpdateConfigDeadZones(const std::string& path, double leftDeadZone, double rightDeadZone, std::string* error)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        if (error)
        {
            *error = "Cannot open config file: " + path;
        }
        return false;
    }

    nlohmann::json cfg;
    try
    {
        ifs >> cfg;
    }
    catch (const std::exception& e)
    {
        if (error)
        {
            *error = std::string("JSON parse failed: ") + e.what();
        }
        return false;
    }

    if (!cfg.is_object())
    {
        if (error)
        {
            *error = "Config JSON root is not an object";
        }
        return false;
    }

    cfg["MOTOR_PWM_DEAD_ZONE_LEFT"] = leftDeadZone;
    cfg["MOTOR_PWM_DEAD_ZONE_RIGHT"] = rightDeadZone;

    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        if (error)
        {
            *error = "Cannot write config file: " + path;
        }
        return false;
    }
    ofs << cfg.dump(4);
    return true;
}
}

bool ParseCameraFpsArgument(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        std::string valueText;

        if (arg == "--camera-fps" || arg == "--fps")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "[Args] " << arg << " 缺少帧率数值" << std::endl;
                return false;
            }
            valueText = argv[++i];
        }
        else
        {
            const std::string prefixes[] = {
                "--camera-fps=",
                "--fps=",
                "CameraFps=",
                "camera_fps=",
                "fps=",
            };

            for (const std::string& prefix : prefixes)
            {
                if (arg.rfind(prefix, 0) == 0)
                {
                    valueText = arg.substr(prefix.size());
                    break;
                }
            }
        }

        if (!valueText.empty())
        {
            int parsedFps = 0;
            if (!ParsePositiveInt(valueText, &parsedFps))
            {
                std::cerr << "[Args] 无效摄像头帧率: " << valueText
                          << "，请输入 1-1000 的整数" << std::endl;
                return false;
            }

            g_camera_fps = parsedFps;
            std::cout << "[Args] 摄像头请求帧率: " << g_camera_fps << " FPS" << std::endl;
        }
    }

    return true;
}

bool IsVideoSpeedTestMode(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "VideoSpeedTest")
        {
            return true;
        }
    }
    return false;
}

bool IsMotorDeadMode(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "MotorDead")
        {
            return true;
        }
    }
    return false;
}

int RunVideoSpeedTest()
{
    std::cout << "[VideoSpeedTest] 启动 YUY2 视频读取速度检测" << std::endl;
    signal(SIGINT, VideoSpeedTestSignalHandler);
    g_running.store(true);

    const char* cameraPath = "/dev/video0";
    VideoCapture camera;
    camera.open(cameraPath);

    if (!camera.isOpened())
    {
        std::cerr << "[VideoSpeedTest] error: no find uvc camera ." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "[VideoSpeedTest] find uvc camera Successfully." << std::endl;

    camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', '2'));
    camera.set(CAP_PROP_FRAME_WIDTH, kVideoSpeedTestWidth);
    camera.set(CAP_PROP_FRAME_HEIGHT, kVideoSpeedTestHeight);
    camera.set(CAP_PROP_FPS, g_camera_fps);
    camera.set(CAP_PROP_AUTO_EXPOSURE, kVideoSpeedTestAutoExposureMode);

    std::cout << "[VideoSpeedTest] get uvc width = " << camera.get(CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "[VideoSpeedTest] get uvc height = " << camera.get(CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "[VideoSpeedTest] get uvc fps = " << camera.get(CAP_PROP_FPS) << std::endl;
    std::cout << "[VideoSpeedTest] get uvc fourcc = " << FourccToString(camera.get(CAP_PROP_FOURCC)) << std::endl;
    std::cout << "[VideoSpeedTest] get uvc auto exposure mode = "
              << camera.get(CAP_PROP_AUTO_EXPOSURE) << std::endl;

    uint64_t frameCount = 0;
    uint64_t failedReadCount = 0;
    uint64_t emptyFrameCount = 0;
    int64_t windowReadCostUs = 0;
    auto windowStart = std::chrono::steady_clock::now();

    while (g_running.load())
    {
        Mat frame;
        const auto readStart = std::chrono::steady_clock::now();
        const bool ok = camera.read(frame);
        const auto readCostUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - readStart)
                                    .count();

        windowReadCostUs += readCostUs;
        if (!ok)
        {
            ++failedReadCount;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (frame.empty())
        {
            ++emptyFrameCount;
            continue;
        }

        ++frameCount;
        if (frameCount % kVideoSpeedTestReportFrames == 0)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(now - windowStart).count();
            const double fps = (elapsedUs > 0)
                                   ? (kVideoSpeedTestReportFrames * 1000000.0 / static_cast<double>(elapsedUs))
                                   : 0.0;
            const double avgReadUs = windowReadCostUs / static_cast<double>(kVideoSpeedTestReportFrames);

            std::cout << "[VideoSpeedTest] frame=" << frameCount
                      << " fps=" << fixed << setprecision(2) << fps
                      << " avg_read_us=" << fixed << setprecision(1) << avgReadUs
                      << " last_read_us=" << readCostUs
                      << " size=" << frame.cols << "x" << frame.rows
                      << " type=" << frame.type()
                      << " channels=" << frame.channels()
                      << " failed=" << failedReadCount
                      << " empty=" << emptyFrameCount
                      << std::endl;

            windowStart = now;
            windowReadCostUs = 0;
        }
    }

    camera.release();
    std::cout << "[VideoSpeedTest] 停止，总帧数=" << frameCount
              << " failed=" << failedReadCount
              << " empty=" << emptyFrameCount << std::endl;
    return EXIT_SUCCESS;
}

int RunMotorDeadZoneMode()
{
    if (!motors)
    {
        std::cerr << "[MotorDead] motors 未初始化" << std::endl;
        return EXIT_FAILURE;
    }

    motors->setPwmDeadZones(0.0f, 0.0f);
    motors->setMaxDutyLimits(static_cast<float>(JSON_VehicleConfigData_s.motorMaxDuty));

    const double speedThreshold = 0.01;
    const double step = 0.01;
    const int settleMs = 200;

    std::cout << "[MotorDead] speed_threshold=" << speedThreshold
              << " step=" << step << std::endl;

    const double leftDead = FindMotorDeadZone(
        motors->getLeftMotor(),
        encoder_left,
        speedThreshold,
        step,
        settleMs);

    const double rightDead = FindMotorDeadZone(
        motors->getRightMotor(),
        encoder_right,
        speedThreshold,
        step,
        settleMs);

    motors->stopAll();

    JSON_VehicleConfigData_s.motorPwmDeadZoneLeft = leftDead;
    JSON_VehicleConfigData_s.motorPwmDeadZoneRight = rightDead;
    motors->setPwmDeadZones(static_cast<float>(leftDead), static_cast<float>(rightDead));

    const std::string configPath = Sync.GetConfigFilePath();
    std::string error;
    if (!UpdateConfigDeadZones(configPath, leftDead, rightDead, &error))
    {
        std::cerr << "[MotorDead] 写入配置失败: " << error << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[MotorDead] left=" << leftDead
              << " right=" << rightDead
              << " -> " << configPath << std::endl;
    return EXIT_SUCCESS;
}

bool StartMotorControlTask()
{
    if (!motorTask)
    {
        std::cerr << "[Motor] motorTask 未创建，无法启动主循环" << std::endl;
        return false;
    }

    motorTask->setMotorMaxDuty(JSON_VehicleConfigData_s.motorMaxDuty);
    motors->setPwmDeadZones(static_cast<float>(JSON_VehicleConfigData_s.motorPwmDeadZoneLeft),
                            static_cast<float>(JSON_VehicleConfigData_s.motorPwmDeadZoneRight));

    motorTask->enableCollisionProtection(JSON_VehicleConfigData_s.collisionProtectEnable);
    motorTask->setCollisionImuJerkThreshold(JSON_VehicleConfigData_s.collisionImuJerkThreshold);
    motorTask->setCollisionStallThresholds(
        JSON_VehicleConfigData_s.collisionStallDutyThreshold,
        JSON_VehicleConfigData_s.collisionStallSpeedThreshold,
        JSON_VehicleConfigData_s.collisionStallCycles);
    motorTask->setCollisionKeyConfig(
        JSON_VehicleConfigData_s.collisionResetKey,
        JSON_VehicleConfigData_s.collisionBumperKey);
    motorTask->configureCollisionGpio({KEY_0, KEY_1, KEY_2, KEY_3});

    motorTask->enableLowPassFilter(true);
    motorTask->setSpeedFilterTimeConstant(JSON_VehicleConfigData_s.lpfSpeedTau);
    motorTask->setAngularFilterTimeConstant(JSON_VehicleConfigData_s.lpfAngularTau);
    motorTask->setSteerErrorFilterTimeConstant(0.04);

    motorTask->setRampRates(JSON_VehicleConfigData_s.rampAccelRate,
                            JSON_VehicleConfigData_s.rampDecelRate);
    motorTask->enableRampControl(true);
    motorTask->enableDiffOutputRamp(JSON_VehicleConfigData_s.diffOutputRampEnable);
    motorTask->setDiffOutputRampRates(JSON_VehicleConfigData_s.diffOutputRampAccelRate,
                                      JSON_VehicleConfigData_s.diffOutputRampDecelRate);

    motorTask->setCurvatureSpeedGain(JSON_VehicleConfigData_s.curvatureSpeedGain);
    motorTask->setCurvatureSpeedMin(JSON_VehicleConfigData_s.curvatureSpeedMin);

    motorTask->start();
    return true;
}

void argument_config(void)
{
    test_config.buzzer_test = false;
    test_config.imu_test = false;
    test_config.motor_test = false;                 // true
    test_config.angular_velocity_test = false;
    test_config.encoder_test = false;

    Buzzer& buzzer = GetBuzzer();
    buzzer.setShortDuration(60)
            .setLongDuration(300)
            .setIntervalDuration(120);

    motors = std::make_unique<DualMotorController>();

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
        }

        JSON_DifferentialPDConfigData_s = Data_Path_s.JSON_DifferentialPDConfigData_v[0];
        JSON_AngularVelocityPIDConfigData_s = Data_Path_s.JSON_AngularVelocityPIDConfigData_v[0];
        JSON_SpeedIncrementalPIConfigData_s = Data_Path_s.JSON_SpeedIncrementalPIConfigData_v[0];
        JSON_VehicleConfigData_s = Data_Path_s.JSON_VehicleConfigData_v[0];

        diffOuterParams.Kp = JSON_DifferentialPDConfigData_s.Kp;
        diffOuterParams.Kd = JSON_DifferentialPDConfigData_s.Kd;
        diffOuterParams.limitP = JSON_DifferentialPDConfigData_s.limitP;
        diffOuterParams.limitD = JSON_DifferentialPDConfigData_s.limitD;
        diffOuterParams.limitOutput = JSON_DifferentialPDConfigData_s.limitOutput;
        diffOuterParams.Ki = 0.0;

        diffInnerParams.Kp = JSON_AngularVelocityPIDConfigData_s.Kp;
        diffInnerParams.Ki = JSON_AngularVelocityPIDConfigData_s.Ki;
        diffInnerParams.Kd = JSON_AngularVelocityPIDConfigData_s.Kd;
        diffInnerParams.limitP = JSON_AngularVelocityPIDConfigData_s.limitP;
        diffInnerParams.limitI = JSON_AngularVelocityPIDConfigData_s.limitI;
        diffInnerParams.limitD = JSON_AngularVelocityPIDConfigData_s.limitD;
        diffInnerParams.limitOutput = JSON_AngularVelocityPIDConfigData_s.limitOutput;
        diffInnerParams.limitIMin = JSON_AngularVelocityPIDConfigData_s.limitIMin;
        diffInnerParams.enableAntiWindup = JSON_AngularVelocityPIDConfigData_s.enableAntiWindup;

        speedIncrParams.Kp = JSON_SpeedIncrementalPIConfigData_s.Kp;
        speedIncrParams.Ki = JSON_SpeedIncrementalPIConfigData_s.Ki;
        speedIncrParams.Kd = JSON_SpeedIncrementalPIConfigData_s.Kd;
        speedIncrParams.limitOutput = JSON_SpeedIncrementalPIConfigData_s.limitOutput;

        g_camera_kind = Function_EN_s.JSON_FunctionConfigData_v[0].Camera_EN;
        Function_EN_s.Game_EN = true;
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

    const double ctrlPeriod = g_runtime_config_ok ? JSON_VehicleConfigData_s.controlPeriod : 0.01;
    motorTask = std::make_unique<MotorControlTask>(
            diffOuterParams,
            diffInnerParams,
            speedIncrParams,
            motors.get(),
            &encoder_left,
            &encoder_right,
            &imu,
            ctrlPeriod
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
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    setbuf(stdout, NULL);
    ips200_init("/dev/fb0");

    display_ip_address(0, 181);
    printf("IP address displayed on screen.\n");

    if (udp_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("[UDP] \u521d\u59cb\u5316\u6210\u529f, %s:%d\n", SERVER_IP, PORT);
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    uint8 temp_str[] = "UDP IS READY.\r\n";
    udp_dev.send_data(temp_str, sizeof(temp_str));

    if (!imu.initialize())
    {
        printf("Failed to initialize IMU device\n");
        return EXIT_FAILURE;
    }

    printf("[IMU] \u521d\u59cb\u5316\u6210\u529f\n");
    return EXIT_SUCCESS;
}

int main_test_task(const MainTestConfig& test_config)
{
    if (test_config.buzzer_test)
    {
        try
        {
            Buzzer& buzzer = GetBuzzer();
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
                    printf("  Raw Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n",
                           raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z);
                    printf("  Comp Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n",
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
                printf("  Gyro: X=%6d(%.2f rad/s), Y=%6d(%.2f rad/s), Z=%6d(%.2f rad/s)\n", data.gyro_x, unit_data.gyro_x, data.gyro_y, unit_data.gyro_y, data.gyro_z, unit_data.gyro_z);

                printf("\n");
                sleep_for(milliseconds(20));
            }
        }
    }

    if (test_config.motor_test)
    {
        motorTask->start();
        try
        {
            printf("\n=== 电机基本功能测试 ===\n");
            motorTask->setTargetSpeed(1.0);
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(20));

            motorTask->setTargetSpeed(3.0);
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(7));

            motorTask->setTargetSpeed(5.0);
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(7));

            motorTask->setTargetSpeed(1.0);
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(7));

            printf("\n=== 差速测试 ===\n");
            motorTask->setSteerError(0.5);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            motorTask->setTargetSpeed(0.0);
            motorTask->stop();
            printf("\n电机测试完成\n");

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
        printf("\n=== 串级差速控制测试 ===\n");
        try
        {
            motorTask->start();

            motorTask->setTargetSpeed(0.5);
            printf("\nTest 1: 直线前进 (偏差=0)\n");
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            printf("\nTest 2: 右偏 (SteerError=0.3)\n");
            motorTask->setSteerError(0.3);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            printf("\nTest 3: 左偏 (SteerError=-0.3)\n");
            motorTask->setSteerError(-0.3);
            std::this_thread::sleep_for(std::chrono::seconds(5));

            printf("\nTest 4: 大角度右偏 (SteerError=0.8)\n");
            motorTask->setSteerError(0.8);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            printf("\nTest 5: 回正 (SteerError=0)\n");
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            printf("\nTest 6: 速度切换 1.0m/s\n");
            motorTask->setTargetSpeed(1.0);
            motorTask->setSteerError(0.0);
            std::this_thread::sleep_for(std::chrono::seconds(4));

            motorTask->stop();
            printf("\n串级差速控制测试完成.\n");
        }
        catch (const std::exception& e)
        {
            std::cerr << "串级差速控制测试错误: " << e.what() << std::endl;
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

    motorTask->stop();
    return EXIT_SUCCESS;
}
