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
Function_EN Function_EN_s;
Data_Path Data_Path_s;

ImgProcess imgProcess;
Judge judge;
SYNC Sync;
CameraCalibrationCorrector g_calibration_corrector;

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
    motorTask->enableAngularVelocityControl(true);
    motorTask->enableRampLimiting(true);
    motorTask->setRampLimits(0.8, 0.2);
    motorTask->setWheelbase(0.158);
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
            perfRecorder.record(std::chrono::steady_clock::now() - frameStart,
                                captureCost,
                                undistortCost,
                                undistortExecuted);
            continue;
        }

        tempCapture.saveFrameIfNeeded(Img_Store_s.Img_Color);

        FrameTaskAfterRead(&Img_Store_s);
        perfRecorder.record(std::chrono::steady_clock::now() - frameStart,
                            captureCost,
                            undistortCost,
                            undistortExecuted);
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
