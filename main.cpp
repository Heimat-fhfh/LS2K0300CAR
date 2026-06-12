#define MAKE_MAIN_CPP

#include "main.hpp"
#include "main_runtime.hpp"
#include "AAAtools.h"
#include <iomanip>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    if (!ParseCameraFpsArgument(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (IsVideoSpeedTestMode(argc, argv))
    {
        return RunVideoSpeedTest();
    }

    const bool motorDeadMode = IsMotorDeadMode(argc, argv);

    // 1. 配置参数初始化
    argument_config();
    if (!g_runtime_config_ok)
    {
        cout << "配置同步失败" << endl;
        return EXIT_FAILURE;
    }

    if (motorDeadMode)
    {
        return RunMotorDeadZoneMode();
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

    // 4. 启动电机控制任务  26-27% --> 39-44% CPU占用率 提升了15% 左右
    if (!StartMotorControlTask())
    {
        return EXIT_FAILURE;
    }

    // 5. 初始化摄像头并启动图像采集线程
    VideoCapture Camera;
    CameraInit(Camera, g_camera_kind, 320, 240, g_camera_fps);
    Img_Store Img_Store_s;
    std::thread captureThread;
    CameraCaptureThreadStart(Camera, &Img_Store_s, captureThread);

    TempCaptureSession tempCapture(false);
    PerfWindowRecorder perfRecorder(30, false);

    Function_EN_s.Control_EN = true;
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_s.JSON_FunctionConfigData_v[0];
    // 6. 主循环：图像处理 -> 赛道识别 -> 电机控制
    while (g_running.load() && Function_EN_s.Game_EN)
    {
        if (!tempCapture.handleKeyEvent())
        {
            break;
        }

        CameraImgGet(&Img_Store_s);

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

        // 69-75% , 提升了 50% 左右
        // 独立黑块检测占用5%左右
        // 数据分析占用 64% ，降低了 10% 左右
        // 八邻域占用 63% 几乎没有占用
        // 主要占用为图像预处理
        FrameTaskAfterRead(&Img_Store_s, &Data_Path_s, &Function_EN_s, &imgProcess, &judge);

        // // 归一化偏差并下发电机控制任务
        {
            float errorNorm = static_cast<float>(Data_Path_s.SteerErrorPx) / 160.0f;
            errorNorm = std::max(-1.0f, std::min(1.0f, errorNorm));
            motorTask->setSteerError(static_cast<double>(errorNorm));
            motorTask->setTargetSpeed(Data_Path_s.TargetBaseSpeedMps);
        }

        if(JSON_FunctionConfigData.VideoShow_EN)
        {
            imgProcess.ImgLabel(&Img_Store_s, &Data_Path_s, &Function_EN_s);
            imgProcess.ImgInflectionPointDraw(&Img_Store_s, &Data_Path_s);
            // ImgProcess::ImgBendPointDraw(Img_Store_p,Data_Path_p);
            imgProcess.ImgTransitionScanDraw(&Img_Store_s, &Data_Path_s);
            imgProcess.ImgForwardLine(&Img_Store_s, &Data_Path_s);
            imgProcess.ImgReferenceLine(&Img_Store_s, &Data_Path_s);
            displayMatOnIPS200(Img_Store_s.Img_Track);
            ips200_show_int(0,200,Data_Path_s.SteerErrorPx,3);
        }

        if (!Function_EN_s.Control_EN)
        {
            continue;
        }

        cout << "EPx,TBS: " << Data_Path_s.SteerErrorPx << ",\t"
        << fixed << setprecision(3)
        << Data_Path_s.TargetBaseSpeedMps << endl;
    }

    perfRecorder.flush();
    tempCapture.printSummary();

    CameraCaptureThreadStop(&Img_Store_s, captureThread);
    Camera.release();

    if (motorTask)
    {
        motorTask->setTargetSpeed(0.0);
        motorTask->stop();
    }

    return 0;
}
