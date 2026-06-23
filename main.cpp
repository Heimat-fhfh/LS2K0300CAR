#define MAKE_MAIN_CPP

#include "common/main.hpp"
#include "common/main_runtime.hpp"
#include "common/AAAtools.h"
#include "vision/Image_Process.h"
#include "network/seekfree_udp.h"


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    printf("选择配置(0/1/2): ");
    if (!ParseCameraFpsArgument(argc, argv)){return EXIT_FAILURE;}
    if (IsVideoSpeedTestMode(argc, argv)){return RunVideoSpeedTest();}
    const bool motorDeadMode = IsMotorDeadMode(argc, argv);

    // 1. 配置参数初始化
    argument_config();
    if (!g_runtime_config_ok)
{cout << "配置同步失败" << endl;return EXIT_FAILURE;}

    if (motorDeadMode){return RunMotorDeadZoneMode();}

    // 2. 硬件设备初始化
    if (main_init_task() == EXIT_SUCCESS){printf("<硬件> 初始化成功\n");}
    else{printf("<硬件> 初始化失败\n");return EXIT_FAILURE;}

    // 3. 功能测试
    if (main_test_task(test_config) != EXIT_SUCCESS){cout << "功能测试失败" << endl;return EXIT_FAILURE;}

    // 4. 启动电机控制任务  26-27% --> 39-44% CPU占用率 提升了15% 左右
    if (!StartMotorControlTask()){return EXIT_FAILURE;}

    // 5. 初始化摄像头并启动图像采集线程
    VideoCapture Camera;
    CameraInit(Camera, g_camera_kind, 160, 120, g_camera_fps);
    
    Img_Store Img_Store_s;
    std::thread captureThread;
    CameraCaptureThreadStart(Camera, &Img_Store_s, captureThread);

    TempCaptureSession tempCapture(false);

    Function_EN_s.Control_EN = true;
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_s.JSON_FunctionConfigData_v[0];

    // 启动状态显示
    printf("[配置] 圆环最大帧数: %d\n", Data_Path_s.JSON_TrackConfigData_v[0].CircleMaxFrames);
    printf("[状态] IPS200显示: %s | UDP图像上传: %s\n",JSON_FunctionConfigData.IPS200_Show_EN ? "开启" : "关闭",
           JSON_FunctionConfigData.UDP_Image_Upload_EN ? "开启" : "关闭");

    // 6. 主循环：图像处理 -> 赛道识别 -> 电机控制
    uint64_t frame_count = 0;
    auto t_fps_begin = std::chrono::steady_clock::now();
    while (g_running.load() && Function_EN_s.Game_EN)
    {
        if (!tempCapture.handleKeyEvent()){break;}

        CameraImgGet(&Img_Store_s);

        if (!g_running.load()){printf("退出信号已接收，正在停止摄像头捕获线程...\n");break;}
        if (Img_Store_s.Img_Color.empty()){printf("Warning: Captured image is empty, skipping this frame.\n");continue;}

        tempCapture.saveFrameIfNeeded(Img_Store_s.Img_Color);

        FrameTaskAfterRead(&Img_Store_s, &Data_Path_s, &Function_EN_s, &imgProcess, &judge);
        frame_count++;

        // 帧率统计 (每100帧打印一次)
        if (frame_count % 100 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_fps_begin);
            if (elapsed.count() > 0) {
                double fps = frame_count * 1000.0 / elapsed.count();
                printf("<帧率> %.1f FPS\n", fps);
            }
        }

        // 十字状态蜂鸣器
        {
            static bool acrossBuzzerOn = false;
            const bool inAcross = (ImageStatus.Road_type == Cross ||
                                   ImageStatus.Road_type == Cross_ture);
            if (inAcross && !acrossBuzzerOn)
            {
                GetBuzzer().customPattern({60}, {60}, 999);
                acrossBuzzerOn = true;
            }
            else if (!inAcross && acrossBuzzerOn)
            {
                GetBuzzer().stop();
                acrossBuzzerOn = false;
            }
        }

        // 圆环状态蜂鸣器：4声短鸣循环
        {
            static bool circleBuzzerOn = false;
            const bool inCircle = (ImageFlag.image_element_rings_flag >= 1
                                && ImageFlag.image_element_rings_flag <= 9);
            if (inCircle && !circleBuzzerOn)
            {
                GetBuzzer().customPattern({60,60,60,60}, {60,60,60,400}, 999);
                circleBuzzerOn = true;
            }
            else if (!inCircle && circleBuzzerOn)
            {
                GetBuzzer().stop();
                circleBuzzerOn = false;
            }
        }

        // 出界保护：连续丢线超过指定帧数则停止电机，2声短鸣，按KEY_0恢复
        {
            static int offline_accum = 0;       // 连续丢线帧数计数
            const int OFFLINE_FRAME_MAX = 10;   // 连续丢线帧数阈值，超过则触发出界保护
            if (ImageStatus.OFFLine >= 55)
                offline_accum++;
            else
                offline_accum = 0;

            if (offline_accum >= OFFLINE_FRAME_MAX)
            {
                motorTask->emergencyStop();
                GetBuzzer().customPattern({60, 60}, {60, 400}, 999);
                printf("[OUT_OF_BOUNDS] 丢线出界！OFFLine=%d 连续 %d 帧。按 KEY_0 复位。\n",
                       ImageStatus.OFFLine, offline_accum);

                while (g_running.load())
                {
                    CameraImgGet(&Img_Store_s);
                    if (gpio_get_level(KEY_0) == 0)
                    {
                        offline_accum = 0;
                        GetBuzzer().stop();
                        motorTask->clearEmergencyStop();
                        printf("[OUT_OF_BOUNDS] 已复位，恢复巡线。\n");
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }
        }

        // // 归一化偏差并下发电机控制任务
        {
            float errorNorm = static_cast<float>(Data_Path_s.SteerErrorPx) / 40.0f;
            errorNorm = std::max(-1.0f, std::min(1.0f, errorNorm));
            motorTask->setSteerError(static_cast<double>(errorNorm));
            motorTask->setTargetSpeed(Data_Path_s.TargetBaseSpeedMps);
        }

        // IPS200 屏幕显示（my_zf 配套 80x60）
        if (JSON_FunctionConfigData.IPS200_Show_EN)
        {
            displayMyZFOnIPS200();
        }

        // UDP 图像上传（INCLUDE_BOUNDARY_TYPE=3）
        if (JSON_FunctionConfigData.UDP_Image_Upload_EN)
        {
            static uint8_t img_buf[80 * 60];
            static uint8_t left_x_buf[BOUNDARY_NUM * 2];
            static uint8_t left_y_buf[BOUNDARY_NUM * 2];
            static uint8_t center_x_buf[BOUNDARY_NUM * 2];
            static uint8_t center_y_buf[BOUNDARY_NUM * 2];
            static uint8_t right_x_buf[BOUNDARY_NUM * 2];
            static uint8_t right_y_buf[BOUNDARY_NUM * 2];

            // fill grayscale image from Image_Use
            for (int i = 0; i < 60; i++)
                for (int j = 0; j < 80; j++)
                    img_buf[i * 80 + j] = Image_Use[i][j];

            // fill boundary arrays (XY, 16-bit, bottom-to-top for SEEKFREE assistant)
            int dot = 0;
            uint8_t off = ImageStatus.OFFLine;
            for (int i = off; i < 60 && dot < BOUNDARY_NUM; i++, dot++) {
                uint16_t x = ImageDeal[i].LeftBorder;
                uint16_t y = 59 - i;  // flip Y for display
                left_x_buf[dot * 2]     = x & 0xFF;
                left_x_buf[dot * 2 + 1] = (x >> 8) & 0xFF;
                left_y_buf[dot * 2]     = y & 0xFF;
                left_y_buf[dot * 2 + 1] = (y >> 8) & 0xFF;
            }
            int dot_l = dot;

            dot = 0;
            for (int i = off; i < 60 && dot < BOUNDARY_NUM; i++, dot++) {
                uint16_t x = ImageDeal[i].Center;
                uint16_t y = 59 - i;
                center_x_buf[dot * 2]     = x & 0xFF;
                center_x_buf[dot * 2 + 1] = (x >> 8) & 0xFF;
                center_y_buf[dot * 2]     = y & 0xFF;
                center_y_buf[dot * 2 + 1] = (y >> 8) & 0xFF;
            }
            int dot_c = dot;

            dot = 0;
            for (int i = off; i < 60 && dot < BOUNDARY_NUM; i++, dot++) {
                uint16_t x = ImageDeal[i].RightBorder;
                uint16_t y = 59 - i;
                right_x_buf[dot * 2]     = x & 0xFF;
                right_x_buf[dot * 2 + 1] = (x >> 8) & 0xFF;
                right_y_buf[dot * 2]     = y & 0xFF;
                right_y_buf[dot * 2 + 1] = (y >> 8) & 0xFF;
            }
            int dot_r = dot;

            int max_dot = dot_l;
            if (dot_c > max_dot) max_dot = dot_c;
            if (dot_r > max_dot) max_dot = dot_r;
            if (max_dot < 1) max_dot = 1;

            seekfree_udp_send_frame(img_buf, 80, 60,
                                    left_x_buf, left_y_buf,
                                    center_x_buf, center_y_buf,
                                    right_x_buf, right_y_buf,
                                    max_dot);
        }

    }

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
