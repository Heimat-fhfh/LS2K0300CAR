
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <math.h>		
#include <stdlib.h>
#include <thread>
#include <atomic>
#include "base.h"
#include "common_system.h"
#include "common_program.h"
#include "register.h"
#include "stdio.h"
#include "zf_common_headfile.h"
#include "display_show.h"
#include "web_server.h"
#include "libimage_process.h"

using namespace std;
using namespace cv;

JSON_PIDConfigData  JSON_PIDConfigData_c;
JSON_PIDConfigData  *JSON_PIDConfigData_p = &JSON_PIDConfigData_c;

Function_EN         Function_EN_c;
Function_EN         *Function_EN_p = &Function_EN_c;

Data_Path           Data_Path_c;
Data_Path           *Data_Path_p = &Data_Path_c;

Img_Store           Img_Store_c; 
Img_Store           *Img_Store_p = &Img_Store_c;

ImgProcess imgProcess;
Judge judge;
SYNC Sync;

int encoder_left = 0;
int encoder_right = 0;

int main() {
    
    setbuf(stdout, NULL);

    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    
    pit_ms_init(10, pit_callback);
    
    ips200_init("/dev/fb0");
    
    // 显示IP地址
    display_ip_address(0, 181);
    printf("IP address displayed on screen.\n");
    
    // 将web服务分离出来单独运行
    std::thread web_thread(start_web_server);
    web_thread.detach();
    printf("Web server started successfully.\n");

    Sync.ConfigData_SYNC(Data_Path_p,Function_EN_p,JSON_PIDConfigData_p);
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    
    VideoCapture Camera;
    CameraInit(Camera,JSON_FunctionConfigData.Camera_EN,320,240,60);
    Function_EN_p -> Game_EN = true;
    Function_EN_p -> Loop_Kind_EN = CAMERA_CATCH_LOOP;

    Mat frame;

    while(running && Function_EN_p -> Game_EN == true)
    {
        
        while( Function_EN_p -> Loop_Kind_EN == CAMERA_CATCH_LOOP)
        {
            Data_Path_p -> JSON_TrackConfigData_v[0].Forward = Data_Path_p -> JSON_TrackConfigData_v[0].Default_Forward;
            Camera >> Img_Store_p -> Img_Color;

            imgProcess.imgPreProc(Img_Store_p,Data_Path_p,Function_EN_p); // 图像预处理
            memcpy(Img_Store_p->bin_image[0], Img_Store_p->Img_OTSU.data, image_h * image_w * sizeof(uint8));
            imgSearch_l_r(Img_Store_p,Data_Path_p);   // 边线八邻域寻线

            imgProcess.ImgLabel(Img_Store_p,Data_Path_p,Function_EN_p);
            displayMatOnIPS200(Img_Store_p->Img_Track);

            // Img_Store_p -> ImgNum++;
            // Function_EN_p -> Loop_Kind_EN = JUDGE_LOOP;
        }

        while( Function_EN_p -> Loop_Kind_EN == JUDGE_LOOP )
        {
            Function_EN_p -> Loop_Kind_EN = judge.TrackKind_Judge(Img_Store_p,Data_Path_p,Function_EN_p);  // 切换至赛道循环
        }

        while( Function_EN_p -> Loop_Kind_EN == COMMON_TRACK_LOOP )
        {
            Function_EN_p -> Loop_Kind_EN = CAMERA_CATCH_LOOP;
        }

        while( Function_EN_p -> Loop_Kind_EN == L_CIRCLE_TRACK_LOOP || Function_EN_p -> Loop_Kind_EN == R_CIRCLE_TRACK_LOOP)
        {
            switch(Data_Path_p -> Circle_Track_Step)
            {
                case IN_PREPARE:
                {
                    CircleTrack_Step_IN_Prepare(Img_Store_p,Data_Path_p);   // 准备入环补线
                    break;
                }
                case IN:
                {
                    CircleTrack_Step_IN(Img_Store_p,Data_Path_p);   // 入环补线
                    break;
                }
                case OUT:
                {
                    CircleTrack_Step_OUT(Img_Store_p,Data_Path_p);   // 出环补线
                    break;
                }
            }
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            Function_EN_p -> Loop_Kind_EN = CAMERA_CATCH_LOOP; // 切换至串口发送循环
        }
        
        while( Function_EN_p -> Loop_Kind_EN == ACROSS_TRACK_LOOP )
        {
            AcrossTrack(Img_Store_p,Data_Path_p);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            Function_EN_p -> Loop_Kind_EN = CAMERA_CATCH_LOOP; // 切换至串口发送循环
        }

        // Camera.read(frame);
        // displayMatOnIPS200(frame); 
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    Camera.release();
    
    printf("Web服务已停止，主线程退出\n");
    // fflush(stdout);
    
    return 0;
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    
    // 设置运行标志为false，让主循环退出
    running = false;
    
    // 停止web服务器
    if (g_server) {
        g_server->stop();
    }
    
    // 给一点时间让线程清理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    exit(0);
}

void cleanup()
{
    printf("程序退出，执行清理操作\n");
}

void pit_callback()
{
    imu660ra_get_acc();
    imu660ra_get_gyro();
    encoder_left  = encoder_get_count(ENCODER_1);
    encoder_right = encoder_get_count(ENCODER_2);
}
