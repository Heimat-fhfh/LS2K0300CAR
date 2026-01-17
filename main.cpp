
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
    CameraInit(Camera,JSON_FunctionConfigData.Camera_EN,120);
    Function_EN_p -> Game_EN = true;
    Function_EN_p -> Loop_Kind_EN = CAMERA_CATCH_LOOP;

    Mat frame;

    


    while(running && Function_EN_p -> Game_EN == true)
    {
        Camera.read(frame);
        
        // while( Function_EN_p -> Loop_Kind_EN == CAMERA_CATCH_LOOP)
        // {
        // }

        // while( Function_EN_p -> Loop_Kind_EN == JUDGE_LOOP )
        // {
        // }

        // while( Function_EN_p -> Loop_Kind_EN == COMMON_TRACK_LOOP )
        // {
        // }

        // while( Function_EN_p -> Loop_Kind_EN == L_CIRCLE_TRACK_LOOP || Function_EN_p -> Loop_Kind_EN == R_CIRCLE_TRACK_LOOP)
        // {
        // }
        
        // while( Function_EN_p -> Loop_Kind_EN == ACROSS_TRACK_LOOP )
        // {
        // }

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
