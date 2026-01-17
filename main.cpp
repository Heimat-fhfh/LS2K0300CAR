
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

int main() {
    
    setbuf(stdout, NULL);

    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    ips200_init("/dev/fb0");
    
    // 将web服务分离出来单独运行
    std::thread web_thread(start_web_server);
    web_thread.detach();
    printf("Web server started successfully.\n");

    VideoCapture Camera;
    Camera.open("/dev/video0",CAP_V4L2);
    Camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); 
    Camera.set(CAP_PROP_FRAME_WIDTH, 320);      // 帧宽
    Camera.set(CAP_PROP_FRAME_HEIGHT, 240);     // 帧高
    Camera.set(CAP_PROP_FPS, 120);              // 帧率

    double actualWidth = Camera.get(CAP_PROP_FRAME_WIDTH); 
    double actualHeight = Camera.get(CAP_PROP_FRAME_HEIGHT); 
    double actualFps = Camera.get(CAP_PROP_FPS); 
    printf("摄像头配置信息：\n"); 
    printf("分辨率：%.0fx%.0f\n", actualWidth, actualHeight); 
    printf("帧率：%.0f FPS\n", actualFps);

    Mat frame;

    while(running)
    {
        Camera.read(frame);
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
