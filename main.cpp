
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
#include "base.h"
#include "common_system.h"
#include "common_program.h"
#include "register.h"
#include "stdio.h"
#include "zf_common_headfile.h"
#include "display_show.h"
#include "httplib.h"



// HTTP服务器线程函数
void run_http_server() {
    httplib::Server svr;
    
    // 设置根路径处理函数
    svr.Get("/hi", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("Hello World", "text/plain");
    });
    
    // 启动服务器，监听80端口
    printf("启动HTTP服务器，监听端口80...\n");
    if (!svr.listen("0.0.0.0", 80)) {
        printf("HTTP服务器启动失败！可能原因：端口已被占用或权限不足。\n");
    }
}

int main(int argc, char *argv[])
{
    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    ips200_init("/dev/fb0");

    // 启动HTTP服务器线程
    std::thread http_thread(run_http_server);
    http_thread.detach(); // 分离线程，让它在后台运行

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

    while(1)
    {
        Camera.read(frame);
        // displayMatOnIPS200(frame); 
    }
    
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
