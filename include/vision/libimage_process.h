#include "common/common_system.h"
#include "common/common_program.h"

#ifndef _LIBIMAGE_PROCESS_H_
#define _LIBIMAGE_PROCESS_H_

using namespace std;

/*
    CameraInit说明
    摄像头初始化
    @参数说明
    Camera 传入VideoCapture类
    Camera_EN 相机使能
    FPS 摄像头帧率
*/
void CameraInit(cv::VideoCapture& Camera,CameraKind Camera_EN,int Width,int Height,int FPS);


/*
    摄像头图像采集(多线程)
    @参数说明
    Camera 传入VideoCapture类
    Img_Store_p 图像存储结构体指针
*/
void CameraImgGetThread(cv::VideoCapture& Camera,Img_Store *Img_Store_p);


/*
    启动摄像头独立采集线程
*/
void CameraCaptureThreadStart(cv::VideoCapture& Camera,Img_Store *Img_Store_p,std::thread& captureThread);


/*
    停止摄像头独立采集线程
*/
void CameraCaptureThreadStop(Img_Store *Img_Store_p,std::thread& captureThread);


/*
	获取图像
    @参数说明
    Img_Store_p 图像存储结构体指针
*/
void CameraImgGet(Img_Store *Img_Store_p);


class ImgProcess
{
    public:
        string TextLoopKind[5] = {"camera_catch","judge","common","across","circle"};
        string TextCircleTrackStep[8] = {"IN_PREPARE","IN_PREPARE_2","IN","IN_CIRCLE","OUT_PREPARE","OUT_STRIGHT","OUT","INIT_CIRCLE"};
        string TextAcrossTrackStep[5] = {"ACROSS_PREPARE","ACROSS","ACROSS_OUT","ACROSS_OUT_2","INIT_ACROSS"};
        string TextGyroscope[2] = {"FALSE","TRUE"};
        string TextModelTrackKind[5] = {"BRIDGE_ZONE","CROSSWALK_ZONE","DANGER_ZONE","RESCUE_ZONE","CHASE_ZONE"};
        string TextControl[2] = {"FALSE","TRUE"};

        /*
            图像合成显示并保存
            @参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
            Function_EN_p 函数使能指针
        */
        void ImgShow(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p);


    // private:


        void ImgLabel(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p);

        /*
            图像边线拐点绘制
            @参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关指针
        */
        void ImgInflectionPointDraw(Img_Store *Img_Store_p,Data_Path *Data_Path_p);


        /*
            赛道类型、圆环步骤显示
            @参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
            Function_EN_p 函数使能指针
        */
        void ImgText(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p);


        /*
            将多个图像合成在同一窗口
            @参数说明
            Img_Store_p 图像存储指针
            Function_EN_p 函数使能指针
        */
        void ImgSynthesis(Img_Store *Img_Store_p,Function_EN *Function_EN_p);


        /*
            存储图像
            @参数说明
            Img_Store_p 图像存储指针
        */
        void ImgSave(Img_Store *Img_Store_p);


        /*
            图像参考线绘制
            1.边线断点起始线
            2.边线断点结束线
            3.中心竖线
            @参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
        */
        void ImgReferenceLine(Img_Store *Img_Store_p,Data_Path *Data_Path_p);
};

#endif
