#include "common/main.hpp"
#include "vision/Image_Process.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;



void FrameTaskAfterRead(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{   
    if (Img_Store_p->Img_Color.empty()) {cerr << "Error: Img_Color is empty!" << endl;return;}
    
	Img_Store_p->Img_Track = Img_Store_p->Img_Color.clone();
	cvtColor(Img_Store_p->Img_Track, Img_Store_p->Img_Gray, cv::COLOR_BGR2GRAY);

	Mat gray_cropped = Img_Store_p->Img_Gray(Rect(0, 30, 160, 60));
	Mat gray_80x60;
	resize(gray_cropped, gray_80x60, Size(80, 60), 0, 0, INTER_AREA);
	ImageProcess(gray_80x60);

    static int ring_frame_count = 0;
    JSON_TrackConfigData cfg = Data_Path_p->JSON_TrackConfigData_v[0];
    
    if (ImageFlag.image_element_rings_flag != 0) {
        ring_frame_count++;
        if (ring_frame_count > cfg.CircleMaxFrames) {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
            ring_frame_count = 0;
        }
    } else {
        ring_frame_count = 0;
    }
}

void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p, ImgProcess *imgProcess_p, Judge *judge_p)
{
    FrameTaskAfterRead(Img_Store_p, Data_Path_p, Function_EN_p, imgProcess_p, judge_p);
}

void ApplyDifferentialControl(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p, Judge *judge_p)
{
    (void)Img_Store_p;
    (void)Data_Path_p;
    (void)Function_EN_p;
    (void)judge_p;
}
