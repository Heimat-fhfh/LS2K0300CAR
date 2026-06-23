#include "common/common_system.h"
#include "common/common_program.h"
#include "vision/AAAdefine.h"
#include "vision/Image_Process.h"
#include <condition_variable>
#include <iomanip>
using namespace std;
using namespace cv;

namespace
{
std::string FourccToText(double fourccValue)
{
	const int fourcc = static_cast<int>(fourccValue);
	std::string text(4, ' ');
	text[0] = static_cast<char>(fourcc & 0xFF);
	text[1] = static_cast<char>((fourcc >> 8) & 0xFF);
	text[2] = static_cast<char>((fourcc >> 16) & 0xFF);
	text[3] = static_cast<char>((fourcc >> 24) & 0xFF);
	return text;
}
} // namespace



mutex CameraCapture_Mutex;
condition_variable CameraCapture_CV;

void CameraCaptureThreadStart(VideoCapture& Camera,Img_Store *Img_Store_p,std::thread& captureThread)
{
	{
		lock_guard<mutex> lock(CameraCapture_Mutex);
		Img_Store_p->CameraThreadRunning = true;
	}
	if (captureThread.joinable())
	{
		captureThread.join();
	}
	captureThread = std::thread(CameraImgGetThread, std::ref(Camera), Img_Store_p);
}


void CameraCaptureThreadStop(Img_Store *Img_Store_p,std::thread& captureThread)
{
	{
		lock_guard<mutex> lock(CameraCapture_Mutex);
		Img_Store_p->CameraThreadRunning = false;
	}
	CameraCapture_CV.notify_all();
	if (captureThread.joinable())
	{
		captureThread.join();
	}
}

void CameraInit(VideoCapture& Camera,CameraKind Camera_EN,int Width,int Height,int FPS)
{	
	int ret;
    switch(Camera_EN)
    {
        case DEMO_VIDEO:{
			ret = Camera.open("img/test_4.mp4"); 
			printf("VIDEO OPEN RETURN: %d\n", ret);
			if (ret == 0)
			{
				printf("Open Video Fail!\n");
				exit(-1);
			}
			break; 
		}
        case VIDEO_0:{ Camera.open("/dev/video0",CAP_V4L2); break; }
    }

	if (Camera_EN == DEMO_VIDEO){
		
	}
	else
	{
		Camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', '2'));
		Camera.set(CAP_PROP_FRAME_WIDTH, Width);
		Camera.set(CAP_PROP_FRAME_HEIGHT, Height);
		Camera.set(CAP_PROP_FPS, FPS);
		Camera.set(CAP_PROP_AUTO_EXPOSURE, 3);
		double actualWidth = Camera.get(CAP_PROP_FRAME_WIDTH); 
		double actualHeight = Camera.get(CAP_PROP_FRAME_HEIGHT); 

		if (!Camera.isOpened())
		{
			cout << "摄像头 初始化失败" << endl;
			abort();
		}
		else
		{
			printf("摄像头 初始化成功 %.0fx%.0f YUYV @%dFPS\n", actualWidth, actualHeight, FPS);
		}

	}
}


void CameraImgGetThread(VideoCapture& Camera,Img_Store *Img_Store_p)
{
	Mat Img;
	double cameraFps = Camera.get(CAP_PROP_FPS);
	uint64_t threadFrameCount = 0;
	uint64_t threadFailedCount = 0;
	uint64_t threadEmptyCount = 0;
	auto statsStart = chrono::steady_clock::now();
	{
		lock_guard<mutex> lock(CameraCapture_Mutex);
		Img_Store_p->CameraThreadRunning = true;
	}

    while (1)
    {
		{
			lock_guard<mutex> lock(CameraCapture_Mutex);
			if (!Img_Store_p->CameraThreadRunning)
			{
				break;
			}
		}

        if (!Camera.read(Img)) {
            cerr << "Error: Camera read failed!" << endl;
			++threadFailedCount;
			this_thread::sleep_for(chrono::milliseconds(2));
            continue;
        }

        if (Img.empty()) {
            cerr << "Error: Captured image is empty!" << endl;
			++threadEmptyCount;
			this_thread::sleep_for(chrono::milliseconds(2));
            continue;
        }

		{
			lock_guard<mutex> lock(CameraCapture_Mutex);
			int writeIndex = Img_Store_p->Img_WriteIndex;
			int staleIndex = 1 - writeIndex;
			Img_Store_p->Img_CaptureBuffer[writeIndex] = std::move(Img);
			Img_Store_p->Img_BufferReady[writeIndex] = true;
			Img_Store_p->Img_BufferReady[staleIndex] = false;
			Img_Store_p->Img_ReadIndex = writeIndex;
			Img_Store_p->Img_WriteIndex = 1 - writeIndex;
			Img_Store_p->Img_FrameSeq++;
		}
		CameraCapture_CV.notify_one();

		++threadFrameCount;
		const auto now = chrono::steady_clock::now();
		const auto elapsedMs = chrono::duration_cast<chrono::milliseconds>(now - statsStart).count();
		if (elapsedMs >= 1000)
		{
			threadFrameCount = 0;
			threadFailedCount = 0;
			threadEmptyCount = 0;
			statsStart = now;
		}
    }
}


void CameraImgGet(Img_Store *Img_Store_p)
{
	unique_lock<mutex> lock(CameraCapture_Mutex);
	CameraCapture_CV.wait(lock, [Img_Store_p]() {
		return (Img_Store_p->Img_FrameSeq != Img_Store_p->Img_LastReadSeq) || (!Img_Store_p->CameraThreadRunning);
	});

	if ((Img_Store_p->Img_FrameSeq == Img_Store_p->Img_LastReadSeq) && (!Img_Store_p->CameraThreadRunning))
	{
		return;
	}

	int readIndex = Img_Store_p->Img_ReadIndex;
	Img_Store_p->Img_Color = std::move(Img_Store_p->Img_CaptureBuffer[readIndex]);
	Img_Store_p->Img_BufferReady[readIndex] = false;
	Img_Store_p->Img_LastReadSeq = Img_Store_p->Img_FrameSeq;
}
