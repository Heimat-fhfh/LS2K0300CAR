#include "common_system.h"
#include "common_program.h"
#include "AAAdefine.h"
#include "vision_transform.h"
#include <condition_variable>
#include <display_show.h>
#include <iomanip>
using namespace std;
using namespace cv;

namespace
{
VisionTransformPipeline g_vision_transform_pipeline;
std::once_flag g_vision_transform_init_once;
bool g_vision_transform_ready = false;

bool EnsureVisionTransformReady()
{
	std::call_once(g_vision_transform_init_once, []() {
		std::string errorMessage;
		// 使用相对于项目根目录的绝对路径
		g_vision_transform_ready = g_vision_transform_pipeline.loadConfig("./config/vision_transform.json", &errorMessage);
		if (!g_vision_transform_ready)
		{
			std::cerr << "[VisionTransform] 配置加载失败，后续按原图处理: " << errorMessage << std::endl;
		}
	});
	return g_vision_transform_ready;
}

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



mutex CameraCapture_Mutex;  // 摄像头采集资源互斥锁
condition_variable CameraCapture_CV; // 双缓冲图像就绪通知

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

/*
	CameraInit说明
	摄像头初始化
*/
void CameraInit(VideoCapture& Camera,CameraKind Camera_EN,int Width,int Height,int FPS)
{	
	int ret;
	// 相机类型设置
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
		}    // 演示视频
        case VIDEO_0:{ Camera.open("/dev/video0",CAP_V4L2); break; }  // 摄像头video0
    }

	if (Camera_EN == DEMO_VIDEO){
		
	}
	else
	{
		Camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', '2'));
		Camera.set(CAP_PROP_FRAME_WIDTH, Width);      // 帧宽
		Camera.set(CAP_PROP_FRAME_HEIGHT, Height);     // 帧高
		Camera.set(CAP_PROP_FPS, FPS);              // 帧率
		Camera.set(CAP_PROP_AUTO_EXPOSURE, 3);
		// Camera.set(CAP_PROP_EXPOSURE, 3);	// 曝光度
		// Camera.set(CAP_PROP_BRIGHTNESS, 0.8);    // 亮度，范围通常0~1
		// Camera.set(CAP_PROP_GAIN, 1);          // 增益，提高亮度但会增加噪点
		// Camera.set(CAP_PROP_CONTRAST, 0.7);      // 对比度，让画面更通透
		double actualWidth = Camera.get(CAP_PROP_FRAME_WIDTH); 
		double actualHeight = Camera.get(CAP_PROP_FRAME_HEIGHT); 
		double actualFps = Camera.get(CAP_PROP_FPS); 
		printf("摄像头配置信息：\n"); 
		printf("请求帧率：%d FPS\n", FPS);
		printf("分辨率：%.0fx%.0f\n", actualWidth, actualHeight); 
		printf("帧率：%.0f FPS\n", actualFps);
		printf("格式：%s\n", FourccToText(Camera.get(CAP_PROP_FOURCC)).c_str());
		printf("自动曝光：%.0f\n", Camera.get(CAP_PROP_AUTO_EXPOSURE));

		if (!Camera.isOpened())
		{
			cout << "<---------------------相机初始化失败--------------------->" << endl;
			abort();
		}
		else
		{
			cout << "<---------------------相机初始化成功--------------------->" << endl;
		}

	}
}


/*
	摄像头获取图像线程
*/
void CameraImgGetThread(VideoCapture& Camera,Img_Store *Img_Store_p)
{
	Mat Img;
	double cameraFps = Camera.get(CAP_PROP_FPS);
	cout << "摄像头获取图像线程 Camera FPS: " << cameraFps
	     << " FOURCC: " << FourccToText(Camera.get(CAP_PROP_FOURCC)) << endl;
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
			Img_Store_p->Img_BufferReady[staleIndex] = false; // 始终丢弃旧帧，仅保留最新帧
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
			const double readFps = threadFrameCount * 1000.0 / static_cast<double>(elapsedMs);
			cout << "[CameraThread] read_fps=" << fixed << setprecision(2) << readFps
			     << " camera_fps=" << cameraFps
			     << " failed=" << threadFailedCount
			     << " empty=" << threadEmptyCount << endl;
			threadFrameCount = 0;
			threadFailedCount = 0;
			threadEmptyCount = 0;
			statsStart = now;
		}
    }
}


/*
	获取图像
*/
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

	//displayMatOnIPS200(Img_Store_p->Img_OTSU); // 在IPS200显示二值化图像，便于调试
	
}

/**
 * @brief 图像预处理
 * 图像预处理函数，包含以下步骤：
 * 1. 将彩色图像转换为灰度图像。
 * 2. 对灰度图像进行高斯模糊以减少噪声。
 * 3. 使用Otsu's方法对模糊后的图像进行二值化处理。
 * 4. 在二值化图像的边界绘制白色边框以防止八邻域寻线出错。
 * @param Img_Store_p 图像存储指针，包含原始图像和处理后的图像。
 * @param Data_Path_p 路径数据指针。
 * @param Function_EN_p 功能使能状态指针。
 */
void ImgProcess::imgPreProc(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	if (Img_Store_p->Img_Color.empty()) {
        cerr << "Error: Img_Color is empty!" << endl;
        return;
    }
	
	Img_Store_p->Img_Track = Img_Store_p->Img_Color.clone();

	vector<Mat> bgrChannels;
	Mat rgSum;

	// split(Img_Store_p->Img_Color, bgrChannels);
	// add(bgrChannels[2], bgrChannels[1], rgSum);          // R + G
	// subtract(rgSum, bgrChannels[0], Img_Store_p->Img_Gray); // R + G - B（8U 饱和裁剪）

	// 上下相差 10% 左右

	cvtColor(Img_Store_p->Img_Track, Img_Store_p->Img_Gray, cv::COLOR_BGR2GRAY);

	/**
	 * 手动算法: 2.343 ms
	 * cvtColor: 1.359 ms
	 * cvtColor 快 1.724 倍
	 **/

	Mat blurred;
	// remap(Img_Store_p->Img_Gray, blurred, map1, map2, cv::INTER_CUBIC);
	// Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
	// morphologyEx(Img_Store_p->Img_Gray, blurred, MORPH_CLOSE, kernel);
	// bilateralFilter(Img_Store_p->Img_Gray, blurred, 7, 100, 100);
    GaussianBlur(Img_Store_p->Img_Gray, blurred, Size(3, 3), 0);
    threshold(blurred, Img_Store_p->Img_OTSU, 0, 255, THRESH_BINARY | THRESH_OTSU);
	// 上下相差占用 6% 左右
    // threshold(Img_Store_p->Img_Gray, Img_Store_p->Img_OTSU, 0, 255, THRESH_BINARY | THRESH_OTSU);

	// 将Img_Track改为二值化图像的彩色版本，用于绘制线条
	// cv::Mat temp;
	// cv::cvtColor(Img_Store_p->Img_OTSU, temp, cv::COLOR_GRAY2BGR);
	// Img_Store_p->Img_Track = temp.clone();

	const int imgWidth = Img_Store_p->Img_OTSU.cols;
	const int imgHeight = Img_Store_p->Img_OTSU.rows;
	line(Img_Store_p->Img_OTSU,Point(0,0),Point(imgWidth-1,0),Scalar(0),3);
	line(Img_Store_p->Img_OTSU,Point(imgWidth-1,0),Point(imgWidth-1,imgHeight-1),Scalar(0),3);
	line(Img_Store_p->Img_OTSU,Point(imgWidth-1,imgHeight-1),Point(0,imgHeight-1),Scalar(0),3);
	line(Img_Store_p->Img_OTSU,Point(0,imgHeight-1),Point(0,0),Scalar(0),3);

	
	
}	

/*
	ImgPrepare说明
	图像预处理
	先二值化使赛道边缘更为清晰
	然后用sobel算子检测边缘
	最后再次二值化
*/
void ImgProcess::ImgPrepare(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	
	// 加白框防止八邻域寻线出错
	int border_thickness = 3;
	rectangle(Img_Store_p->Img_OTSU, 
			  Point(border_thickness, border_thickness),                     // 左上角
			  Point(CAMERA_W - border_thickness - 1, CAMERA_H - border_thickness - 1), // 右下角
			  Scalar(255),  // 白色
			  border_thickness);
}


/*
	ImgSobel说明
	Sobel算子检测边缘
	将传入的图像用Sobel算子处理
*/
void ImgProcess::ImgSobel(Mat& Img)
{
	Mat ImgX;
	Mat ImgY;

	//对X方向微分
    Sobel(Img,ImgX,CV_16S,1,0,5); 	//x方向差分阶数 y方向差分阶数 核大小  
    convertScaleAbs(ImgX,ImgX);     //可将任意类型的数据转化为CV_8UC1
	//对Y方向微分
	Sobel(Img,ImgY,CV_16S,0,1,5); 	//x方向差分阶数 y方向差分阶数 核大小  
    convertScaleAbs(ImgY,ImgY);     //将任意类型的图像转化为CV_8UC1
    addWeighted(ImgX,0.5,ImgY,0.5,0,Img);	//图像的线性混合
}


/*
	ImgScharr说明
	Scharr算子检测边缘
	将传入的逆透视边缘二值化图像用Scharr算子处理
*/
void ImgProcess::ImgScharr(Mat& Img)
{
	Mat ImgX;
	Mat ImgY;

	//对X方向微分
    Scharr(Img,ImgX,CV_16S,1,0,3); 	//x方向差分阶数 y方向差分阶数 核大小  
    convertScaleAbs(ImgX,ImgX);     //可将任意类型的数据转化为CV_8UC1
	//对Y方向微分
	Scharr(Img,ImgY,CV_16S,1,0,3); 	//x方向差分阶数 y方向差分阶数 核大小  
    convertScaleAbs(ImgY,ImgY);     //将任意类型的图像转化为CV_8UC1
    addWeighted(ImgX,0.5,ImgY,0.5,0,Img);	//图像的线性混合
}


/*
	ImgSharpen说明
	通过原图像和高斯滤波图像进行融合进行图像锐化
*/
void ImgProcess::ImgSharpen(Mat &Img,int blursize = 5)
{
	Mat Img_Gauss;
	GaussianBlur(Img,Img_Gauss,Size(blursize,blursize),3,3);
	addWeighted(Img,2,Img_Gauss,-1,0,Img);
}


/*
	ImgUnpivot说明
	逆透视
*/
void ImgProcess::ImgUnpivot(Mat Img,Mat& Img_Unpivot)
{
    Point2f SrcPoints[] = { 
		Point2f(0,240),
		Point2f(320,240),
		Point2f(120,25),
		Point2f(200,25) };
 
	Point2f DstPoints[] = {
		Point2f(80,240),
		Point2f(240,240),
		Point2f(80,0),
		Point2f(240,0) };
 
	Mat UnpivotMat = getPerspectiveTransform(SrcPoints , DstPoints);

    warpPerspective(Img , Img_Unpivot , UnpivotMat , Size(320,240) , INTER_LINEAR);
}


/*
	ImgSynthesis说明
	将多个图像合成在同一窗口
*/
void ImgProcess::ImgSynthesis(Img_Store *Img_Store_p,Function_EN *Function_EN_p)
{
	JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];

	int ImgAllWidth = (Img_Store_p -> Img_Color).cols;	//宽度
	int ImgAllHeight = (Img_Store_p -> Img_Color).rows; //高度
	Mat ImgAll = Mat(ImgAllHeight+210,ImgAllWidth*3+18,CV_8UC3,Scalar(0,0,0));	//显示全部画面的画布

	//统一图像的类型为8UC3
	cvtColor((Img_Store_p -> Img_OTSU) , (Img_Store_p -> Img_OTSU) ,COLOR_GRAY2RGB);
	// cvtColor((Img_Store_p -> Img_OTSU_Unpivot) , (Img_Store_p -> Img_OTSU_Unpivot) ,COLOR_GRAY2RGB);
	
	//Rect roi(ImgAllWidth*i,0,ImgAllWidth,ImgAllHeight);  
	//定义一个矩形roi
	//将img_tmp复制到img中roi指定的矩形位置
	//此处简化
    
	(Img_Store_p -> Img_Color).copyTo(ImgAll(Rect(0,0,ImgAllWidth,ImgAllHeight))); 
	// (Img_Store_p -> Img_Track_Unpivot).copyTo(ImgAll(Rect(0,ImgAllHeight+6,Img_Store_p -> Img_Track_Unpivot.cols,Img_Store_p -> Img_Track_Unpivot.rows))); 
	(Img_Store_p -> Img_Track).copyTo(ImgAll(Rect(ImgAllWidth+6,0,ImgAllWidth,ImgAllHeight)));  
	(Img_Store_p -> Img_OTSU).copyTo(ImgAll(Rect(ImgAllWidth*2+12,0,ImgAllWidth,ImgAllHeight))); 
	// (Img_Store_p -> Img_OTSU_Unpivot).copyTo(ImgAll(Rect(ImgAllWidth*2+12,ImgAllHeight+6,Img_Store_p -> Img_OTSU_Unpivot.cols,Img_Store_p -> Img_OTSU_Unpivot.rows))); 
	(Img_Store_p -> Img_Text).copyTo(ImgAll(Rect(ImgAllWidth+6,ImgAllHeight+6,ImgAllWidth,200))); 

    (Img_Store_p -> Img_All) = ImgAll;

	if(JSON_FunctionConfigData.VideoShow_EN == true)
	{
		imshow("CAMERA",(Img_Store_p -> Img_All));
		// imshow("逆透视",(Img_Store_p -> Img_OTSU_Unpivot));
	}
}


/*
	ImgSave说明
	以Mat形式传入待存储图像
*/
void ImgProcess::ImgSave(Img_Store *Img_Store_p)
{
	string ImgWritePath = "img/ImgAll/" + to_string(Img_Store_p -> ImgNum) + ".jpg";
	//建立一个字符串用于存储图片存储路径
	//使用字符串定义图片存储路径
	//必须要加后缀 否则编译会报错

	imwrite(ImgWritePath , (Img_Store_p -> Img_All));
	//存储图片流
}



/*
	ImgCompress说明
	图像压缩
	将图像压缩至320*240大小
*/
void ImgProcess::ImgCompress(Mat& Img,bool ImgCompress_EN)
{
	Mat ImgCompress;
	if(ImgCompress_EN == true)
	{
		Size size = Size(320,240);
		resize(Img,ImgCompress,size,0,0,INTER_AREA);
		//将图像压缩为320*240大小
		Img = ImgCompress;
	}
}


/*
	ImgText说明
	赛道类型、圆环步骤、编码器积分标志位、模型区域类型显示
*/
void ImgProcess::ImgText(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	int ImgWidth = (Img_Store_p -> Img_Color).cols;	// 宽度
	(Img_Store_p -> Img_Text) = Mat(200,ImgWidth,CV_8UC3,Scalar(0,0,0));	// 显示文字画布

	putText((Img_Store_p -> Img_Text),TextTrackKind[int(Data_Path_p -> Temp_Track_Kind)],Point(5,25),FONT_HERSHEY_COMPLEX,1,(255),2);
	putText((Img_Store_p -> Img_Text),TextTrackKind[int(Data_Path_p -> Track_Kind)],Point(5,65),FONT_HERSHEY_COMPLEX,1,(255),2);
	putText((Img_Store_p -> Img_Text),TextLoopKind[int(Data_Path_p -> Loop_Kind)],Point(5,105),FONT_HERSHEY_COMPLEX,1,(255),2);	
	putText((Img_Store_p -> Img_Text),TextCircleTrackStep[int(Data_Path_p -> Circle_Track_Step)],Point(5,145),FONT_HERSHEY_COMPLEX,1,(255),2);
}


/*
	ImgShow说明
	图像合成显示并保存
*/
void ImgProcess::ImgShow(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];

	ImgProcess::ImgLabel(Img_Store_p,Data_Path_p,Function_EN_p);
	ImgProcess::ImgInflectionPointDraw(Img_Store_p,Data_Path_p); 
	// ImgProcess::ImgBendPointDraw(Img_Store_p,Data_Path_p); 
	ImgProcess::ImgTransitionScanDraw(Img_Store_p, Data_Path_p);
	// ImgProcess::ImgForwardLine(Img_Store_p,Data_Path_p);
	ImgProcess::ImgReferenceLine(Img_Store_p,Data_Path_p);
	ImgProcess::ImgText(Img_Store_p,Data_Path_p,Function_EN_p);
	ImgProcess::ImgSynthesis(Img_Store_p,Function_EN_p);
	if(JSON_FunctionConfigData.ImageSave_EN == true)
	{
		ImgProcess::ImgSave(Img_Store_p);
	}
}
/*
    ImgInflectionPointDraw说明
	图像边线拐点绘制
	限制绘制数目
*/
void ImgProcess::ImgInflectionPointDraw(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	// 变量设置
	int i = 0;
	int j = 0;
	// 左边线拐点绘制
	if((Data_Path_p -> InflectionPointNum[0]) >= 1)
	{
		for(i = 0;i <= (Data_Path_p -> InflectionPointNum[0])-1;i++)
		{
			if(i == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> InflectionPointNum[0]),
				Point((Data_Path_p -> InflectionPointCoordinate[i][0]),(Data_Path_p -> InflectionPointCoordinate[i][1])),
				FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);

				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[i][0]),(Data_Path_p -> InflectionPointCoordinate[i][1])),
				6,Scalar(128,0,128),2);	// 左边线拐点画点：浅紫色
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[i][0]),(Data_Path_p -> InflectionPointCoordinate[i][1])),
				6,Scalar(255,0,255),1);	// 左边线拐点画点：紫色
			}
		}
	}
	// 右边线拐点绘制
	if((Data_Path_p -> InflectionPointNum[1]) >= 1)
	{
		for(j = 0;j <= (Data_Path_p -> InflectionPointNum[1])-1;j++)
		{
			if(j == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> InflectionPointNum[1]),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),
				(Data_Path_p -> InflectionPointCoordinate[j][3])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);

				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),(Data_Path_p -> InflectionPointCoordinate[j][3])),
				6,Scalar(128,0,128),2);	// 右边线拐点画点：浅紫色
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),(Data_Path_p -> InflectionPointCoordinate[j][3])),
				6,Scalar(255,0,255),1);	// 右边线拐点画点：紫色
			}
		}
	}
}


/*
    ImgBendPointDraw说明
	图像边线弯点绘制
	限制绘制数目
*/
void ImgProcess::ImgBendPointDraw(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	// 变量设置
	int i = 0;
	int j = 0;
	// 左边线弯点点绘制
	if((Data_Path_p -> BendPointNum[0]) >= 1)
	{
		for(i = 0;i <= (Data_Path_p -> BendPointNum[0])-1;i++)
		{
			if(i == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> BendPointNum[0]),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),6,Scalar(0,128,128),2);	// 左边线弯点画点：浅黄色
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),6,Scalar(0,255,255),1);	// 左边线弯点画点：黄色
			}
		}
	}
	// 右边线弯点点绘制
	if((Data_Path_p -> BendPointNum[1]) >= 1)
	{
		for(j = 0;j <= (Data_Path_p -> BendPointNum[1])-1;j++)
		{
			if(j == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> BendPointNum[1]),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),6,Scalar(0,128,128),2);	// 右边线弯点画点：浅黄色

			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),6,Scalar(0,255,255),1);	// 右边线弯点画点：黄色
			
			}
		}
	}
}

/*
	ImgForwardLine说明
	前瞻点画线
*/
void ImgProcess::ImgForwardLine(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	circle(Img_Store_p -> Img_Track,Point(Data_Path_p -> center_line[Data_Path_p->forword_line_h],Data_Path_p->forword_line_h),3,Scalar(255,0,0),2);
	// line((Img_Store_p -> Img_Track),Point(0,Data_Path_p->forword_line_h),Point(image_w-1,Data_Path_p->forword_line_h),Scalar(255,0,0),3);
    // line((Img_Store_p -> Img_Track),Point(image_w/2,300),Point((Data_Path_p -> center_line[(JSON_TrackConfigData.Forward)-(JSON_TrackConfigData.Path_Search_Start)][0]),(Data_Path_p -> TrackCoordinate[(JSON_TrackConfigData.Forward)-(JSON_TrackConfigData.Path_Search_Start)][1])),Scalar(255,0,0),3);
	// putText((Img_Store_p -> Img_Track),to_string(abs(image_w/2-(Data_Path_p -> center_line[(JSON_TrackConfigData.Forward)-(JSON_TrackConfigData.Path_Search_Start)][0]))),Point((Data_Path_p -> TrackCoordinate[(JSON_TrackConfigData.Forward)-(JSON_TrackConfigData.Path_Search_Start)][0]),(Data_Path_p -> TrackCoordinate[(JSON_TrackConfigData.Forward)-(JSON_TrackConfigData.Path_Search_Start)][1])),FONT_HERSHEY_COMPLEX,0.6,(255,255,255),1);
}


/*
	ImgReferenceLine说明
	图像参考线绘制
*/
void ImgProcess::ImgReferenceLine(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	
	line(Img_Store_p->Img_Track,Point(0,image_h-JSON_TrackConfigData.Path_Search_End),
	Point(Img_Store_p->Img_Track.cols-1,image_h - JSON_TrackConfigData.Path_Search_End),Scalar(0xEA832F),1);	// 上边界 #2F83EA

	line(Img_Store_p->Img_Track,Point(0,image_h-JSON_TrackConfigData.Path_Search_Start),
	Point(image_w-1,image_h-JSON_TrackConfigData.Path_Search_Start),Scalar(0,200,0),1);	// 下边界 
}

/*
    ImgTransitionScanDraw说明
	独立黑色区域绘画
*/
void ImgProcess::ImgTransitionScanDraw(Img_Store *Img_Store_p, Data_Path *Data_Path_p)
{
    if (Img_Store_p == nullptr || Data_Path_p == nullptr) return;
    if (Img_Store_p->Img_Track.empty()) return;
	if (!Data_Path_p->black_left_found && !Data_Path_p->black_right_found) return; // 仅在找到时绘制独立黑块，避免干扰正常边线显示


    JSON_TrackConfigData cfg = Data_Path_p->JSON_TrackConfigData_v[0];

	vector<vector<Point>> contours = Data_Path_p->TransitionContours;
	vector<Vec4i> hierarchy = Data_Path_p->TransitionHierarchy;

	int hole_count = 0;
	for (size_t i = 0; i < contours.size() && i < hierarchy.size(); ++i) {
		if (hierarchy[i][3] < 0) continue;
		double area = contourArea(contours[i]);
		if (area < cfg.TransitionMinArea) continue;

		drawContours(Img_Store_p->Img_Track, contours, static_cast<int>(i), Scalar(0, 0, 255), 2);

		Moments mu = moments(contours[i]);
		if (mu.m00 > 0.0) {
			int cx = static_cast<int>(mu.m10 / mu.m00);
			int cy = static_cast<int>(mu.m01 / mu.m00);
			circle(Img_Store_p->Img_Track, Point(cx, cy), 2, Scalar(0, 255, 255), -1);
		}
		++hole_count;
	}

	circle((Img_Store_p -> Img_Track),Data_Path_p -> leftmost_point, 6,Scalar(255,0,255),1);	// 独立黑块最左侧位置
	circle((Img_Store_p -> Img_Track),Data_Path_p -> rightmost_point, 6,Scalar(255,0,255),1);	// 独立黑块最右侧位置

}

void ImgProcess::ImgLabel(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_l[0][0],Data_Path_p->points_l[0][1]), 6, Scalar(0, 0, 255), 1);
	circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_r[0][0],Data_Path_p->points_r[0][1]), 6, Scalar(0, 0, 255), 1);

	// circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_l[1][0],Data_Path_p->points_l[1][1]), 6, Scalar(0, 255, 0), 1);
	// circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_r[1][0],Data_Path_p->points_r[1][1]), 6, Scalar(0, 255, 0), 1);

	for (int i = 0; i < Data_Path_p->NumSearch[0]; i++)
	{
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_l[i][0],Data_Path_p->points_l[i][1]), 2, Scalar(0, 125, 0), FILLED); // 
	}
	for (int i = 0; i < Data_Path_p->NumSearch[1]; i++)
	{
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->points_r[i][0],Data_Path_p->points_r[i][1]), 2, Scalar(125, 0, 0), FILLED);
	}

	for (int i = Data_Path_p->search_print_h_max; i < image_h-JSON_TrackConfigData.Path_Search_Start; i++)
	{
		// Data_Path_p->center_line[i] = (Data_Path_p->l_border[i] + Data_Path_p->r_border[i]) >> 1;//求中线

		circle(Img_Store_p->Img_Track, Point(Data_Path_p->l_border[i],i), 1, Scalar(0, 0, 255), FILLED);//显示起点 显示左边线
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->r_border[i],i), 1, Scalar(0, 255, 0), FILLED);//显示起点 显示右边线
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->center_line[i],i), 1, Scalar(0, 0, 0), FILLED);//显示起点 显示中线	
	}
}
