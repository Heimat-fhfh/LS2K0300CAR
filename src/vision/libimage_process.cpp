#include "common/common_system.h"
#include "common/common_program.h"
#include "vision/AAAdefine.h"
#include "vision/image_my_zf.h"
using namespace std;
using namespace cv;

void ImgProcess::imgPreProc(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	if (Img_Store_p->Img_Color.empty()) {
        cerr << "Error: Img_Color is empty!" << endl;
        return;
    }
	
	Img_Store_p->Img_Track = Img_Store_p->Img_Color.clone();
	cvtColor(Img_Store_p->Img_Track, Img_Store_p->Img_Gray, cv::COLOR_BGR2GRAY);

	Mat gray_cropped = Img_Store_p->Img_Gray(Rect(0, 30, 160, 60));
	Mat gray_80x60;
	resize(gray_cropped, gray_80x60, Size(80, 60), 0, 0, INTER_AREA);
	ImageProcess_my_zf(gray_80x60);

	Img_Store_p->Img_OTSU = Mat(60, 80, CV_8UC1);
	for (int i = 0; i < 60; i++) {
		for (int j = 0; j < 80; j++) {
			Img_Store_p->Img_OTSU.at<uint8_t>(i, j) = Pixle[i][j] ? 255 : 0;
		}
	}
}

void ImgProcess::ImgPrepare(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	(void)JSON_TrackConfigData;
	int border_thickness = 3;
	rectangle(Img_Store_p->Img_OTSU, 
			  Point(border_thickness, border_thickness),
			  Point(CAMERA_W - border_thickness - 1, CAMERA_H - border_thickness - 1),
			  Scalar(255),
			  border_thickness);
}


void ImgProcess::ImgSobel(Mat& Img)
{
	Mat ImgX;
	Mat ImgY;

    Sobel(Img,ImgX,CV_16S,1,0,5);
    convertScaleAbs(ImgX,ImgX);
	Sobel(Img,ImgY,CV_16S,0,1,5);
    convertScaleAbs(ImgY,ImgY);
    addWeighted(ImgX,0.5,ImgY,0.5,0,Img);
}


void ImgProcess::ImgScharr(Mat& Img)
{
	Mat ImgX;
	Mat ImgY;

    Scharr(Img,ImgX,CV_16S,1,0,3);
    convertScaleAbs(ImgX,ImgX);
	Scharr(Img,ImgY,CV_16S,1,0,3);
    convertScaleAbs(ImgY,ImgY);
    addWeighted(ImgX,0.5,ImgY,0.5,0,Img);
}


void ImgProcess::ImgSharpen(Mat &Img,int blursize = 5)
{
	Mat Img_Gauss;
	GaussianBlur(Img,Img_Gauss,Size(blursize,blursize),3,3);
	addWeighted(Img,2,Img_Gauss,-1,0,Img);
}


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
