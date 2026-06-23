#include "common/common_system.h"
#include "common/common_program.h"
#include "vision/AAAdefine.h"
#include "vision/image_my_zf.h"
#include "devices/display_show.h"
#include <iomanip>
using namespace std;
using namespace cv;

void ImgProcess::ImgSynthesis(Img_Store *Img_Store_p,Function_EN *Function_EN_p)
{
	JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];

	int ImgAllWidth = (Img_Store_p -> Img_Color).cols;
	int ImgAllHeight = (Img_Store_p -> Img_Color).rows;
	Mat ImgAll = Mat(ImgAllHeight+210,ImgAllWidth*3+18,CV_8UC3,Scalar(0,0,0));

	cvtColor((Img_Store_p -> Img_OTSU) , (Img_Store_p -> Img_OTSU) ,COLOR_GRAY2RGB);
    
	(Img_Store_p -> Img_Color).copyTo(ImgAll(Rect(0,0,ImgAllWidth,ImgAllHeight))); 
	(Img_Store_p -> Img_Track).copyTo(ImgAll(Rect(ImgAllWidth+6,0,ImgAllWidth,ImgAllHeight)));  
	(Img_Store_p -> Img_OTSU).copyTo(ImgAll(Rect(ImgAllWidth*2+12,0,ImgAllWidth,ImgAllHeight))); 
	(Img_Store_p -> Img_Text).copyTo(ImgAll(Rect(ImgAllWidth+6,ImgAllHeight+6,ImgAllWidth,200))); 

    (Img_Store_p -> Img_All) = ImgAll;

	if(JSON_FunctionConfigData.VideoShow_EN == true)
	{
		imshow("CAMERA",(Img_Store_p -> Img_All));
	}
}


void ImgProcess::ImgSave(Img_Store *Img_Store_p)
{
	string ImgWritePath = "img/ImgAll/" + to_string(Img_Store_p -> ImgNum) + ".jpg";

	imwrite(ImgWritePath , (Img_Store_p -> Img_All));
}


void ImgProcess::ImgCompress(Mat& Img,bool ImgCompress_EN)
{
	Mat ImgCompress;
	if(ImgCompress_EN == true)
	{
		Size size = Size(320,240);
		resize(Img,ImgCompress,size,0,0,INTER_AREA);
		Img = ImgCompress;
	}
}


void ImgProcess::ImgText(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	int ImgWidth = (Img_Store_p -> Img_Color).cols;
	(Img_Store_p -> Img_Text) = Mat(200,ImgWidth,CV_8UC3,Scalar(0,0,0));

	putText((Img_Store_p -> Img_Text),TextTrackKind[int(Data_Path_p -> Temp_Track_Kind)],Point(5,25),FONT_HERSHEY_COMPLEX,1,(255),2);
	putText((Img_Store_p -> Img_Text),TextTrackKind[int(Data_Path_p -> Track_Kind)],Point(5,65),FONT_HERSHEY_COMPLEX,1,(255),2);
	putText((Img_Store_p -> Img_Text),TextLoopKind[int(Data_Path_p -> Loop_Kind)],Point(5,105),FONT_HERSHEY_COMPLEX,1,(255),2);	
	putText((Img_Store_p -> Img_Text),TextCircleTrackStep[int(Data_Path_p -> Circle_Track_Step)],Point(5,145),FONT_HERSHEY_COMPLEX,1,(255),2);
	if (Data_Path_p->Track_Kind == L_ACROSS_TRACK || Data_Path_p->Track_Kind == R_ACROSS_TRACK) {
		putText((Img_Store_p -> Img_Text),TextAcrossTrackStep[int(Data_Path_p -> Across_Track_Step)],Point(5,185),FONT_HERSHEY_COMPLEX,1,(255),2);
	}
}


void ImgProcess::ImgShow(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];

	ImgProcess::ImgLabel(Img_Store_p,Data_Path_p,Function_EN_p);
	ImgProcess::ImgInflectionPointDraw(Img_Store_p,Data_Path_p); 
	ImgProcess::ImgTransitionScanDraw(Img_Store_p, Data_Path_p);
	ImgProcess::ImgReferenceLine(Img_Store_p,Data_Path_p);
	ImgProcess::ImgText(Img_Store_p,Data_Path_p,Function_EN_p);
	ImgProcess::ImgSynthesis(Img_Store_p,Function_EN_p);
	if(JSON_FunctionConfigData.ImageSave_EN == true)
	{
		ImgProcess::ImgSave(Img_Store_p);
	}
}

void ImgProcess::ImgInflectionPointDraw(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	int i = 0;
	int j = 0;
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
				6,Scalar(128,0,128),2);
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[i][0]),(Data_Path_p -> InflectionPointCoordinate[i][1])),
				6,Scalar(255,0,255),1);
			}
		}
	}
	if((Data_Path_p -> InflectionPointNum[1]) >= 1)
	{
		for(j = 0;j <= (Data_Path_p -> InflectionPointNum[1])-1;j++)
		{
			if(j == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> InflectionPointNum[1]),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),
				(Data_Path_p -> InflectionPointCoordinate[j][3])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);

				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),(Data_Path_p -> InflectionPointCoordinate[j][3])),
				6,Scalar(128,0,128),2);
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> InflectionPointCoordinate[j][2]),(Data_Path_p -> InflectionPointCoordinate[j][3])),
				6,Scalar(255,0,255),1);
			}
		}
	}
}


void ImgProcess::ImgBendPointDraw(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	int i = 0;
	int j = 0;
	if((Data_Path_p -> BendPointNum[0]) >= 1)
	{
		for(i = 0;i <= (Data_Path_p -> BendPointNum[0])-1;i++)
		{
			if(i == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> BendPointNum[0]),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),6,Scalar(0,128,128),2);
			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[i][0]),(Data_Path_p -> BendPointCoordinate[i][1])),6,Scalar(0,255,255),1);
			}
		}
	}
	if((Data_Path_p -> BendPointNum[1]) >= 1)
	{
		for(j = 0;j <= (Data_Path_p -> BendPointNum[1])-1;j++)
		{
			if(j == 0)
			{
				putText((Img_Store_p -> Img_Track),to_string(Data_Path_p -> BendPointNum[1]),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),FONT_HERSHEY_COMPLEX,0.6,Scalar(0,0,255),1);
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),6,Scalar(0,128,128),2);

			}
			else
			{
				circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> BendPointCoordinate[j][2]),(Data_Path_p -> BendPointCoordinate[j][3])),6,Scalar(0,255,255),1);
			
			}
		}
	}
}

void ImgProcess::ImgForwardLine(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
	(void)Data_Path_p;
	circle(Img_Store_p -> Img_Track,Point(Data_Path_p -> center_line[Data_Path_p->forword_line_h],Data_Path_p->forword_line_h),3,Scalar(255,0,0),2);
}


void ImgProcess::ImgReferenceLine(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	
	line(Img_Store_p->Img_Track,Point(0,image_h-JSON_TrackConfigData.Path_Search_End),
	Point(Img_Store_p->Img_Track.cols-1,image_h - JSON_TrackConfigData.Path_Search_End),Scalar(0xEA832F),1);

	line(Img_Store_p->Img_Track,Point(0,image_h-JSON_TrackConfigData.Path_Search_Start),
	Point(image_w-1,image_h-JSON_TrackConfigData.Path_Search_Start),Scalar(0,200,0),1);
}

void ImgProcess::ImgTransitionScanDraw(Img_Store *Img_Store_p, Data_Path *Data_Path_p)
{
    if (Img_Store_p == nullptr || Data_Path_p == nullptr) return;
    if (Img_Store_p->Img_Track.empty()) return;
	if (!Data_Path_p->black_left_found && !Data_Path_p->black_right_found) return;

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

	circle((Img_Store_p -> Img_Track),Data_Path_p -> leftmost_point, 6,Scalar(255,0,255),1);
	circle((Img_Store_p -> Img_Track),Data_Path_p -> rightmost_point, 6,Scalar(255,0,255),1);

}

void ImgProcess::ImgLabel(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
	JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

	for (int i = Data_Path_p->search_print_h_max; i < image_h-JSON_TrackConfigData.Path_Search_Start; i++)
	{
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->l_border[i],i), 1, Scalar(0, 0, 255), FILLED);
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->r_border[i],i), 1, Scalar(0, 255, 0), FILLED);
		circle(Img_Store_p->Img_Track, Point(Data_Path_p->center_line[i],i), 1, Scalar(0, 0, 0), FILLED);
	}

	{
		char buf[64];
		const char* road_name = "Normol";
		switch (ImageStatus.Road_type) {
			case Straight:     road_name = "Straight";   break;
			case LeftCirque:   road_name = "L-Cirque";   break;
			case RightCirque:  road_name = "R-Cirque";   break;
			case Cross:        road_name = "Cross";      break;
			case Cross_ture:   road_name = "CrossTure";  break;
			case Ramp:         road_name = "Ramp";       break;
			case Barn_in:      road_name = "Barn_in";    break;
			case Barn_out:     road_name = "Barn_out";   break;
			default:           road_name = "Normol";     break;
		}
		snprintf(buf, sizeof(buf), "Road:%s OFF:%d Det:%d",
			road_name, ImageStatus.OFFLine, ImageStatus.Det_True);
		putText(Img_Store_p->Img_Track, buf, Point(5, 18),
			FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1);
	}
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "RingFlag:%d Ring:%d Size:%d LL:%d RL:%d WL:%d",
			ImageFlag.image_element_rings_flag, ImageFlag.image_element_rings,
			ImageFlag.ring_big_small,
			ImageStatus.Left_Line, ImageStatus.Right_Line, ImageStatus.WhiteLine);
		putText(Img_Store_p->Img_Track, buf, Point(5, 36),
			FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 255, 255), 1);
	}
	{
		char buf[48];
		snprintf(buf, sizeof(buf), "SErrPx:%d TBS:%.0f",
			Data_Path_p->SteerErrorPx, Data_Path_p->TargetBaseSpeedMps);
		putText(Img_Store_p->Img_Track, buf, Point(5, 54),
			FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 0), 1);
	}
	{
		char buf[32];
		snprintf(buf, sizeof(buf), "TK:%d", static_cast<int>(Data_Path_p->Track_Kind));
		putText(Img_Store_p->Img_Track, buf, Point(Img_Store_p->Img_Track.cols - 50, 18),
			FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1);
	}
}
