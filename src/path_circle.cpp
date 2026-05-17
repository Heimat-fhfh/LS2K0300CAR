#include "common_program.h"
#include "common_system.h"

using namespace std;
using namespace cv;


// 圆环准备入环步骤：补线
void CircleTrack_Step_IN_Prepare(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Track_Kind)
    {
    }
}


// 圆环准备入环步骤：补线
void CircleTrack_Step_IN_Prepare_Stright(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Previous_Circle_Kind)
    {
    }
}



// 圆环入环步骤：补线
void CircleTrack_Step_IN(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    switch(Data_Path_p -> Previous_Circle_Kind)
    {
    }
}


// 圆环出环步骤：补线
void CircleTrack_Step_OUT(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Previous_Circle_Kind)
    {
    }
}


// 圆环出环后直线补线
void Circle2CommonTrack(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Previous_Circle_Kind)
    {
    }    
}
