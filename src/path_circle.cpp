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
        case L_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Data_Path_p->rightmost_point, Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
        }
        case R_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2+JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Data_Path_p->leftmost_point, Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
         }
    }
}

void CircleTrack_Step_IN_Prepare_2(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Track_Kind)
    {
        case L_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(image_w/2-JSON_TrackConfigData.Track_width/2, Data_Path_p->InflectionPointCoordinate[0][1]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
        }
        case R_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2+JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(image_w/2+JSON_TrackConfigData.Track_width/2, Data_Path_p->InflectionPointCoordinate[0][3]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
         }
    }
}

// 圆环入环步骤：补线
void CircleTrack_Step_IN(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    switch(Data_Path_p -> Track_Kind)
    {
        case L_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2+JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(Data_Path_p->InflectionPointCoordinate[0][0], Data_Path_p->InflectionPointCoordinate[0][1]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
        }
        case R_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(Data_Path_p->InflectionPointCoordinate[0][2], Data_Path_p->InflectionPointCoordinate[0][3]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
         }
    }
}

void CircleTrack_Step_OUT_PREPARE(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    // bool left_border_valid = Data_Path_p->l_border[image_h-JSON_TrackConfigData.Path_Search_Start] == border_min;
    // bool right_border_valid = Data_Path_p->r_border[image_h-JSON_TrackConfigData.Path_Search_Start] == border_max;
    // Point left_border_top,right_border_top;

    // if (left_border_valid){
    //     for (int i = image_h-JSON_TrackConfigData.Path_Search_Start; i < Data_Path_p->search_print_h_max; i++)
    //     {
    //         if (Data_Path_p->l_border[i] != 3){
    //             left_border_top = Point(Data_Path_p->l_border[i], i);
    //         }
    //     }
    // }
    // if (right_border_valid){
    //     for (int i = image_h-JSON_TrackConfigData.Path_Search_Start; i < Data_Path_p->search_print_h_max; i++)
    //     {
    //         if (Data_Path_p->r_border[i] != 3){
    //             right_border_top = Point(Data_Path_p->r_border[i], i);
    //         }
    //     }
    // }


    switch(Data_Path_p -> Track_Kind)
    {
        case L_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2+JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(border_min,image_h-JSON_TrackConfigData.Path_Search_End), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
        }
        case R_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(border_max,image_h-JSON_TrackConfigData.Path_Search_End), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
         }
    }
}

// 圆环出环步骤：补线
void CircleTrack_Step_OUT(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    switch(Data_Path_p -> Track_Kind)
    {
        case L_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(Data_Path_p->InflectionPointCoordinate[0][0],Data_Path_p->InflectionPointCoordinate[0][1]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
        }
        case R_CIRCLE_TRACK:
        {
            line(Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start),
            Point(Data_Path_p->InflectionPointCoordinate[0][2],Data_Path_p->InflectionPointCoordinate[0][3]), Scalar(0), 2);
            imgSearch_l_r(Img_Store_p,Data_Path_p);
            break;
         }
    }
}

