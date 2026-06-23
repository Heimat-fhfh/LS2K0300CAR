#include "common/common_system.h"
#include "common/common_program.h"
#include "vision/myacross.h"

using namespace std;
using namespace cv;


void AcrossTrack_Step_ACROSS_PREPARE(Img_Store *Img_Store_p,Data_Path *Data_Path_p){
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

	switch (Data_Path_p->Track_Kind){
		case R_ACROSS_TRACK:
		{
			line(Img_Store_p->Img_OTSU,Point(image_w/2+JSON_TrackConfigData.Track_width/2,image_h-JSON_TrackConfigData.Path_Search_Start),
			Data_Path_p->leftmost_point,Scalar(0),2);
			for(int i = Data_Path_p->leftmost_point.x; i > Data_Path_p->leftmost_point.x-JSON_TrackConfigData.Track_width/2; i--){
				if(Img_Store_p->bin_image[Data_Path_p->leftmost_point.y][i] == 0){
					line(Img_Store_p->Img_OTSU,Point(image_w/2-JSON_TrackConfigData.Track_width/2,image_h-JSON_TrackConfigData.Path_Search_Start),
					Point(i,Data_Path_p->leftmost_point.y),Scalar(0),2);
					break;
				}
			}
			imgSearch_l_r(Img_Store_p,Data_Path_p);
			break;
		}
		case L_ACROSS_TRACK:
		{
			line(Img_Store_p->Img_OTSU,Point(image_w/2-JSON_TrackConfigData.Track_width/2,image_h-JSON_TrackConfigData.Path_Search_Start),
			Data_Path_p->rightmost_point,Scalar(0),2);
			for(int i = Data_Path_p->rightmost_point.x; i < Data_Path_p->rightmost_point.x+JSON_TrackConfigData.Track_width/2; i++){
				if(Img_Store_p->bin_image[Data_Path_p->rightmost_point.y][i] == 0){
					line(Img_Store_p->Img_OTSU,Point(image_w/2+JSON_TrackConfigData.Track_width/2,image_h-JSON_TrackConfigData.Path_Search_Start),
					Point(i,Data_Path_p->rightmost_point.y),Scalar(0),2);
					break;
				}
			}
			imgSearch_l_r(Img_Store_p,Data_Path_p);
			break;
		}
	}
}

void two_point_line(cv::Mat *img, Point point1, Point point2, int line_bottom = 0, int line_top = image_h-1)
{
    // 参数有效性检查
    if (img == nullptr || img->empty()) return;
    
    // 确保上下边界在图像范围内
    line_bottom = std::max(0, line_bottom);
    line_top = std::min(img->rows - 1, line_top);
    if (line_bottom > line_top) return;
    
    // 当两点重合时，无法确定直线，直接返回
    if (point1.x == point2.x && point1.y == point2.y) return;
    
    // 处理垂直线（斜率无穷大）
    if (point1.x == point2.x)
    {
        int x = point1.x;
        if (x >= 0 && x < img->cols)
        {
            Point start(x, line_bottom);
            Point end(x, line_top);
            cv::line(*img, start, end, Scalar(0), 2);

        }
        return;
    }
    
    // 一般情况：计算斜率和截距  y = k * x + b
    double k = static_cast<double>(point2.y - point1.y) / (point2.x - point1.x);
    double b = point1.y - k * point1.x;
    
    // 计算在限定Y范围对应的X范围
    int x_start = static_cast<int>(round((line_bottom - b) / k));
    int x_end   = static_cast<int>(round((line_top - b) / k));
    
    if (x_start > x_end) std::swap(x_start, x_end);
    
    // 限制X范围在图像宽度内
    x_start = std::max(0, x_start);
    x_end = std::min(img->cols - 1, x_end);
    
	Point start(x_start, static_cast<int>(round(k * x_start + b)));
    Point end(x_end, static_cast<int>(round(k * x_end + b)));

    cv::line(*img, start, end, Scalar(0), 2);

}

void Points_to_line_drew(cv::Mat *img, Point points[], int num, int line_bottom=0, int line_top=image_h) {
    if (num < 2 || img == nullptr || img->empty()) {
        return;
    }
    
    // 计算拟合直线
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    
    for (int i = 0; i < num; i++) {
        sum_x += points[i].x;
        sum_y += points[i].y;
        sum_xy += points[i].x * points[i].y;
        sum_x2 += points[i].x * points[i].x;
    }
    
    // 计算斜率和截距
    double denominator = num * sum_x2 - sum_x * sum_x;
    if (fabs(denominator) < 1e-6) {
        // Vertical line case
        double x = sum_x / num;
        Point pt1(x, line_bottom);
        Point pt2(x, line_top);
        line(*img, pt1, pt2, Scalar(0, 0, 255), 2);
        return;
    }
    
    double slope = (num * sum_xy - sum_x * sum_y) / denominator;
    double intercept = (sum_y - slope * sum_x) / num;
    
    // 计算线段端点
    Point pt1, pt2;
    
    // 使用 line_bottom 和 line_top 作为 y 坐标
    // 默认值: line_bottom = 0, line_top = img->rows
    int y1 = line_bottom;
    int y2 = line_top;
    
    // 计算相应的 x 坐标
    int x1 = static_cast<int>((y1 - intercept) / slope);
    int x2 = static_cast<int>((y2 - intercept) / slope);
    
    pt1 = Point(x1, y1);
    pt2 = Point(x2, y2);
    
    // Clip points to image boundaries
    pt1.x = max(0, min(pt1.x, img->cols - 1));
    pt2.x = max(0, min(pt2.x, img->cols - 1));
    pt1.y = max(0, min(pt1.y, img->rows - 1));
    pt2.y = max(0, min(pt2.y, img->rows - 1));
    
    // Draw the fitted line
    cv::line(*img, pt1, pt2, Scalar(0), 2);
    
}

void AcrossTrack_Step_ACROSS_OUT(Img_Store *Img_Store_p,Data_Path *Data_Path_p){
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

	two_point_line(&Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start), 
		Point(Data_Path_p->InflectionPointCoordinate[0][0], Data_Path_p->InflectionPointCoordinate[0][1]),Data_Path_p->search_print_h_max, image_h-JSON_TrackConfigData.Path_Search_Start);
	imgSearch_l_r(Img_Store_p,Data_Path_p);

	two_point_line(&Img_Store_p->Img_OTSU, Point(image_w/2+JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start), 
		Point(Data_Path_p->InflectionPointCoordinate[0][2], Data_Path_p->InflectionPointCoordinate[0][3]),Data_Path_p->search_print_h_max, image_h-JSON_TrackConfigData.Path_Search_Start);
	imgSearch_l_r(Img_Store_p,Data_Path_p);

	return;


	int left_border_num = 0, right_border_num = 0;
    Point left_border_point_bottom(0,0), left_border_point_top(0,0);
    Point right_border_point_bottom(0,0), right_border_point_top(0,0);

    // 计算搜索范围
    int start_row = image_h - JSON_TrackConfigData.Path_Search_Start;
    int end_row = Data_Path_p->search_print_h_max;

    // 搜索左边界
    for (int i = start_row; i >= end_row; i--) {
        if (Data_Path_p->l_border[i] <= 5) {
            left_border_num++;
            
            // 记录边界点
            if (left_border_num == 1) {
                left_border_point_bottom = Point(Data_Path_p->l_border[i], i);
            }
            left_border_point_top = Point(Data_Path_p->l_border[i], i);
        }
    }

    // 搜索右边界
    for (int i = start_row; i >= end_row; i--) {
        if (Data_Path_p->r_border[i] >= image_w - 5) {
            right_border_num++;
            
            // 记录边界点
            if (right_border_num == 1) {
                right_border_point_bottom = Point(Data_Path_p->r_border[i], i);
            }
            right_border_point_top = Point(Data_Path_p->r_border[i], i);
        }
    }

	if(left_border_point_bottom.y < Data_Path_p->InflectionPointCoordinate[0][1]){		// 边界左侧底部点在拐点上方
		// Point points[5];
		// int y = Data_Path_p->InflectionPointCoordinate[0][1];
		// for(int i = 0;i<5; i++){
		// 	points[i] = Point(Data_Path_p->l_border[y], y);
		// 	y++;
		// }
		// Points_to_line_drew(&Img_Store_p->Img_OTSU, points, 5, image_h-JSON_TrackConfigData.Path_Search_Start, Data_Path_p->search_print_h_max);
		// two_point_line(&Img_Store_p->Img_OTSU, Point(image_w/2-JSON_TrackConfigData.Track_width/2, image_h-JSON_TrackConfigData.Path_Search_Start), 
		// Point(Data_Path_p->InflectionPointCoordinate[0][0], Data_Path_p->InflectionPointCoordinate[0][1]), Data_Path_p->search_print_h_max);
		imgSearch_l_r(Img_Store_p,Data_Path_p);
	}else{
		// Point points[5];
		// int y = Data_Path_p->InflectionPointCoordinate[0][1];
		// for(int i = 0;i<5; i++){
		// 	points[i] = Point(Data_Path_p->l_border[y], y);
		// 	y--;
		// }
		// Points_to_line_drew(&Img_Store_p->Img_OTSU, points, 5, image_h-JSON_TrackConfigData.Path_Search_Start, Data_Path_p->search_print_h_max);
		imgSearch_l_r(Img_Store_p,Data_Path_p);
	}

	if(right_border_point_bottom.y < Data_Path_p->InflectionPointCoordinate[0][3]){		// 边界右侧底部点在拐点上方
		// Point points[5];
		// int y = Data_Path_p->InflectionPointCoordinate[0][3];
		// for(int i = 0;i<5; i++){
		// 	points[i] = Point(Data_Path_p->r_border[y], y);
		// 	y++;
		// }
		// Points_to_line_drew(&Img_Store_p->Img_OTSU, points, 5, image_h-JSON_TrackConfigData.Path_Search_Start, Data_Path_p->search_print_h_max);
		imgSearch_l_r(Img_Store_p,Data_Path_p);
	}else{
		// Point points[5];
		// int y = Data_Path_p->InflectionPointCoordinate[0][3];
		// for(int i = 0;i<5; i++){
		// 	points[i] = Point(Data_Path_p->r_border[y], y);
		// 	y--;
		// }
		// Points_to_line_drew(&Img_Store_p->Img_OTSU, points, 5, image_h-JSON_TrackConfigData.Path_Search_Start, Data_Path_p->search_print_h_max);
		imgSearch_l_r(Img_Store_p,Data_Path_p);
	}

}

