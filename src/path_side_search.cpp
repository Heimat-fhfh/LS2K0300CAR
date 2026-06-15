#include "common_system.h"
#include "common_program.h"
#include "path_refactor.h"

using namespace std;
using namespace cv;

void ImgSideLineTransitionSearch(Img_Store *Img_Store_p,Data_Path *Data_Path_p);


int my_abs(int value)
{
	if (value >= 0) return value;
	else return -value;
}

// 获取左边界
// 获取左边界
void get_left(uint16 total_L, Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    uint16 i = 0;
    uint16 j = 0;
    uint16 h = 0;
    
    // 初始化
    for (i = 0; i < RESULT_ROW; i++)
    {
        Data_Path_p->l_border[i] = border_min;
    }
    
    h = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;
    
    // 左边
    for (j = 0; j < total_L; j++)
    {
        if (Data_Path_p->points_l[j][1] == h)
        {
            uint16 current_x = Data_Path_p->points_l[j][0] + 1;
            uint16 center_x = image_w / 2;  // 图像宽度宏定义 image_w
            
            // 如果当前行还没有设置边界，或者当前点比已记录的点更靠近中心，则更新
            if (Data_Path_p->l_border[h] == border_min ||
                abs((int)current_x - (int)center_x) < abs((int)Data_Path_p->l_border[h] - (int)center_x))
            {
                Data_Path_p->l_border[h] = current_x;
            }
        }
        else if (Data_Path_p->points_l[j][1] < h)
        {
            // 当 y 坐标小于当前高度 h 时，说明这一行已经处理完毕，移动到上一行
            h--;
            // 回退 j 以便重新检查当前 point 在新 h 下是否匹配
            if (h == 0) break;
            j--;  // 重新处理同一个点，因为它的 y 可能等于新的 h
        }
    }
}

// 获取右边界
void get_right(uint16 total_R, Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    uint16 i = 0;
    uint16 j = 0;
    uint16 h = 0;
    
    for (i = 0; i < RESULT_ROW; i++)
    {
        Data_Path_p->r_border[i] = border_max;  // 右边线初始化放到最右边
    }
    
    h = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;
    
    // 右边
    for (j = 0; j < total_R; j++)
    {
        if (Data_Path_p->points_r[j][1] == h)
        {
            uint16 current_x = Data_Path_p->points_r[j][0] - 1;
            uint16 center_x = image_w / 2;  // 图像宽度宏定义 image_w
            
            // 如果当前行还没有设置边界，或者当前点比已记录的点更靠近中心，则更新
            if (Data_Path_p->r_border[h] == border_max ||
                abs((int)current_x - (int)center_x) < abs((int)Data_Path_p->r_border[h] - (int)center_x))
            {
                Data_Path_p->r_border[h] = current_x;
            }
        }
        else if (Data_Path_p->points_r[j][1] < h)
        {
            // 当 y 坐标小于当前高度 h 时，说明这一行已经处理完毕，移动到上一行
            h--;
            // 回退 j 以便重新检查同一个点在新 h 下是否匹配
            if (h == 0) break;
            j--;  // 重新处理同一个点
        }
    }
}

/*
    ImgPathSearch说明
    对赛道进行寻边线处理以此寻找路径线
    将边线坐标存储至 Data_Path_p -> SideCoordinate 中
    将路径线坐标存储至 Data_Path_p -> TrackCoordinate 中
*/
void ImgPathSearch(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    // 变量设置
    //————————————————————————————————————————————————————————————————————————————————————//
    // 边线坐标
    int X[2] = {0};
    int Y = 0;

    // 设置路径起始坐标
    Data_Path_p -> TrackCoordinate[0][0] = ((Data_Path_p -> SideCoordinate_Eight[JSON_TrackConfigData.Path_Search_Start-JSON_TrackConfigData.Side_Search_Start+1][0])+(Data_Path_p -> SideCoordinate_Eight[JSON_TrackConfigData.Path_Search_Start-JSON_TrackConfigData.Side_Search_Start+1][2]))/2;
    Data_Path_p -> TrackCoordinate[0][1] = 239-(JSON_TrackConfigData.Path_Search_Start);

    int NumSearch = 0;  // 坐标数组的行序号
    //————————————————————————————————————————————————————————————————————————————————————//

    // 寻线
    //————————————————————————————————————————————————————————————————————————————————————//
    if(Img_Store_p -> ImgNum <= 5)
    {

    }
    for(Y = 239-(JSON_TrackConfigData.Path_Search_Start);Y >= 239-(JSON_TrackConfigData.Path_Search_End);Y--)
    {
        // 左边线
        for(X[0] = (Data_Path_p -> TrackCoordinate[NumSearch][0]);X[0] >= 0;X[0]--)
        {
            if((Img_Store_p -> Img_OTSU).at<uchar>(Y,X[0]) == 255)
            {
                (Data_Path_p -> SideCoordinate[NumSearch][0]) = X[0];
                (Data_Path_p -> SideCoordinate[NumSearch][1]) = Y;

                break;
            }
        }
        // 右边线
        for(X[1] = (Data_Path_p -> TrackCoordinate[NumSearch][0]);X[1] <= 319;X[1]++)
        {
            if((Img_Store_p -> Img_OTSU).at<uchar>(Y,X[1]) == 255)
            {
                (Data_Path_p -> SideCoordinate[NumSearch][2]) = X[1];
                (Data_Path_p -> SideCoordinate[NumSearch][3]) = Y;

                break;
            }
        }
        if(NumSearch != 0)
        {
            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate[NumSearch][0]),(Data_Path_p -> SideCoordinate[NumSearch][1])),1,Scalar(0,0,255),1);	// 左边线画点
            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate[NumSearch][2]),(Data_Path_p -> SideCoordinate[NumSearch][3])),1,Scalar(0,0,255),1);	// 右边线画点
        }
        else
        {
            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate[NumSearch][0]),(Data_Path_p -> SideCoordinate[NumSearch][1])),6,Scalar(0,0,255),2);	// 左边线起点画点
            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate[NumSearch][2]),(Data_Path_p -> SideCoordinate[NumSearch][3])),6,Scalar(0,0,255),2);	// 右边线起点画点
        }

        // 寻边线提前结束条件：1.左右边线间距小于20 2.左右边线位置反了
        if(abs((Data_Path_p -> SideCoordinate[NumSearch][0])-(Data_Path_p -> SideCoordinate[NumSearch][2])) <= 20 || ((Data_Path_p -> SideCoordinate[NumSearch][0]) >= (Data_Path_p -> SideCoordinate[NumSearch][2])))
        {
            NumSearch--;
            JSON_TrackConfigData.Forward = Data_Path_p -> SideCoordinate[NumSearch][1];            
            break;
        }

        Data_Path_p -> TrackCoordinate[NumSearch][1] = Y;
        Data_Path_p->TrackCoordinate[NumSearch+1][0] =  (Data_Path_p->SideCoordinate[NumSearch][0] + 
                                                        Data_Path_p->SideCoordinate[NumSearch][2]) / 2;
        NumSearch++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//
}

/*
    ImgSideSearch 说明
    八邻域边线搜索函数。

    算法目标：
    - 在已经完成二值化的赛道图像上，从靠近图像下方的一条起始搜索线出发，
      分别向左、向右提取赛道边线点。
    - 采用 8 邻域的“黑到白”边缘转折判定，持续向上追踪左右边线。
    - 搜索结果会继续写入 Data_Path_p，供后续中线拟合、弯点识别、控制量计算使用。

    输入数据：
    - Img_Store_p->Img_OTSU：上一阶段生成的二值图像，是八邻域追踪的唯一图像来源。
    - Data_Path_p->JSON_TrackConfigData_v[0]：提供 Side_Search_Start / Side_Search_End 等搜索参数。
    - Data_Path_p->SideCoordinate_Eight / points_l / points_r / dir_l / dir_r / NumSearch / search_print_h_max：
      用于存储和回传搜索过程中的边线点、方向与终止状态。

    输出数据：
    - Data_Path_p->SideCoordinate_Eight：左右边线离散点缓存。
    - Data_Path_p->points_l / points_r：左右边线点序列。
    - Data_Path_p->dir_l / dir_r：每一步的生长方向。
    - Data_Path_p->NumSearch[0] / NumSearch[1]：左右边线最终点数。
    - Data_Path_p->search_print_h_max：左右边线相遇或达到有效终止条件时的最高点。
*/
void ImgSideSearch(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    if (Img_Store_p == nullptr || Data_Path_p == nullptr) {
        return;
    }

    const Mat& binary = Img_Store_p->Img_OTSU;
    if (binary.empty() || binary.type() != CV_8UC1) {
        cerr << "Error: Img_OTSU is empty or not CV_8UC1 in ImgSideSearch!" << endl;
        return;
    }

    const int image_width = binary.cols;
    const int image_height = binary.rows;
    const int last_col = image_width - 1;
    const int last_row = image_height - 1;

    // 变量设置
    // 寻种子变量设置
    // 边线坐标
    int X = 0;
    int Y = 0;
    // 设置种子寻找起始点横坐标
    static int StartX = 160;
    const int seed_row = last_row - JSON_TrackConfigData.Side_Search_Start;
    const int seed_row_next = seed_row - 1;

    // 八临域寻线变量设置
    int SeedGrow_Dir[16][4] = { {0,1,0,1} , {-1,1,1,1} , {-1,0,1,0} , {-1,-1,1,-1} , {0,-1,0,-1} , {1,-1,-1,-1} , {1,0,-1,0} , {1,1,-1,1} ,
                                {0,1,0,1} , {-1,1,1,1} , {-1,0,1,0} , {-1,-1,1,-1} , {0,-1,0,-1} , {1,-1,-1,-1} , {1,0,-1,0} , {1,1,-1,1}};    // 种子X,Y方向的生长向量：从正下方顺时针 和 从正下方逆时针 
    int Dir_Num = 0;
    int Dir_Num_Store = 0;

    int NumSearch[2] = {0};  // 坐标数组的行序号
    //————————————————————————————————————————————————————————————————————————————————————//

    // 八邻域寻边线
    //————————————————————————————————————————————————————————————————————————————————————//
    // 确定种子寻找起始点
    // 前5帧默认为160开始向左右寻找
    // 之后所有帧的起始点由上一帧的中点决定
    if(Img_Store_p -> ImgNum > 5)
    {
        StartX = ((Data_Path_p -> SideCoordinate_Eight[0][0])+(Data_Path_p -> SideCoordinate_Eight[0][2]))/2;
    }
    // 八邻域种子寻找
    if(NumSearch[0] <= 1 && NumSearch[1] <= 1)
    {
        for(Y = seed_row; Y >= seed_row_next; --Y)
        {
            // 左边线
            for(X = StartX; X >= 0; --X)
            {
                if(binary.at<uchar>(Y, X) == 255)
                {
                    // cout << "L_SIDE" << endl;
                    (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0]) = X;
                    (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1]) = Y;

                    break;
                }
            }
            // 右边线
            for(X = StartX; X <= last_col; ++X)
            {
                if(binary.at<uchar>(Y, X) == 255)
                {
                    // cout << "R_SIDE" << endl;
                    (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2]) = X;
                    (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3]) = Y;

                    break;
                }
            }

            if(NumSearch[0] == 0 && NumSearch[1] == 0)
            {
                circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[0][0]),(Data_Path_p -> SideCoordinate_Eight[0][1])),6,Scalar(255,0,255),2);	//左边线起点画点
                circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[0][2]),(Data_Path_p -> SideCoordinate_Eight[0][3])),6,Scalar(255,0,255),2);	//右边线起点画点
            }
            else
            {
                circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0]),(Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1])),1,Scalar(255,0,255),1);	//左边线画点
                circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2]),(Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3])),1,Scalar(255,0,255),1);	//右边线画点

            }
           
            NumSearch[0]++;
            NumSearch[1]++;
        }
    }

    // 八邻域寻线
    if(NumSearch[0] >= 2 && NumSearch[1] >= 2)
    {
        // 左边线寻线循环
        while(true)
        {
            // 左边线
            for(Dir_Num = Dir_Num_Store;Dir_Num <= Dir_Num_Store+7;Dir_Num++)
            {
                if(binary.at<uchar>((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-1][1])+SeedGrow_Dir[Dir_Num][1],(Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-1][0])+SeedGrow_Dir[Dir_Num][0]) == 0)
                {
                    if(Dir_Num-1 >= 0){Dir_Num = Dir_Num;}
                    else{Dir_Num = Dir_Num+8;}

                    Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0] = Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-1][0]+SeedGrow_Dir[Dir_Num-1][0];
                    Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1] = Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-1][1]+SeedGrow_Dir[Dir_Num-1][1];

                    // 开发调试显示
                    // cout << Data_Path_p -> SideCoordinate_Eight[NumSearch-1][0] << "  " << SeedGrow_Dir[Dir_Num-1][0] << "  " << Data_Path_p -> SideCoordinate_Eight[NumSearch-1][1] << "  " << SeedGrow_Dir[Dir_Num-1][1] << endl;
                    // cout << Data_Path_p -> SideCoordinate_Eight[NumSearch][0] << "  " << Data_Path_p -> SideCoordinate_Eight[NumSearch][1] << endl;
                    // cout << NumSearch << endl;
                    // cout << Dir_Num-1 << endl;

                    // 下次种子生长向量起始序号
                    if(Dir_Num >= 3){Dir_Num_Store = Dir_Num-3;}
                    if(Dir_Num <= 2){Dir_Num_Store = Dir_Num+5;}
                    // 防止种子生长向量越界
                    if(Dir_Num >= 11){Dir_Num_Store = Dir_Num-11;}    // *_Dir_Num-3-8
                    if(Dir_Num <= 10 && Dir_Num >= 8){Dir_Num_Store = Dir_Num-3;}   // *_Dir_Num++5-8                    

                    break;
                }
            }

            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0]),(Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1])),1,Scalar(255,0,255),1);	//左边线画点
            
            // 循环退出条件：1.寻线到寻线结束点和起始点 2.寻线折返 3.寻线到中心线 4.坐标数量大于阈值
            if((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1]) <= last_row-(JSON_TrackConfigData.Side_Search_End) || (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1]) >= last_row-(JSON_TrackConfigData.Side_Search_Start))
            {
                break;
            } 

            if(NumSearch[0] >= 20 && (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][1]) == (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-20][1]))
            {
                if(abs((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0]) - (Data_Path_p -> SideCoordinate_Eight[NumSearch[0]-20][0])) <= 10)
                {
                    break;
                }
            }

		    if((Data_Path_p -> SideCoordinate_Eight[NumSearch[0]][0]) > last_col)
            {
                break;
            } 

            NumSearch[0]++;
            Data_Path_p -> NumSearch[0] = NumSearch[0]-1;

            if(NumSearch[0] >= 500)
            {
                break;
            }
        }
        // 右边线寻线循环
        while(true)
        {
            // 左边线
            for(Dir_Num = Dir_Num_Store;Dir_Num <= Dir_Num_Store+7;Dir_Num++)
            {
                if(binary.at<uchar>((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-1][3])+SeedGrow_Dir[Dir_Num][3],(Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-1][2])+SeedGrow_Dir[Dir_Num][2]) == 0)
                {
                    if(Dir_Num-1 >= 0){Dir_Num = Dir_Num;}
                    else{Dir_Num = Dir_Num+8;}

                    Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2] = Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-1][2]+SeedGrow_Dir[Dir_Num-1][2];
                    Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3] = Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-1][3]+SeedGrow_Dir[Dir_Num-1][3];

                    // 开发调试显示
                    // cout << Data_Path_p -> SideCoordinate_Eight[NumSearch-1][2] << "  " << SeedGrow_Dir[Dir_Num-1][2] << "  " << Data_Path_p -> SideCoordinate_Eight[NumSearch-1][3] << "  " << SeedGrow_Dir[Dir_Num-1][3] << endl;
                    // cout << Data_Path_p -> SideCoordinate_Eight[NumSearch][2] << "  " << Data_Path_p -> SideCoordinate_Eight[NumSearch][3] << endl;
                    // cout << NumSearch << endl;
                    // cout << Dir_Num-1 << endl;

                    // 下次种子生长向量起始序号
                    if(Dir_Num >= 3){Dir_Num_Store = Dir_Num-3;}
                    if(Dir_Num <= 2){Dir_Num_Store = Dir_Num+5;}
                    // 防止种子生长向量越界
                    if(Dir_Num >= 11){Dir_Num_Store = Dir_Num-11;}    // *_Dir_Num-3-8
                    if(Dir_Num <= 10 && Dir_Num >= 8){Dir_Num_Store = Dir_Num-3;}   // *_Dir_Num++5-8                    

                    break;
                }
            }

            circle((Img_Store_p -> Img_Track),Point((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2]),(Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3])),1,Scalar(255,0,255),1);	//右边线画点
            
            // 循环退出条件：1.寻线到寻线结束点和起始点 2.寻线折返 3.寻线到中心线 4.坐标数量大于阈值
            if((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3]) <= last_row-(JSON_TrackConfigData.Side_Search_End) || (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3]) >= last_row-(JSON_TrackConfigData.Side_Search_Start))
            {
                break;
            } 

            if(NumSearch[1] >= 20 && (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][3]) == (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-20][3]))
            {
                if(abs((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2]) - (Data_Path_p -> SideCoordinate_Eight[NumSearch[1]-20][2])) <= 10)
                {
                    break;
                }
            }

            if((Data_Path_p -> SideCoordinate_Eight[NumSearch[1]][2]) < border_min)
            {
                break;
            } 

            NumSearch[1]++;
            Data_Path_p -> NumSearch[1] = NumSearch[1]-1;

            if(NumSearch[1] >= 500)
            {
                break;
            }
        }
    }
    //————————————————————————————————————————————————————————————————————————————————————//
}

void dataMove(Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    int i,j;
    //————————————————————————————————————————————————————————————————————————————————————//
    // 左边线坐标
    j = 0;
    for(i = 0;i < Data_Path_p -> NumSearch[0];i++)
    {
        Data_Path_p -> SideCoordinate_Eight[j][0] = Data_Path_p -> points_l[i][0];
        Data_Path_p -> SideCoordinate_Eight[j][1] = Data_Path_p -> points_l[i][1];
        j++;
    }
    // 右边线坐标
    j = 0;
    for(i = 0;i < Data_Path_p -> NumSearch[1];i++)
    {
        Data_Path_p -> SideCoordinate_Eight[j][2] = Data_Path_p -> points_r[i][0];
        Data_Path_p -> SideCoordinate_Eight[j][3] = Data_Path_p -> points_r[i][1];
        j++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//
    j = 0;
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->search_print_h_max;i++)
    {
        Data_Path_p -> SideCoordinate[j][0] = Data_Path_p -> l_border[i];
        Data_Path_p -> SideCoordinate[j][1] = i;
        j++;
    }
    j = 0;
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->search_print_h_max;i++)
    {
        Data_Path_p -> SideCoordinate[j][2] = Data_Path_p -> r_border[i];
        Data_Path_p -> SideCoordinate[j][3] = i;
        j++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//
    j = 0;
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->search_print_h_max;i++)
    {
        Data_Path_p -> TrackCoordinate[j][0] = Data_Path_p -> center_line[i];
        Data_Path_p -> TrackCoordinate[j][1] = i;
        j++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//

}

namespace {

inline bool in_image_bounds(int x, int y) {
    return x >= 0 && x < image_w && y >= 0 && y < image_h;
}

inline int clamp_int(int value, int low, int high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

bool find_seed_points_eight(const uint8 bin_image[image_h][image_w],
                            int start_row,
                            int start_x,
                            cv::Point& start_point_l,
                            cv::Point& start_point_r) {
    bool l_found = false;
    bool r_found = false;

    for (int x = start_x; x > border_min; --x) {
        if (bin_image[start_row][x] == 255 && bin_image[start_row][x - 1] == 0) {
            start_point_l = cv::Point(x, start_row);
            l_found = true;
            break;
        }
    }

    for (int x = start_x; x < border_max; ++x) {
        if (bin_image[start_row][x] == 255 && bin_image[start_row][x + 1] == 0) {
            start_point_r = cv::Point(x, start_row);
            r_found = true;
            break;
        }
    }

    return l_found && r_found;
}

bool advance_one_side_eight(const uint8 bin_image[image_h][image_w],
                            const cv::Point& center,
                            const int seeds[8][2],
                            cv::Point& out_next,
                            uint16& out_dir) {
    cv::Point next = center;
    int best_dir = -1;
    bool found = false;

    for (int i = 0; i < 8; ++i) {
        const int nx = center.x + seeds[i][0];
        const int ny = center.y + seeds[i][1];
        const int nx_next = center.x + seeds[(i + 1) & 7][0];
        const int ny_next = center.y + seeds[(i + 1) & 7][1];

        if (!in_image_bounds(nx, ny) || !in_image_bounds(nx_next, ny_next)) {
            continue;
        }

        if (bin_image[ny][nx] == 0 && bin_image[ny_next][nx_next] == 255) {
            if (!found || ny < next.y) {
                next.x = nx;
                next.y = ny;
                best_dir = i;
                found = true;
            }
        }
    }

    out_next = next;
    out_dir = (best_dir >= 0) ? static_cast<uint16>(best_dir) : 0;
    return found;
}

} // namespace

/*
    ImgSideSearchEightNeighborhood 说明
    现代化、结构化、安全的八邻域边线搜索主函数。
*/
void ImgSideSearchEightNeighborhood(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    if (Img_Store_p == nullptr || Data_Path_p == nullptr) {
        return;
    }
    if (Data_Path_p->JSON_TrackConfigData_v.empty()) {
        return;
    }

    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p->JSON_TrackConfigData_v[0];
    const Mat& binary = Img_Store_p->Img_OTSU;
    if (binary.empty() || binary.rows != image_h || binary.cols != image_w || binary.type() != CV_8UC1) {
        cerr << "Error: Img_OTSU is empty or has unexpected size/type in ImgSideSearchEightNeighborhood!" << endl;
        return;
    }

    for (int row = 0; row < image_h; ++row) {
        const uint8* row_ptr = binary.ptr<uint8>(row);
        std::copy_n(row_ptr, image_w, Img_Store_p->bin_image[row]);
    }

    memset(Data_Path_p->points_l, 0, sizeof(Data_Path_p->points_l));
    memset(Data_Path_p->points_r, 0, sizeof(Data_Path_p->points_r));
    memset(Data_Path_p->dir_l, 0, sizeof(Data_Path_p->dir_l));
    memset(Data_Path_p->dir_r, 0, sizeof(Data_Path_p->dir_r));
    Data_Path_p->NumSearch[0] = 0;
    Data_Path_p->NumSearch[1] = 0;

    const int start_row = clamp_int(RESULT_ROW - JSON_TrackConfigData.Path_Search_Start, 1, image_h - 2);
    const int end_row = clamp_int(RESULT_ROW - JSON_TrackConfigData.Side_Search_End, 0, image_h - 1);

    static int last_mid_x = image_w / 2;
    int start_x = image_w / 2;
    if (Img_Store_p->ImgNum > 5) {
        start_x = clamp_int(last_mid_x, 1, image_w - 2);
    }

    Point start_point_l;
    Point start_point_r;
    if (!find_seed_points_eight(Img_Store_p->bin_image, start_row, start_x, start_point_l, start_point_r)) {
        if (!find_seed_points_eight(Img_Store_p->bin_image, start_row, image_w / 2, start_point_l, start_point_r)) {
            return;
        }
    }

    last_mid_x = (start_point_l.x + start_point_r.x) / 2;

    static const int seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}
    };
    static const int seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };

    Point center_l = start_point_l;
    Point center_r = start_point_r;
    int left_count = 0;
    int right_count = 0;
    bool left_active = true;
    bool right_active = true;

    Data_Path_p->search_print_h_max = static_cast<uint16>(start_row);
    for (int step = 0; step < static_cast<int>(USE_num); ++step) {
        if (left_active && left_count < static_cast<int>(USE_num)) {
            Data_Path_p->points_l[left_count][0] = static_cast<uint16>(center_l.x);
            Data_Path_p->points_l[left_count][1] = static_cast<uint16>(center_l.y);
            ++left_count;
        }
        if (right_active && right_count < static_cast<int>(USE_num)) {
            Data_Path_p->points_r[right_count][0] = static_cast<uint16>(center_r.x);
            Data_Path_p->points_r[right_count][1] = static_cast<uint16>(center_r.y);
            ++right_count;
        }

        if (left_count > 0 && right_count > 0) {
            const int lx = static_cast<int>(Data_Path_p->points_l[left_count - 1][0]);
            const int ly = static_cast<int>(Data_Path_p->points_l[left_count - 1][1]);
            const int rx = static_cast<int>(Data_Path_p->points_r[right_count - 1][0]);
            const int ry = static_cast<int>(Data_Path_p->points_r[right_count - 1][1]);
            if (my_abs(lx - rx) <= 2 && my_abs(ly - ry) <= 2) {
                Data_Path_p->search_print_h_max = static_cast<uint16>((ly + ry) / 2);
                break;
            }
        }

        const bool left_row_valid = left_active && center_l.y > end_row && center_l.y <= start_row + 2;
        const bool right_row_valid = right_active && center_r.y > end_row && center_r.y <= start_row + 2;

        Point next_l = center_l;
        Point next_r = center_r;
        uint16 dir_l = 0;
        uint16 dir_r = 0;
        bool left_moved = false;
        bool right_moved = false;

        if (left_row_valid) {
            left_moved = advance_one_side_eight(Img_Store_p->bin_image, center_l, seeds_l, next_l, dir_l);
        } else {
            left_active = false;
        }

        if (right_row_valid) {
            right_moved = advance_one_side_eight(Img_Store_p->bin_image, center_r, seeds_r, next_r, dir_r);
        } else {
            right_active = false;
        }

        if (left_moved) {
            center_l = next_l;
            Data_Path_p->dir_l[left_count - 1] = dir_l;
        } else {
            left_active = false;
        }

        if (right_moved) {
            center_r = next_r;
            Data_Path_p->dir_r[right_count - 1] = dir_r;
        } else {
            right_active = false;
        }

        if (!left_active && !right_active) {
            break;
        }
    }

    Data_Path_p->NumSearch[0] = left_count;
    Data_Path_p->NumSearch[1] = right_count;

    if (Data_Path_p->search_print_h_max == static_cast<uint16>(start_row)) {
        if (left_count > 0 && right_count > 0) {
            Data_Path_p->search_print_h_max = static_cast<uint16>(std::min(static_cast<int>(Data_Path_p->points_l[left_count - 1][1]),
                                                                 static_cast<int>(Data_Path_p->points_r[right_count - 1][1])));
        } else if (left_count > 0) {
            Data_Path_p->search_print_h_max = static_cast<uint16>(Data_Path_p->points_l[left_count - 1][1]);
        } else if (right_count > 0) {
            Data_Path_p->search_print_h_max = static_cast<uint16>(Data_Path_p->points_r[right_count - 1][1]);
        }
    }

    // optimize_edge_lines(Data_Path_p,
    //                     image_w,
    //                     2.0F,
    //                     5,
    //                     static_cast<int>(USE_num));

    get_left(static_cast<uint16>(Data_Path_p->NumSearch[0]), Data_Path_p);
    get_right(static_cast<uint16>(Data_Path_p->NumSearch[1]), Data_Path_p);

    for (int i = Data_Path_p->search_print_h_max; i < image_h-JSON_TrackConfigData.Path_Search_Start; i++)
    {
        Data_Path_p->center_line[i] = (Data_Path_p->l_border[i] + Data_Path_p->r_border[i]) >> 1;
    }     //求中线
    // cout << "search_print_h_max: " << Data_Path_p->search_print_h_max << endl;


    dataMove(Data_Path_p);
    ImgSideLineTransitionSearch(Img_Store_p, Data_Path_p);

    // --- border edge traversal: count points in left/right border regions ---
    {
        const int BORDER_MARGIN = 5; // 边界区域阈值（像素），与 LEFT_EDGE/RIGHT_EDGE 保持一致
        int left_border_cnt = 0;
        int right_border_cnt = 0;

        // 统计左边线中位于左边界区域（x <= BORDER_MARGIN）的点
        for (int i = 0; i < left_count; i++) {
            if (Data_Path_p->points_l[i][0] <= static_cast<uint16>(BORDER_MARGIN)) {
                left_border_cnt++;
            }
        }

        // 统计右边线中位于右边界区域（x >= image_w - 1 - BORDER_MARGIN）的点
        for (int i = 0; i < right_count; i++) {
            if (Data_Path_p->points_r[i][0] >= static_cast<uint16>(image_w - 1 - BORDER_MARGIN)) {
                right_border_cnt++;
            }
        }

        Data_Path_p->BorderPointNum[0] = left_border_cnt;
        Data_Path_p->BorderPointNum[1] = right_border_cnt;
    }

}

void imgSearch_l_r(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    ImgSideSearchEightNeighborhood(Img_Store_p, Data_Path_p);
}

/*
    ImgSideLineTransitionSearch 说明
    直接扫描 bin_image 的左右边界列，记录 0/255 二值跳变点。
    左侧扫描列为 border_min，右侧扫描列为 border_max。
    EdgeLineColorBlockStart[side][0] 保存扫描起点；
    EdgeLineColorBlockStart[side][1..] 保存每次跳变后的第一个点，供 ImgLabel 标注。
    
    static constexpr int kEdgeLineColorBlockMax = 3;
    int EdgeLineColorBlockNum[2] = {0}; // 左右边线有效同色段数量，最多记录3段
    int EdgeLineJumpNum[2] = {0}; // 左右边线有效同色段之间的跳变次数
    int EdgeLineColorBlockColor[2][kEdgeLineColorBlockMax] = {{0}}; // 有效同色段颜色
    int EdgeLineColorBlockLength[2][kEdgeLineColorBlockMax] = {{0}}; // 有效同色段长度
    cv::Point EdgeLineColorBlockStart[2][kEdgeLineColorBlockMax] = {}; // 有效同色段起点
    cv::Point EdgeLineColorBlockEnd[2][kEdgeLineColorBlockMax] = {}; // 有效同色段终点
*/
void ImgSideLineTransitionSearch(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    const int LEFT_EDGE = 5;
    const int RIGHT_EDGE = image_w - 1 - 5;

    const int start_row = static_cast<int>(Data_Path_p->search_print_h_max);
    const int end_row = image_h - JSON_TrackConfigData.Path_Search_Start;

    Data_Path_p->EdgeLineJumpNum[0] = 0;
    Data_Path_p->EdgeLineJumpNum[1] = 0;

    if (start_row >= end_row) return;

    // --- left border (side 0) ---
    {
        bool prev_bnd = (Data_Path_p->l_border[start_row] <= LEFT_EDGE);
        int jump_cnt = 0;
        int last_jump_row = -999;

        for (int r = start_row; r < end_row; r++) {
            uint16 bx = Data_Path_p->l_border[r];
            bool cur_bnd = (bx <= LEFT_EDGE);
            if (cur_bnd && !prev_bnd && jump_cnt < 3 && (r - last_jump_row) >= 20) {
                Data_Path_p->EdgeLineColorBlockStart[0][jump_cnt] = cv::Point(bx, r);
                last_jump_row = r;
                jump_cnt++;
            }
            prev_bnd = cur_bnd;
        }
        Data_Path_p->EdgeLineJumpNum[0] = jump_cnt;
    }

    // --- right border (side 1) ---
    {
        bool prev_bnd = (Data_Path_p->r_border[start_row] >= RIGHT_EDGE);
        int jump_cnt = 0;
        int last_jump_row = -999;

        for (int r = start_row; r < end_row; r++) {
            uint16 bx = Data_Path_p->r_border[r];
            bool cur_bnd = (bx >= RIGHT_EDGE);
            if (cur_bnd && !prev_bnd && jump_cnt < 3 && (r - last_jump_row) >= 20) {
                Data_Path_p->EdgeLineColorBlockStart[1][jump_cnt] = cv::Point(bx, r);
                last_jump_row = r;
                jump_cnt++;
            }
            prev_bnd = cur_bnd;
        }
        Data_Path_p->EdgeLineJumpNum[1] = jump_cnt;
    }
}
