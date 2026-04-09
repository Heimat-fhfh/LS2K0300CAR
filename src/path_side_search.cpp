#include "common_system.h"
#include "common_program.h"
#include "path_refactor.h"

using namespace std;
using namespace cv;


int my_abs(int value)
{
	if (value >= 0) return value;
	else return -value;
}

// 获取左边界
void get_left(uint16 total_L,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	uint16 i = 0;
	uint16 j = 0;
	uint16 h = 0;
	//初始化
	for (i = 0;i < RESULT_ROW;i++)
	{
		Data_Path_p->l_border[i] = border_min;
	}
	h = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;
	//左边
	for (j = 0; j < total_L; j++)
	{
        // printf("第%d点的y:%d,正在寻找的高度:%d\n", j,Data_Path_p->points_l[j][1],h);
		if (Data_Path_p->points_l[j][1] == h)
		{
			Data_Path_p->l_border[h] = Data_Path_p->points_l[j][0] + 1;
            // printf("%d\n", j);
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)
		{
			break;//到最后一行退出
		}
	}
}

// 获取右边界
void get_right(uint16 total_R,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
	uint16 i = 0;
	uint16 j = 0;
	uint16 h = 0;
	for (i = 0; i < RESULT_ROW; i++)
	{
		Data_Path_p->r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (Data_Path_p->points_r[j][1] == h)
		{
			Data_Path_p->r_border[h] = Data_Path_p->points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
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
    - Data_Path_p->SideCoordinate_Eight / points_l / points_r / dir_l / dir_r / NumSearch / hightest：
      用于存储和回传搜索过程中的边线点、方向与终止状态。

    输出数据：
    - Data_Path_p->SideCoordinate_Eight：左右边线离散点缓存。
    - Data_Path_p->points_l / points_r：左右边线点序列。
    - Data_Path_p->dir_l / dir_r：每一步的生长方向。
    - Data_Path_p->NumSearch[0] / NumSearch[1]：左右边线最终点数。
    - Data_Path_p->hightest：左右边线相遇或达到有效终止条件时的最高点。

    这次改动的重点：
    - 使用二值图的实际宽高替代固定的 239 / 319 魔数，减少硬编码。
    - 补齐输入检查，防止空图或尺寸不匹配时继续访问。
    - 修正“左右边线是否都至少找到两个点”的判断笔误。
    - 将搜索起点、结束条件和回退条件写得更明确，降低后续维护成本。
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
    //————————————————————————————————————————————————————————————————————————————————————//
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
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->hightest;i++)
    {
        Data_Path_p -> SideCoordinate[j][0] = Data_Path_p -> l_border[i];
        Data_Path_p -> SideCoordinate[j][1] = i;
        j++;
    }
    j = 0;
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->hightest;i++)
    {
        Data_Path_p -> SideCoordinate[j][2] = Data_Path_p -> r_border[i];
        Data_Path_p -> SideCoordinate[j][3] = i;
        j++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//
    j = 0;
    for(i = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;i < Data_Path_p->hightest;i++)
    {
        Data_Path_p -> TrackCoordinate[j][0] = Data_Path_p -> center_line[i];
        Data_Path_p -> TrackCoordinate[j][1] = i;
        j++;
    }
    //————————————————————————————————————————————————————————————————————————————————————//

}

/*
    imgSearch_l_r 说明
    八邻域边线提取前的二值图准备函数。

    输入数据：
    - Img_Store_p->Img_OTSU：上一阶段完成预处理后的二值图，作为八邻域搜索的唯一图像依据。
    - Data_Path_p：承载左右边线点集、搜索方向、最高点和边界缓存的路径数据结构。

    输出数据：
    - Img_Store_p->bin_image：将二值图按行展开到二维数组，方便后续按坐标快速访问。
    - 后续的八邻域搜索会继续写入 Data_Path_p->points_l / points_r / dir_l / dir_r / hightest 等字段。

    实现方式：
    - 先校验输入图像是否为空，以及尺寸和通道类型是否满足固定寻线假设。
    - 再将 Mat 中的每一行复制到 bin_image，保留连续的二维数组访问方式。
    - 这一步本身不做寻线，只负责把图像数据转换成更适合八邻域算法的访问格式。

    为什么这样优化：
    - 避免在后续像素访问中频繁调用 Mat::at，降低函数调用和边界检查开销。
    - 尺寸不匹配时直接返回，避免使用错误图像导致越界和脏数据传播。
    - 行级复制使用 std::copy_n，比分像素逐个赋值更紧凑，也更容易被编译器优化。
*/
void imgSearch_l_r(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    // printf("获取八邻域起始点\n");
	int i = 0, j = 0, l_found = 0, r_found = 0;

    Point start_point_l,start_point_r;

    int start_row = RESULT_ROW - JSON_TrackConfigData.Path_Search_Start;

/*************************************************************************************************************
 *********************************        将Mat转化为数组更高效        *****************************************
 *************************************************************************************************************/

    const Mat& binary = Img_Store_p->Img_OTSU;
    if (binary.empty() || binary.rows != image_h || binary.cols != image_w || binary.type() != CV_8UC1) {
        cerr << "Error: Img_OTSU is empty or has unexpected size/type in imgSearch_l_r!" << endl;
        return;
    }

    for (int row = 0; row < image_h; ++row) {
        const uint8* row_ptr = binary.ptr<uint8>(row);
        std::copy_n(row_ptr, image_w, Img_Store_p->bin_image[row]);
    }
    
    
    // ApplyInversePerspective(Img_Store_p);

/*************************************************************************************************************
 **********************************        寻找左右两边的起始点        *****************************************
 *************************************************************************************************************/

	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l.x = i;    start_point_l.y = start_row;
		if (Img_Store_p->bin_image[start_row][i] == 255 && Img_Store_p->bin_image[start_row][i - 1] == 0)
		{
			// printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r.x = i;    start_point_r.y = start_row;
		if (Img_Store_p->bin_image[start_row][i] == 255 && Img_Store_p->bin_image[start_row][i + 1] == 0)
		{
			// printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if (l_found && r_found) ;   // 左右都找到了起点
	else return;                // 没找到起点

/*************************************************************************************************************
 ***************************************        变量初始化        *********************************************
 *************************************************************************************************************/

	//左边变量
	uint16 search_filds_l[8][2] = { {  0 } };
	Point center_point_l = Point(0,0);
	uint16 index_l = 0;
	uint16 temp_l[8][2] = { {  0 } };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint16 search_filds_r[8][2] = { {  0 } };
	Point center_point_r = Point(0,0);
	uint16 index_r = 0;//索引下标
	uint16 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = 0; r_data_statics = 0;

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l = start_point_l;
	center_point_r = start_point_r;

	//开启邻域循环
    uint16 break_flag = (uint16)USE_num;


/*************************************************************************************************************
 ***************************************        八邻域巡线        *********************************************
 *************************************************************************************************************/

	while (break_flag--)
	{
		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l.x + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l.y + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		Data_Path_p->points_l[l_data_statics][0] = center_point_l.x;//x
		Data_Path_p->points_l[l_data_statics][1] = center_point_l.y;//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r.x + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r.y + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		Data_Path_p->points_r[r_data_statics][0] = center_point_r.x;//x
		Data_Path_p->points_r[r_data_statics][1] = center_point_r.y;//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (Img_Store_p->bin_image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& Img_Store_p->bin_image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				Data_Path_p->dir_l[l_data_statics - 1] = (i);//记录生长方向
			}

			if (index_l)
			{
				//更新坐标点
				center_point_l.x = temp_l[0][0];//x
				center_point_l.y = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l.y > temp_l[j][1])
					{
						center_point_l.x = temp_l[j][0];//x
						center_point_l.y = temp_l[j][1];//y
					}
				}
			}

		}
		if ((Data_Path_p->points_r[r_data_statics][0] == Data_Path_p->points_r[r_data_statics - 1][0] && Data_Path_p->points_r[r_data_statics][0] == Data_Path_p->points_r[r_data_statics - 2][0]
			&& Data_Path_p->points_r[r_data_statics][1] == Data_Path_p->points_r[r_data_statics - 1][1] && Data_Path_p->points_r[r_data_statics][1] == Data_Path_p->points_r[r_data_statics - 2][1])
			|| (Data_Path_p->points_l[l_data_statics - 1][0] == Data_Path_p->points_l[l_data_statics - 2][0] && Data_Path_p->points_l[l_data_statics - 1][0] == Data_Path_p->points_l[l_data_statics - 3][0]
				&& Data_Path_p->points_l[l_data_statics - 1][1] == Data_Path_p->points_l[l_data_statics - 2][1] && Data_Path_p->points_l[l_data_statics - 1][1] == Data_Path_p->points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(Data_Path_p->points_r[r_data_statics][0] - Data_Path_p->points_l[l_data_statics - 1][0]) < 2
			&& my_abs(Data_Path_p->points_r[r_data_statics][1] - Data_Path_p->points_l[l_data_statics - 1][1] < 2)
			)
		{
			//printf("\n左右相遇退出\n");	
			Data_Path_p->hightest = (Data_Path_p->points_r[r_data_statics][1] + Data_Path_p->points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((Data_Path_p->points_r[r_data_statics][1] < Data_Path_p->points_l[l_data_statics - 1][1]))
		{
			// printf("\n如果左边比右边高了，左边等待右边\n");
			continue;//如果左边比右边高了，左边等待右边
		}
		if (Data_Path_p->dir_l[l_data_statics - 1] == 7
			&& (Data_Path_p->points_r[r_data_statics][1] > Data_Path_p->points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l.x = Data_Path_p->points_l[l_data_statics - 1][0];//x
			center_point_l.y = Data_Path_p->points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
        // if (center_point_l.y > JSON_TrackConfigData.Path_Search_End)
        // {
        //     printf("达到巡线最高点，退出");
        //     break;
        // }
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (Img_Store_p->bin_image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& Img_Store_p->bin_image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				Data_Path_p->dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, Data_Path_p->dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r.x = temp_r[0][0];//x
				center_point_r.y = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r.y > temp_r[j][1])
					{
						center_point_r.x = temp_r[j][0];//x
						center_point_r.y = temp_r[j][1];//y
					}
				}

			}
		}


	}

/*************************************************************************************************************
 ***********************************        传递找到的点的数量        ******************************************
 *************************************************************************************************************/
	Data_Path_p->NumSearch[0] = l_data_statics;
	Data_Path_p->NumSearch[1] = r_data_statics;

    // 参考算法迁移：对迷宫法提取的边线做三角平滑和等距采样，减少噪点并统一点密度。
    // 参数说明：
    // - sample_dist_px = 2.0F: 约每 2 像素采样一个点，兼顾精度与计算量。
    // - blur_kernel = 5: 中等平滑强度，降低边线抖动。
    // - max_points_per_side = USE_num: 与现有缓存保持一致，避免写越界。
    optimize_edge_lines(Data_Path_p,
                        image_w,
                        2.0F,
                        5,
                        static_cast<int>(USE_num));

/*************************************************************************************************************
 ***********************************        获取右左边界        ******************************************
 *************************************************************************************************************/

    get_left(l_data_statics,Data_Path_p);
    get_right(r_data_statics,Data_Path_p);

    dataMove(Data_Path_p);
    // Img_Store_p->LoadData(Img_Store_p->Img_OTSU_Unpivot,Img_Store_p->bin_image);
}

