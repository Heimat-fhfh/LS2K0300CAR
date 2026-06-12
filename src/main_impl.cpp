#include "main.hpp"
#include "camera_calibration.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

void RunCameraCatchTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    // imgProcess.ImgCompress(Img_Store_p->Img_Color, false);   // 图像压缩
    imgProcess_p->imgPreProc(Img_Store_p,Data_Path_p,Function_EN_p); // 图像预处理
    imgSearch_l_r(Img_Store_p,Data_Path_p);   // 边线八邻域寻线
    // judge_p->TransitionScanDetect(Img_Store_p, Data_Path_p, Function_EN_p); // 独立黑块检测
    judge_p->Search_Data_Analysis(Img_Store_p, Data_Path_p, Function_EN_p);
}

void ProcessAlgo_CircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p->JSON_TrackConfigData_v[0];    // 赛道识别设置参数
    // 圆环赛道状态机只在本轮周期内完成“补线 + 寻线 + 回到图像循环”的闭环。
    // 其中 Circle_Track_Step 由决策模块提前写入，这里只负责按步骤执行对应补线算法。
    // cout << "CircleTrack_Step: " << Data_Path_p->Circle_Track_Step << endl;
    static int status_change_count = 0; // 状态变化计数器，用于监控状态持续时间
    switch(Data_Path_p->Circle_Track_Step)
    {
        case IN_PREPARE:
        {
            // 准备入环：根据当前圆环方向先做预补线，避免进入环口时边线断裂。
            if (!Data_Path_p->black_left_found && !Data_Path_p->black_right_found) {
                status_change_count++;
                if (status_change_count > 2) { // 如果连续多帧都未找到黑块，进入下一阶段继续补线提高识别率
                    Data_Path_p->Circle_Track_Step = IN_PREPARE_2;
                    status_change_count = 0; // 重置计数器
                    CircleTrack_Step_IN_Prepare_2(Img_Store_p,Data_Path_p);
                }else {
                    CircleTrack_Step_IN_Prepare_2(Img_Store_p,Data_Path_p);   // 入环补线
                }
            }else {
                status_change_count = 0; // 如果找到黑块，重置计数器
                CircleTrack_Step_IN_Prepare(Img_Store_p,Data_Path_p);   // 准备入环补线
            }
            
            break;
        }
        case IN_PREPARE_2:
        {
            // 准备入环2：增加一个准备阶段提高识别率，继续补线并观察状态变化。
            if (Data_Path_p->l_border[image_h-JSON_TrackConfigData.Path_Search_Start] > 10 && Data_Path_p->r_border[image_h-JSON_TrackConfigData.Path_Search_Start] < 310) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = IN;
                    status_change_count = 0; // 重置计数器
                    CircleTrack_Step_IN(Img_Store_p,Data_Path_p);
                }else {
                    CircleTrack_Step_IN(Img_Store_p,Data_Path_p);   // 入环补线
                }
            } else {
                status_change_count = 0;
                CircleTrack_Step_IN_Prepare_2(Img_Store_p,Data_Path_p);   // 准备入环补线
            }
            break;
        }
        case IN:
        {
            // 入环：沿已确认的圆环方向继续补线，保证进入环内后仍能稳定寻线。
            if (Data_Path_p->InflectionPointNum[0] <= 0 && Data_Path_p->InflectionPointNum[1] <= 0) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = IN_CIRCLE;
                    status_change_count = 0; // 重置计数器
                }else {
                    // CircleTrack_Step_IN(Img_Store_p,Data_Path_p);   // 入环补线
                }
            } else {
                status_change_count = 0;
                CircleTrack_Step_IN(Img_Store_p,Data_Path_p);   // 入环补线
            }
            break;
        }
        case IN_CIRCLE:
        {
            // 圆环内
            // 入环：沿已确认的圆环方向继续补线，保证进入环内后仍能稳定寻线。
            if (Data_Path_p->InflectionPointNum[0] > 0 || Data_Path_p->InflectionPointNum[1] > 0) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = OUT_PREPARE;
                    status_change_count = 0; // 重置计数器
                    CircleTrack_Step_OUT_PREPARE(Img_Store_p,Data_Path_p);
                }else {
                    CircleTrack_Step_OUT_PREPARE(Img_Store_p,Data_Path_p);   // 圆环内补线
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        case OUT_PREPARE:
        {
            if (Data_Path_p->InflectionPointNum[0] <= 0 && Data_Path_p->InflectionPointNum[1] <= 0) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = OUT_STRIGHT;
                    status_change_count = 0; // 重置计数器
                }else {
                    // CircleTrack_Step_OUT_PREPARE(Img_Store_p,Data_Path_p);   // 出环补线
                }
            } else {
                status_change_count = 0;
                CircleTrack_Step_OUT_PREPARE(Img_Store_p,Data_Path_p);   // 准备出环补线
            }
            break;
        }
        case OUT_STRIGHT:
        {
            if (Data_Path_p->InflectionPointNum[0] > 0 || Data_Path_p->InflectionPointNum[1] > 0) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = OUT;
                    status_change_count = 0; // 重置计数器
                }else {
                    CircleTrack_Step_OUT(Img_Store_p,Data_Path_p);   // 出环寻线
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        case OUT:
        {
            if (Data_Path_p->l_border[image_h-JSON_TrackConfigData.Path_Search_Start] > 10 && Data_Path_p->r_border[image_h-JSON_TrackConfigData.Path_Search_Start] < 310) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Circle_Track_Step = INIT_CIRCLE;
                    Data_Path_p->Loop_Kind = CAMERA_CATCH_LOOP;
                    status_change_count = 0; // 重置计数器
                }else {
                    // CircleTrack_Step_OUT(Img_Store_p,Data_Path_p);
                }
            } else {
                status_change_count = 0;
                CircleTrack_Step_OUT(Img_Store_p,Data_Path_p);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}


void RunCircleTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    RunCameraCatchTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
    ProcessAlgo_CircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
}



void ProcessAlgo_AcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p->JSON_TrackConfigData_v[0];    // 赛道识别设置参数
    // cout << "AcrossTrack_Step: " << Data_Path_p->Across_Track_Step << endl;
    static int status_change_count = 0; // 状态变化计数器，用于监控状态持续时间

    
    switch(Data_Path_p->Across_Track_Step)
    {
        case ACROSS_PREPARE:
        {
            if(!Data_Path_p->black_left_found && !Data_Path_p->black_right_found){
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS;
                    status_change_count = 0; // 重置计数器
                }else{

                }
            }else {
                AcrossTrack_Step_ACROSS_PREPARE(Img_Store_p,Data_Path_p);   // 十字赛道预补线
                status_change_count = 0;
            }
            break;
        }
        case ACROSS:
        {
            if(Data_Path_p->InflectionPointNum[0] > 0 && Data_Path_p->InflectionPointNum[1] > 0){
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS_OUT;
                    status_change_count = 0; // 重置计数器
                    AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);   // 十字赛道出十字补线
                }else {
                    AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);   // 十字赛道内补线
                }
            }else {
                status_change_count = 0;
            }
            break;
        }
        case ACROSS_OUT:
        {
            if(Data_Path_p->l_border[image_h-JSON_TrackConfigData.Path_Search_Start] < 10 && Data_Path_p->r_border[image_h-JSON_TrackConfigData.Path_Search_Start] > 310){
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS_OUT_2;
                    status_change_count = 0; // 重置计数器
                    AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);
                }else {
                    AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);
                }
            }else {
                AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);
                status_change_count = 0;
            }
            break;
        }
        case ACROSS_OUT_2:
        {
            if(Data_Path_p->r_border[image_h-JSON_TrackConfigData.Path_Search_Start] - Data_Path_p->l_border[image_h-JSON_TrackConfigData.Path_Search_Start] < 250){
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = INIT_ACROSS;
                    Data_Path_p->Loop_Kind = CAMERA_CATCH_LOOP;
                    status_change_count = 0; // 重置计数器
                }else {
                    
                }
            }else {
                AcrossTrack_Step_ACROSS_OUT(Img_Store_p,Data_Path_p);
                status_change_count = 0;
            }
            break;
        }
        default:
        {
            break;
        }
    }

}

void RunAcrossTrackTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    RunCameraCatchTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
    ProcessAlgo_AcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p);
}


/**
 * @brief 处理每帧的赛道任务
 *
 * 根据当前循环类型调度不同的赛道处理任务。
 * 这里是主状态机的执行入口：
 * 1. CAMERA_CATCH_LOOP 负责采集后的基础图像处理
 * 2. JUDGE_LOOP 负责识别当前赛道并更新下一阶段状态
 * 3. COMMON_TRACK_LOOP 负责普通赛道的方向/速度计算
 * 4. L_CIRCLE_TRACK_LOOP / R_CIRCLE_TRACK_LOOP 负责圆环补线
 * 5. RIGHT_ACROSS_TRACK_LOOP 负责十字赛道处理
 *
 * @param Img_Store_p 图像存储指针
 * @param Data_Path_p 路径数据指针
 * @param Function_EN_p 功能使能状态指针
 * @param judge_p 判断器指针
 */
void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    if (Data_Path_p->Loop_Kind == CAMERA_CATCH_LOOP) {
        // 图像循环：负责采集后的基础图像处理和赛道状态更新。
        RunCameraCatchTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
        Data_Path_p->Loop_Kind = JUDGE_LOOP;
    }

    if (Data_Path_p->Loop_Kind == JUDGE_LOOP) {
        // 赛道判断循环：根据当前图像处理结果判断赛道类型，并切换到对应的赛道处理循环。
        // Data_Path_p->Track_Kind = STRIGHT_TRACK;
        // Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
        judge_p->TrackKind_Judge(Img_Store_p, Data_Path_p, Function_EN_p);
    }

    if (Data_Path_p->Loop_Kind == COMMON_TRACK_LOOP) {
        // 普通赛道循环：输出常规路径控制结果，并立即回到图像循环。
        Data_Path_p->Loop_Kind = CAMERA_CATCH_LOOP;
    }

    if (Data_Path_p->Loop_Kind == CIRCLE_TRACK_LOOP) {
        // 圆环循环：根据当前 Circle_Track_Step 执行对应补线策略。
        RunCircleTrackTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
    }

    if (Data_Path_p->Loop_Kind == ACROSS_TRACK_LOOP) {
        // 十字循环：执行十字赛道的特殊处理逻辑，然后回到图像循环。
        RunAcrossTrackTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
    }

}

/**
 * @brief 应用差速控制
 *
 * 根据视觉处理结果和功能使能状态，计算并设置电机的目标角速度和基础速度
 * 实现上位机控制模式下的差速控制
 *
 * @param Img_Store_p 图像存储指针
 * @param Data_Path_p 路径数据指针
 * @param Function_EN_p 功能使能状态指针
 * @param judge_p 判断器指针
 */
void ApplyDifferentialControl(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,Judge *judge_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    Data_Path_p->forword_line_h = std::max(image_h-JSON_TrackConfigData.Default_Forward, int(Data_Path_p->search_print_h_max));
    
    Data_Path_p->SteerErrorPx = (Data_Path_p->center_line[Data_Path_p->forword_line_h] - image_w / 2) 
                + JSON_TrackConfigData.ForwardHeightCompensationPxPerRow * (Data_Path_p->search_print_h_max-image_h-JSON_TrackConfigData.Default_Forward);
}

void FrameTaskAfterRead(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    if (!Function_EN_p->Game_EN)
    {
        return;
    }
    ProcessTrackTaskPerFrame(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);
    ApplyDifferentialControl(Img_Store_p,Data_Path_p,Function_EN_p,judge_p);
}
