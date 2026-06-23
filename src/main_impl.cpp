#include "main.hpp"
#include "camera_calibration.h"
#include "image_my_zf.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

void RunCameraCatchTask(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    // 同步元素识别使能标志到 my_zf
    if (!Function_EN_p->JSON_FunctionConfigData_v.empty()) {
        g_circle_identify_en = Function_EN_p->JSON_FunctionConfigData_v[0].CircleIdentify_EN;
        g_across_identify_en = Function_EN_p->JSON_FunctionConfigData_v[0].AcrossIdentify_EN;
    }

    imgProcess_p->imgPreProc(Img_Store_p,Data_Path_p,Function_EN_p);

    uint8_t off_line = ImageStatus.OFFLine;
    for (int row = off_line; row < 60 && row < image_h; row++) {
        Data_Path_p->l_border[row] = ImageDeal[row].LeftBorder;
        Data_Path_p->r_border[row] = ImageDeal[row].RightBorder;
        Data_Path_p->center_line[row] = ImageDeal[row].Center;
    }
    Data_Path_p->search_print_h_max = off_line;
    if (Data_Path_p->search_print_h_max > image_h - 1)
        Data_Path_p->search_print_h_max = image_h - 1;
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
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p->JSON_TrackConfigData_v[0];
    static int status_change_count = 0;
    static int frame_count = 0;

    int side = (Data_Path_p->Track_Kind == L_ACROSS_TRACK) ? 1 : 0;

    frame_count++;
    if (frame_count > JSON_TrackConfigData.AcrossMaxFrames) {
        Data_Path_p->Across_Track_Step = INIT_ACROSS;
        Data_Path_p->Loop_Kind = CAMERA_CATCH_LOOP;
        status_change_count = 0;
        frame_count = 0;
        return;
    }

    switch (Data_Path_p->Across_Track_Step)
    {
        case ACROSS_PREPARE:
        {
            if (Data_Path_p->BorderPointNum[side] < JSON_TrackConfigData.AcrossBorderPrepareMax) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS;
                    status_change_count = 0;
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        case ACROSS:
        {
            if (Data_Path_p->EdgeLineJumpNum[side] >= 1 && Data_Path_p->BorderPointNum[side] > 40) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS_OUT;
                    status_change_count = 0;
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        case ACROSS_OUT:
        {
            if (Data_Path_p->BorderPointNum[side] > JSON_TrackConfigData.AcrossBorderOutMin) {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = ACROSS_OUT_2;
                    status_change_count = 0;
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        case ACROSS_OUT_2:
        {
            if (Data_Path_p->BorderPointNum[0] < JSON_TrackConfigData.AcrossBorderExitMax ||
                Data_Path_p->BorderPointNum[1] < JSON_TrackConfigData.AcrossBorderExitMax)
            {
                status_change_count++;
                if (status_change_count > 2) {
                    Data_Path_p->Across_Track_Step = INIT_ACROSS;
                    Data_Path_p->Loop_Kind = CAMERA_CATCH_LOOP;
                    status_change_count = 0;
                    frame_count = 0;
                }
            } else {
                status_change_count = 0;
            }
            break;
        }
        default:
            break;
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
 * 使用 my_zf 图像处理管线完成全部视觉处理：
 * 1. my_zf 的 ImageProcess_my_zf() 内部已完成二值化、寻线、元素检测与处理。
 * 2. 此处仅将 my_zf 的 Road_type 映射到当前 TrackKind 枚举供显示用。
 * 3. 控制偏差由 my_zf 的 Det_True 提供，在 ApplyDifferentialControl 中使用。
 * 4. 圆环状态帧数限制：超过最大帧数自动清除圆环状态。
 * 5. 图像丢线帧数统计：供出赛道保护使用。
 *
 * @param Img_Store_p 图像存储指针
 * @param Data_Path_p 路径数据指针
 * @param Function_EN_p 功能使能状态指针
 * @param judge_p 判断器指针
 */
void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p,Data_Path *Data_Path_p,Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p)
{
    static const int RING_MAX_FRAMES = 300;
    static const int OFFLINE_ROW_THRESHOLD = 55;
    static int ring_frame_count = 0;
    static int offline_frame_count = 0;

    JSON_TrackConfigData cfg = Data_Path_p->JSON_TrackConfigData_v[0];

    RunCameraCatchTask(Img_Store_p,Data_Path_p,Function_EN_p,imgProcess_p,judge_p);

    // 圆环帧数限制
    if (ImageFlag.image_element_rings_flag != 0) {
        ring_frame_count++;
        if (ring_frame_count > cfg.CircleMaxFrames) {
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
            ring_frame_count = 0;
        }
    } else {
        ring_frame_count = 0;
    }

    // 丢线帧数统计
    if (ImageStatus.OFFLine >= OFFLINE_ROW_THRESHOLD) {
        offline_frame_count++;
    } else {
        offline_frame_count = 0;
    }

    // 将 my_zf 的 Road_type 映射到当前 TrackKind
    switch (ImageStatus.Road_type) {
        case LeftCirque:
            Data_Path_p->Track_Kind = L_CIRCLE_TRACK;
            Data_Path_p->Temp_Track_Kind = L_CIRCLE_TRACK;
            break;
        case RightCirque:
            Data_Path_p->Track_Kind = R_CIRCLE_TRACK;
            Data_Path_p->Temp_Track_Kind = R_CIRCLE_TRACK;
            break;
        case Cross:
        case Cross_ture:
            Data_Path_p->Track_Kind = L_ACROSS_TRACK;
            Data_Path_p->Temp_Track_Kind = L_ACROSS_TRACK;
            break;
        case Straight:
            Data_Path_p->Track_Kind = STRIGHT_TRACK;
            Data_Path_p->Temp_Track_Kind = STRIGHT_TRACK;
            break;
        default:
            Data_Path_p->Track_Kind = STRIGHT_TRACK;
            Data_Path_p->Temp_Track_Kind = STRIGHT_TRACK;
            break;
    }

    Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
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
    // 使用 my_zf 的加权偏差 Det_True（80宽度空间），减法归一化
    // Det_True 范围 [0, 79]，偏差 = Det_True - 40，main.cpp 中除以 40 得到 [-1, 1]
    Data_Path_p->SteerErrorPx = static_cast<int>(ImageStatus.Det_True) - 40;

    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    Data_Path_p->forword_line_h = std::max(image_h-JSON_TrackConfigData.Default_Forward, int(Data_Path_p->search_print_h_max));

    judge_p->MotorSpeed_Judge(Img_Store_p,Data_Path_p);
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
