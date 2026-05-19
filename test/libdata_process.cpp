#include "common_system.h"
#include "common_program.h"
#include "libdata_store.h"
#include "path_refactor.h"
#include <cerrno>
#include <cstring>

using namespace std;
using namespace cv;

namespace {
// 将当前左右边线拷贝为浮点点集，供角度法识别模块复用。
std::vector<cv::Point2f> collect_side_points(const Data_Path* data_path, bool left_side) {
    std::vector<cv::Point2f> points;
    if (data_path == nullptr) {
        return points;
    }

    const int num = left_side ? data_path->NumSearch[0] : data_path->NumSearch[1];
    if (num <= 0) {
        return points;
    }

    points.reserve(num);
    for (int i = 0; i < num; ++i) {
        const float x = static_cast<float>(left_side ? data_path->SideCoordinate_Eight[i][0] : data_path->SideCoordinate_Eight[i][2]);
        const float y = static_cast<float>(left_side ? data_path->SideCoordinate_Eight[i][1] : data_path->SideCoordinate_Eight[i][3]);
        points.emplace_back(x, y);
    }
    return points;
}
} // namespace

/*
    对八邻域寻找到的数据和独立黑色区域寻找到的数据进行分析
*/
void Judge::Search_Data_Analysis(Img_Store* Img_Store_p,Data_Path *Data_Path_p,Function_EN* Function_EN_p)
{
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    // 第一步：从当前帧提取拐点和弯点信息，作为状态机输入。
    Judge::InflectionPointSearch(Img_Store_p,Data_Path_p);
    Judge::BendPointSearch(Img_Store_p,Data_Path_p);
}
/*
    TrackKind_Judge说明
    赛道循环类型决策
    COMMON_TRACK_LOOP = 2,   // 普通赛道循环
    R_CIRCLE_TRACK_LOOP = 3,   // 右圆环赛道循环
    L_CIRCLE_TRACK_LOOP = 4,   // 左圆环赛道循环
    RIGHT_ACROSS_TRACK_LOOP = 5,   // 左十字赛道循环
    RIGHT_ACROSS_TRACK_LOOP = 6,   // 右十字赛道循环
*/
LoopKind Judge::TrackKind_Judge(Img_Store* Img_Store_p,Data_Path *Data_Path_p,Function_EN* Function_EN_p)
{
    LoopKind Loop_Kind;
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    // 这些静态量用来记录“最近一次进入某状态的帧号”，
    // 以便对十字、入环、出环等状态增加时间门限，减少误判后状态回跳的概率。
    static int State = 0;   // 当前帧号快照
    static int State_Across = 0;   // 最近一次进入十字状态的帧号
    static int State_Circle_IN_PREPARE = 0; // 最近一次进入准备入环状态的帧号
    static int State_Circle_IN = 0;    // 最近一次进入入环状态的帧号
    static int State_Circle_OUT_PREPARE = 0;    // 最近一次进入准备出环状态的帧号
    static int State_Circle_OUT = 0;    // 最近一次进入出环状态的帧号

    State = Img_Store_p -> ImgNum;

    if(Function_EN_p -> Control_EN == false)
    {
        // 第二步：十字判断。
        // 若左右边线都有拐点，且没有被圆环退出状态干扰，则先按十字赛道处理。
        if((Data_Path_p -> InflectionPointNum[0] >= 1) && (Data_Path_p -> InflectionPointNum[1] >= 1) && JSON_FunctionConfigData.AcrossIdentify_EN == true && Function_EN_p -> Gyroscope_EN == false && (Data_Path_p -> Circle_Track_Step != OUT_PREPARE || Data_Path_p -> Circle_Track_Step != OUT))
        {
            // 记录十字状态的进入时刻，避免接下来的圆环判定立即覆盖十字结论。
            State_Across = Img_Store_p -> ImgNum;
            Loop_Kind = LEFT_ACROSS_TRACK_LOOP;
            Data_Path_p -> Track_Kind = ACROSS_TRACK;
            Data_Path_p -> Circle_Track_Step = INIT;

            // 防止左右边线均寻找到同一个拐点导致误判为十字：
            // 如果左右拐点几乎重合，则按最近一次确认的圆环类型回退。
            if(abs((Data_Path_p -> InflectionPointCoordinate[0][0])-(Data_Path_p -> InflectionPointCoordinate[0][2])) <= 30)
            {
                switch(Data_Path_p -> Previous_Circle_Kind)
                {
                    case L_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = L_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = L_CIRCLE_TRACK_OUTSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                    case R_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = R_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = R_CIRCLE_TRACK_OUTSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                }
            }
        }
        // 第三步：圆环判断。
        // 圆环只允许在“单侧出现拐点 + 单侧出现弯点 + 与十字状态隔离一段帧数”时成立，
        // 这样可以避免十字、普通弯道与圆环之间的互相误判。
        else if((Data_Path_p -> InflectionPointNum[0] == 0) && (Data_Path_p -> InflectionPointNum[1] >= 1) && (Data_Path_p -> BendPointNum[0] <= 2) && (Data_Path_p -> BendPointNum[1] >= 1) && State - State_Across >= 5 && Function_EN_p -> Gyroscope_EN == false && JSON_FunctionConfigData.CircleIdentify_EN == true)
        {
            // 准备入环阶段：先锁定圆环方向，再等待后续帧确认是否真的进入环口。
            // 在出环后经过固定帧数才能再次准备进环，防止环口附近来回抖动。
            if(((Data_Path_p -> Circle_Track_Step) == INIT || (Data_Path_p -> Circle_Track_Step) == IN_PREPARE || (Data_Path_p -> Circle_Track_Step) == IN) && Data_Path_p -> Vector_Add_Unit_Dir[1] == 1)
            {
                Loop_Kind = R_CIRCLE_TRACK_LOOP;
                Data_Path_p -> Track_Kind = R_CIRCLE_TRACK_OUTSIDE;

                Data_Path_p -> Circle_Track_Step = IN_PREPARE;
                Data_Path_p -> Previous_Circle_Kind = R_CIRCLE_TRACK_OUTSIDE;

                // 记录准备进环时间，用于后续超时回退到 INIT。
                State_Circle_IN_PREPARE = Img_Store_p -> ImgNum;
            }
            // 入环阶段：当边线方向反转时，说明车辆已经真正进入环内，切换到持续补线模式。
            else if(Data_Path_p -> Vector_Add_Unit_Dir[1] == -1 && (Data_Path_p -> Circle_Track_Step == IN_PREPARE || Data_Path_p -> Circle_Track_Step == IN))   
            {
                Data_Path_p -> Circle_Track_Step = IN;

                // 以准备入环阶段确定的圆环类型作为入环阶段的圆环类型，
                // 避免在环口中间再次反向切换左右圆环。
                switch(Data_Path_p -> Previous_Circle_Kind)
                {
                    case L_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = L_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = L_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                    case R_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = R_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = R_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                }

                // 记录进环时间，供后续“进入出环准备”判断使用。
                State_Circle_IN = Img_Store_p -> ImgNum;
            }   
            // 考虑十字姿态不好，误判为圆环的情况：
            // 如果只是单次看到疑似特征，但状态尚未进入准备阶段，则直接回退普通赛道。
            else if(Data_Path_p -> Vector_Add_Unit_Dir[1] == -1 && Data_Path_p -> Circle_Track_Step == INIT)  
            {
                Loop_Kind = COMMON_TRACK_LOOP;
                Data_Path_p -> Track_Kind = BEND_TRACK;
            }
            else
            {
                // 其余情况均按普通弯道处理，避免错误进入圆环状态后持续补线。
                Loop_Kind = COMMON_TRACK_LOOP;
                Data_Path_p -> Track_Kind = BEND_TRACK;
            }
        }
        // 左侧圆环判断：与右侧圆环逻辑对称，只是拐点/弯点出现在另一侧。
        else if((Data_Path_p -> InflectionPointNum[0] >= 1) && (Data_Path_p -> InflectionPointNum[1] == 0) && (Data_Path_p -> BendPointNum[0] >= 1) && (Data_Path_p -> BendPointNum[1] <= 2) && State - State_Across >= 5 && Function_EN_p -> Gyroscope_EN == false && JSON_FunctionConfigData.CircleIdentify_EN == true)
        {
            // 准备入环阶段：先锁定圆环方向，再等待后续帧确认是否真的进入环口。
            // 在出环后经过固定帧数才能再次准备进环，防止环口附近来回抖动。
            if(((Data_Path_p -> Circle_Track_Step) == INIT || (Data_Path_p -> Circle_Track_Step) == IN_PREPARE || (Data_Path_p -> Circle_Track_Step) == IN) && Data_Path_p -> Vector_Add_Unit_Dir[0] == 1)
            {
                Loop_Kind = L_CIRCLE_TRACK_LOOP;
                Data_Path_p -> Track_Kind = L_CIRCLE_TRACK_OUTSIDE;

                Data_Path_p -> Circle_Track_Step = IN_PREPARE;
                Data_Path_p -> Previous_Circle_Kind = L_CIRCLE_TRACK_OUTSIDE;

                // 记录准备进环时间，用于后续超时回退到 INIT。
                State_Circle_IN_PREPARE = Img_Store_p -> ImgNum;
            }
            // 入环阶段：当边线方向反转时，说明车辆已经真正进入环内，切换到持续补线模式。
            else if(Data_Path_p -> Vector_Add_Unit_Dir[0] == -1 && (Data_Path_p -> Circle_Track_Step == IN_PREPARE || Data_Path_p -> Circle_Track_Step == IN))   
            {
                Data_Path_p -> Circle_Track_Step = IN;

                // 以准备入环阶段确定的圆环类型作为入环阶段的圆环类型，
                // 避免在环口中间再次反向切换左右圆环。
                switch(Data_Path_p -> Previous_Circle_Kind)
                {
                    case L_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = L_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = L_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                    case R_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = R_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = R_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = IN; break; }
                }
                
                // 记录进环时间，供后续“进入出环准备”判断使用。
                State_Circle_IN = Img_Store_p -> ImgNum;
            }   
            // 考虑十字姿态不好，误判为圆环的情况：
            // 如果只是单次看到疑似特征，但状态尚未进入准备阶段，则直接回退普通赛道。
            else if(Data_Path_p -> Vector_Add_Unit_Dir[0] == -1 && Data_Path_p -> Circle_Track_Step == INIT)  
            {
                Loop_Kind = COMMON_TRACK_LOOP;
                Data_Path_p -> Track_Kind = BEND_TRACK;
            }
            else
            {
                // 其余情况均按普通弯道处理，避免错误进入圆环状态后持续补线。
                Loop_Kind = COMMON_TRACK_LOOP;
                Data_Path_p -> Track_Kind = BEND_TRACK;
            }
        }
        // 第四步：出环判断。
        // 只有在陀螺仪允许的出环时间窗口内，才允许从圆环切回普通赛道。
        else if((Data_Path_p -> Circle_Track_Step == OUT_PREPARE || Data_Path_p -> Circle_Track_Step == OUT) && Function_EN_p -> Gyroscope_EN == true)
        {
            Data_Path_p -> Circle_Track_Step = OUT;

            // 出环阶段沿用入环阶段确认的圆环类型，避免在出环过程中切换左右环。
            switch(Data_Path_p -> Previous_Circle_Kind)
            {
                case L_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = L_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = L_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = OUT; break; }
                case R_CIRCLE_TRACK_OUTSIDE:{ Loop_Kind = R_CIRCLE_TRACK_LOOP; Data_Path_p -> Track_Kind = R_CIRCLE_TRACK_INSIDE; Data_Path_p -> Circle_Track_Step = OUT; break; }
            }

            // 记录出环时间，用于随后切换到 OUT_2_STRIGHT 再回到 INIT。
            State_Circle_OUT = Img_Store_p -> ImgNum;
        }
        // 第五步：普通赛道判断。
        // 当不满足任何特殊元素条件时，按普通直道/弯道处理，并同步维护圆环状态的超时回退。
        else
        {
            Loop_Kind = COMMON_TRACK_LOOP;
            // 普通赛道下先判断弯道还是直道，为后续电机速度选择提供依据。
            if((Data_Path_p -> BendPointNum[0] >= JSON_TrackConfigData.BendPointNum[0]) || (Data_Path_p -> BendPointNum[1] >= JSON_TrackConfigData.BendPointNum[0]))
            {
                Data_Path_p -> Track_Kind = BEND_TRACK;
            }
            else
            {
                Data_Path_p -> Track_Kind = STRIGHT_TRACK;
            }

            // 判定圆环步骤：下面这些条件只负责“状态回退/超时推进”，不改变当前赛道类型判断。

            // 进入圆环后固定帧数进入准备出环步骤，保证车辆有足够时间完成环内行驶。
            if(State - State_Circle_IN >= 10 && Data_Path_p -> Circle_Track_Step == IN)
            {
                Data_Path_p -> Circle_Track_Step = OUT_PREPARE;
                State_Circle_OUT_PREPARE = Img_Store_p -> ImgNum;
            }
            // 若误判为准备入环，则在固定帧数之后进入 INIT：
            // 防止在弯道、十字等位置误判导致一直处于补线相关状态。
            if(State - State_Circle_IN_PREPARE >= JSON_TrackConfigData.Circle_In_Prepare_Time && Data_Path_p -> Circle_Track_Step == IN_PREPARE)
            {
                Data_Path_p -> Circle_Track_Step = INIT;
            }
            // 出环后先进入“出环转直线”过渡态，为恢复普通寻线留出缓冲帧。
            if((Data_Path_p -> Circle_Track_Step) == OUT)
            {
                Data_Path_p -> Circle_Track_Step = OUT_2_STRIGHT;
                State_Circle_OUT = Img_Store_p -> ImgNum;
            }
            // 经过固定帧数后从“出环转直线”回到 INIT，允许下一次圆环重新判定。
            if((Data_Path_p -> Circle_Track_Step) == OUT_2_STRIGHT && State-State_Circle_OUT >= 60)
            {
                Data_Path_p -> Circle_Track_Step = INIT;
            }
            // 防止上次圆环未真正入环导致状态卡在准备出环阶段。
            // 若长时间没有完成出环，则回到 INIT，避免后续一直按圆环补线处理。
            if((Data_Path_p -> Circle_Track_Step) == OUT_PREPARE && State-State_Circle_OUT_PREPARE >= 200)
            {
                Data_Path_p -> Circle_Track_Step = INIT;
            }
        }
    }
    
    return Loop_Kind;
}

/*
    ServoDirAngle_Judge说明
    计算舵机方向和舵机角度
*/
void Judge::ServoDirAngle_Judge(Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    // 使用平滑前瞻替代单行前瞻，降低舵机角在噪声边线上的抖动。
    compute_smoothed_servo_control(Data_Path_p,
                                   image_w,
                                   JSON_TrackConfigData.Forward,
                                   image_h - 1,
                                   0);

    // 带符号转向误差：沿用历史方向定义，右转为正，左转为负。
    Data_Path_p->SteerErrorPx = (Data_Path_p->ServoDir == 1) ? Data_Path_p->ServoAngle : -Data_Path_p->ServoAngle;
}


/*
    MotorSpeed_Judge说明
    电机速度决策
*/
void Judge::MotorSpeed_Judge(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    /*
        由于圆环步骤和赛道类型是独立的，因此会出现如下情况
        1、直道但是圆环步骤不为占位，如准备入环到入环的阶段
        2、弯道但是圆环步骤不为占位，同上
        为防止速度过快时的拐点识别率下降，因此引入直道弯道时也要考虑圆环步骤防止误判
    */
    switch(Data_Path_p -> Track_Kind)
    {
        // 直道速度决策
        case STRIGHT_TRACK:
        {
            // 准备入环的直线部分的速度决策
            if(Data_Path_p -> ServoAngle > 30 || Data_Path_p -> Circle_Track_Step == IN_PREPARE)
            {
                Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[4];
            }
            // 准备出环、出环进直线的直线部分的速度决策
            else if(Data_Path_p -> ServoAngle > 30 || Data_Path_p -> Circle_Track_Step == OUT_PREPARE || Data_Path_p -> Circle_Track_Step == OUT_2_STRIGHT)
            {
                Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[5];
            }
            // 真正的直道的速度决策
            else
            {
                Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[0];
            }
            break;
        }
        case BEND_TRACK:
        {
            // 小弯道速度决策
            if(Data_Path_p -> BendPointNum[0] <= JSON_TrackConfigData.BendPointNum[1] || Data_Path_p -> BendPointNum[1] <= JSON_TrackConfigData.BendPointNum[1])
            {
                // 准备入环的小弯道部分的速度决策
                if(Data_Path_p -> Circle_Track_Step == IN_PREPARE)
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[4];
                }
                // 准备出环、出环进直线的小弯道部分的速度决策
                else if(Data_Path_p -> Circle_Track_Step == OUT_PREPARE || Data_Path_p -> Circle_Track_Step == OUT_2_STRIGHT)
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[5];
                }
                // 其他小弯道部分的速度决策
                else
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[1];
                }
            }
            // 大弯道速度决策
            else
            {
                // 准备入环的大弯道部分的速度决策
                if(Data_Path_p -> Circle_Track_Step == IN_PREPARE)
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[4];
                }
                // 准备出环、出环进直线的大弯道部分的速度决策
                else if(Data_Path_p -> Circle_Track_Step == OUT_PREPARE || Data_Path_p -> Circle_Track_Step == OUT_2_STRIGHT)
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[5];
                }
                // 其他大弯道部分的速度决策
                else
                {
                    Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[2];
                }
            }
            break;
        }
        case L_CIRCLE_TRACK_OUTSIDE:
        {
            Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[4];
            break;
        }
        case R_CIRCLE_TRACK_OUTSIDE:
        {
            Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[4];
            break;
        }
        case L_CIRCLE_TRACK_INSIDE:
        {
            Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[5];
            break;
        }
        case R_CIRCLE_TRACK_INSIDE:
        {
            Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[5];
            break;
        }
        case ACROSS_TRACK:
        {
            Data_Path_p -> MotorSpeed = JSON_TrackConfigData.CommonMotorSpeed[3];
            break;
        }
    }
}


/*
    AngularVelocityTarget_Judge说明
    将方向控制结果转换为目标角速度双轮差速控制目标
*/
void Judge::AngularVelocityTarget_Judge(Data_Path *Data_Path_p)
{
    // 经验参数：像素误差转角速度增益，可按车模响应继续微调。
    constexpr double kYawRatePerPixel = 1.2;     // deg/s per pixel
    constexpr double kMaxYawRateDeg = 220.0;     // deg/s
    constexpr double kBaseSpeedScale = 0.01;     // 旧速度档位(0-100)映射到 m/s
    constexpr double kMinBaseSpeedMps = 0.0;
    constexpr double kMaxBaseSpeedMps = 1.2;
    constexpr double kWheelbaseM = 0.158;

    const double target_yaw = std::clamp(
        static_cast<double>(Data_Path_p->SteerErrorPx) * kYawRatePerPixel,
        -kMaxYawRateDeg,
        kMaxYawRateDeg
    );

    const double base_speed = std::clamp(
        static_cast<double>(Data_Path_p->MotorSpeed) * kBaseSpeedScale,
        kMinBaseSpeedMps,
        kMaxBaseSpeedMps
    );

    const double yaw_rad = target_yaw * PI / 180.0;
    const double v_left = base_speed - yaw_rad * (kWheelbaseM * 0.5);
    const double v_right = base_speed + yaw_rad * (kWheelbaseM * 0.5);

    Data_Path_p->TargetAngularVelocityDeg = target_yaw;
    Data_Path_p->TargetBaseSpeedMps = base_speed;
    Data_Path_p->TargetLeftSpeedMps = v_left;
    Data_Path_p->TargetRightSpeedMps = v_right;
}


/*
    InflectionPointSearch说明
    边线拐点寻找
*/
void Judge::InflectionPointSearch(Img_Store* Img_Store_p,Data_Path *Data_Path_p)
{
    (void)Img_Store_p;
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    Data_Path_p -> InflectionPointNum[0] = 0;
    Data_Path_p -> InflectionPointNum[1] = 0;

    Data_Path_p -> Vector_Add_Unit_Dir[0] = 0;
    Data_Path_p -> Vector_Add_Unit_Dir[1] = 0;

    const int dist_step = std::max(1, JSON_TrackConfigData.InflectionPointVectorDistance);
    const std::vector<cv::Point2f> left_points = collect_side_points(Data_Path_p, true);
    const std::vector<cv::Point2f> right_points = collect_side_points(Data_Path_p, false);

    // 元素拐点识别：采用角度变化率 + 非极大值抑制。
    const PathRefactorFeatureResult left_result = detect_feature_by_angle(left_points,
                                                                          static_cast<float>(JSON_TrackConfigData.InflectionPointIdentifyAngle[0]),
                                                                          static_cast<float>(JSON_TrackConfigData.InflectionPointIdentifyAngle[1]),
                                                                          dist_step,
                                                                          10,
                                                                          20,
                                                                          true);
    const PathRefactorFeatureResult right_result = detect_feature_by_angle(right_points,
                                                                           static_cast<float>(JSON_TrackConfigData.InflectionPointIdentifyAngle[0]),
                                                                           static_cast<float>(JSON_TrackConfigData.InflectionPointIdentifyAngle[1]),
                                                                           dist_step,
                                                                           10,
                                                                           20,
                                                                           false);

    // 保留纵向趋势符号，供圆环准备入环判据使用。
    Data_Path_p->Vector_Add_Unit_Dir[0] = left_result.vertical_direction_sign;
    Data_Path_p->Vector_Add_Unit_Dir[1] = right_result.vertical_direction_sign;

    const int left_max = static_cast<int>(sizeof(Data_Path_p->InflectionPointCoordinate) / sizeof(Data_Path_p->InflectionPointCoordinate[0]));
    const int right_max = left_max;

    for (int k = 0; k < static_cast<int>(left_result.indices.size()) && Data_Path_p->InflectionPointNum[0] < left_max; ++k) {
        const int idx = left_result.indices[k];
        Data_Path_p->InflectionPointCoordinate[Data_Path_p->InflectionPointNum[0]][0] = static_cast<int>(left_points[idx].x);
        Data_Path_p->InflectionPointCoordinate[Data_Path_p->InflectionPointNum[0]][1] = static_cast<int>(left_points[idx].y);
        Data_Path_p->InflectionPointNum[0]++;
    }

    for (int k = 0; k < static_cast<int>(right_result.indices.size()) && Data_Path_p->InflectionPointNum[1] < right_max; ++k) {
        const int idx = right_result.indices[k];
        Data_Path_p->InflectionPointCoordinate[Data_Path_p->InflectionPointNum[1]][2] = static_cast<int>(right_points[idx].x);
        Data_Path_p->InflectionPointCoordinate[Data_Path_p->InflectionPointNum[1]][3] = static_cast<int>(right_points[idx].y);
        Data_Path_p->InflectionPointNum[1]++;
    }
}



/*
    BendPointSearch说明
    边线弯点寻找
*/
void Judge::BendPointSearch(Img_Store* Img_Store_p,Data_Path *Data_Path_p)
{
    (void)Img_Store_p;
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    Data_Path_p -> BendPointNum[0] = 0;
    Data_Path_p -> BendPointNum[1] = 0;

    const int dist_step = std::max(1, JSON_TrackConfigData.BendPointVectorDistance);
    const std::vector<cv::Point2f> left_points = collect_side_points(Data_Path_p, true);
    const std::vector<cv::Point2f> right_points = collect_side_points(Data_Path_p, false);

    // 边线弯点识别：与拐点共用算法，不同的是阈值取 BendPoint 配置。
    const PathRefactorFeatureResult left_result = detect_feature_by_angle(left_points,
                                                                          static_cast<float>(JSON_TrackConfigData.BendPointIdentifyAngle[0]),
                                                                          static_cast<float>(JSON_TrackConfigData.BendPointIdentifyAngle[1]),
                                                                          dist_step,
                                                                          10,
                                                                          30,
                                                                          true);
    const PathRefactorFeatureResult right_result = detect_feature_by_angle(right_points,
                                                                           static_cast<float>(JSON_TrackConfigData.BendPointIdentifyAngle[0]),
                                                                           static_cast<float>(JSON_TrackConfigData.BendPointIdentifyAngle[1]),
                                                                           dist_step,
                                                                           10,
                                                                           30,
                                                                           false);

    const int left_max = static_cast<int>(sizeof(Data_Path_p->BendPointCoordinate) / sizeof(Data_Path_p->BendPointCoordinate[0]));
    const int right_max = left_max;

    for (int k = 0; k < static_cast<int>(left_result.indices.size()) && Data_Path_p->BendPointNum[0] < left_max; ++k) {
        const int idx = left_result.indices[k];
        Data_Path_p->BendPointCoordinate[Data_Path_p->BendPointNum[0]][0] = static_cast<int>(left_points[idx].x);
        Data_Path_p->BendPointCoordinate[Data_Path_p->BendPointNum[0]][1] = static_cast<int>(left_points[idx].y);
        Data_Path_p->BendPointNum[0]++;
    }

    for (int k = 0; k < static_cast<int>(right_result.indices.size()) && Data_Path_p->BendPointNum[1] < right_max; ++k) {
        const int idx = right_result.indices[k];
        Data_Path_p->BendPointCoordinate[Data_Path_p->BendPointNum[1]][2] = static_cast<int>(right_points[idx].x);
        Data_Path_p->BendPointCoordinate[Data_Path_p->BendPointNum[1]][3] = static_cast<int>(right_points[idx].y);
        Data_Path_p->BendPointNum[1]++;
    }
}


/*
    HoughCircleSearch说明
    霍夫圆环识别
*/
void Judge::HoughCircleSearch(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    vector<Vec3f> Circle;
    HoughCircles((Img_Store_p -> Img_OTSU_Unpivot),Circle,HOUGH_GRADIENT,1,100,60,30,40,150);
    for(int i = 0;i < Circle.size();i++)
    {
        circle((Img_Store_p -> Img_Color_Unpivot),Point(Circle[i][0],Circle[i][1]),Circle[i][2],Scalar(255,0,255),2);
    }
}


/*
    Protect_Thread说明
    保护线程
    若检测到有ESC键输入则速度至0
*/
void Judge::Protect_Thread(Data_Path * Data_Path_p)
{
    bool Protect_EN = false;    // 保护使能
    int Stop = 0;
    while(Protect_EN == false)
    {
        cin >> Stop;
        if(Stop != 0)
        {
            Protect_EN = true;
        }
    }
    while(Protect_EN == true)
    {
        Data_Path_p -> MotorSpeed = 0;
    }
}

int jsonnum = 0; // 选择的json文件
bool changetimes = 0;

/*
    ConfigData_SYNC说明
    车辆上位机设置文件数据同步
*/
void SYNC::ConfigData_SYNC(Data_Path *Data_Path_p,Function_EN *Function_EN_p,JSON_PIDConfigData *JSON_PIDConfigData_p)
{
    if (Data_Path_p == nullptr || Function_EN_p == nullptr || JSON_PIDConfigData_p == nullptr)
    {
        std::cerr << "[Config] ConfigData_SYNC 参数非法: Data_Path_p=" << Data_Path_p
                  << ", Function_EN_p=" << Function_EN_p
                  << ", JSON_PIDConfigData_p=" << JSON_PIDConfigData_p << std::endl;
        return;
    }

    JSON_FunctionConfigData JSON_FunctionConfigData;
    JSON_TrackConfigData JSON_TrackConfigData;

    int JSON_FileNum;
    const char* ConfigFilePath;

    if (changetimes == 0){
        cout << "<---------------------JSON文件选择--------------------->" << endl;
        cout << "0.低速参数\n1.中速参数\n2.高速参数" << endl;
        cout << "参数选择：";
        cin >> JSON_FileNum;
        if (cin.fail())
        {
            cin.clear();
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cerr << "[Config] 输入不是有效数字，回退到 config_0.json" << std::endl;
            JSON_FileNum = 0;
        }
        changetimes = true;
        jsonnum = JSON_FileNum;
    }
    else{
        JSON_FileNum = jsonnum;
    }

    if (JSON_FileNum < 0 || JSON_FileNum > 2)
    {
        std::cerr << "[Config] 非法配置编号: " << JSON_FileNum << "，回退到 config_0.json" << std::endl;
        JSON_FileNum = 0;
        jsonnum = 0;
    }

    switch(JSON_FileNum)
    {
        case 0:{ ConfigFilePath = "config/config_0.json"; break; }
        case 1:{ ConfigFilePath = "config/config_1.json"; break; }
        case 2:{ ConfigFilePath = "config/config_2.json"; break; }
        default:{ ConfigFilePath = "config/config_0.json"; break; }
    }

    std::cout << "[Config] OPENING JSON FILE: " << ConfigFilePath << std::endl;
    ifstream ConfigFile(ConfigFilePath);
    if (!ConfigFile.is_open())
    {
        std::cerr << "[Config] 打开配置文件失败: " << ConfigFilePath
                  << ", errno=" << errno
                  << ", msg=" << std::strerror(errno) << std::endl;
        return;
    }

    ConfigFile.seekg(0, std::ios::end);
    std::streampos file_size = ConfigFile.tellg();
    ConfigFile.seekg(0, std::ios::beg);
    std::cout << "[Config] 文件大小: " << static_cast<long long>(file_size) << " bytes" << std::endl;
    if (file_size <= 0)
    {
        std::cerr << "[Config] 配置文件为空: " << ConfigFilePath << std::endl;
        return;
    }

    nlohmann::json ConfigData;
    try
    {
        ConfigData = nlohmann::json::parse(ConfigFile);
    }
    catch (const nlohmann::json::parse_error& e)
    {
        std::cerr << "[Config] JSON 解析失败: " << e.what() << std::endl;
        return;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Config] 读取配置异常: " << e.what() << std::endl;
        return;
    }

    if (!ConfigData.is_object())
    {
        std::cerr << "[Config] JSON 根节点不是 object，实际类型: " << ConfigData.type_name() << std::endl;
        return;
    }

    const std::vector<std::string> required_keys = {
        "SPEED_L", "SPEED_R", "UART_EN", "IMG_COMPRESS_EN", "CAMERA_EN", "IMAGE_SAVE_EN", "VIDEO_SHOW_EN",
        "DATA_PRINT_EN", "ACROSS_IDENTIFY_EN", "CIRCLE_IDENTIFY_EN", "FORWARD", "PATH_SEARCH_START",
        "PATH_SEARCH_END", "SIDE_SEARCH_START", "SIDE_SEARCH_END", "POINT_DISTANCE", "LITTLE_ANGLE_BEND_POINT_NUM",
        "BIG_ANGLE_BEND_POINT_NUM", "MIN_INFLECTION_POINT_ANGLE", "MAX_INFLECTION_POINT_ANGLE", "MIN_BEND_POINT_ANGLE",
        "MAX_BEND_POINT_ANGLE", "TRACK_WIDTH", "TRACK_KIND_COUNT_THRESHOLD", "CIRCLE_OUT_WIDTH", "STRIGHT_TRACK_MOTOR_SPEED",
        "LITTLE_ANGLE_BEND_TRACK_MOTOR_SPEED", "BIG_ANGLE_BEND_TRACK_MOTOR_SPEED", "ACROSS_TRACK_MOTOR_SPEED",
        "CIRCLE_TRACK_MOTOR_SPEED_OUTSIDE", "CIRCLE_TRACK_MOTOR_SPEED_INSIDE", "BRIDGE_ZONE_MOTOR_SPEED",
        "CROSSWALK_ZONE_MOTOR_SPEED_STOP_PREPARE", "CIRCLE_IN_PREPARE_TIME"
    };

    std::vector<std::string> missing_keys;
    missing_keys.reserve(required_keys.size());
    for (const auto& key : required_keys)
    {
        if (!ConfigData.contains(key))
        {
            missing_keys.push_back(key);
        }
    }

    if (!missing_keys.empty())
    {
        std::cerr << "[Config] JSON 缺少必需字段，缺失数量: " << missing_keys.size() << std::endl;
        for (const auto& key : missing_keys)
        {
            std::cerr << "[Config] missing key: " << key << std::endl;
        }
        return;
    }

    std::cout << "[Config] JSON 根节点字段数量: " << ConfigData.size() << std::endl;

    JSON_PIDConfigData_p->speedl = ConfigData.at("SPEED_L");    // 获取电机低速
    JSON_PIDConfigData_p->speedr = ConfigData.at("SPEED_R");    // 获取电机高速

    JSON_FunctionConfigData.Uart_EN = ConfigData.at("UART_EN");    // 获取串口使能参数
    JSON_FunctionConfigData.ImgCompress_EN = ConfigData.at("IMG_COMPRESS_EN");  // 获取图像压缩使能参数
    JSON_FunctionConfigData.Camera_EN = CameraKind(ConfigData.at("CAMERA_EN"));   // 获取摄像头使能参数
    JSON_FunctionConfigData.ImageSave_EN = ConfigData.at("IMAGE_SAVE_EN");  // 图像存储使能
    JSON_FunctionConfigData.VideoShow_EN = ConfigData.at("VIDEO_SHOW_EN"); // 获取图像显示使能参数
    JSON_FunctionConfigData.DataPrint_EN = ConfigData.at("DATA_PRINT_EN");  // 获取数据显示使能参数
    JSON_FunctionConfigData.AcrossIdentify_EN = ConfigData.at("ACROSS_IDENTIFY_EN");   // 获取十字识别使能参数
    JSON_FunctionConfigData.CircleIdentify_EN = ConfigData.at("CIRCLE_IDENTIFY_EN");   // 获取圆环识别使能参数

    JSON_TrackConfigData.Forward = ConfigData.at("FORWARD"); // 获取前瞻点
    JSON_TrackConfigData.Default_Forward = ConfigData.at("FORWARD"); // 获取默认前瞻点
    JSON_TrackConfigData.Path_Search_Start = ConfigData.at("PATH_SEARCH_START"); // 获取路径循线起始点
    JSON_TrackConfigData.Path_Search_End = ConfigData.at("PATH_SEARCH_END"); // 获取路径循线结束点
    JSON_TrackConfigData.Side_Search_Start = ConfigData.at("SIDE_SEARCH_START");    // 获取边线循线起始点
    JSON_TrackConfigData.Side_Search_End = ConfigData.at("SIDE_SEARCH_END");    // 获取边线循线结束点

    JSON_TrackConfigData.InflectionPointVectorDistance = ConfigData.at("POINT_DISTANCE");  // 获取元素拐点角度区
    JSON_TrackConfigData.BendPointVectorDistance = ConfigData.at("POINT_DISTANCE");  // 获取边线弯点角度区
    JSON_TrackConfigData.BendPointNum[0] = ConfigData.at("LITTLE_ANGLE_BEND_POINT_NUM");    // 小角度弯道 弯点数量
    JSON_TrackConfigData.BendPointNum[1] = ConfigData.at("BIG_ANGLE_BEND_POINT_NUM");       // 大角度弯道 弯点数量
    JSON_TrackConfigData.InflectionPointIdentifyAngle[0] = ConfigData.at("MIN_INFLECTION_POINT_ANGLE");  // 获取元素拐点角度区间
    JSON_TrackConfigData.InflectionPointIdentifyAngle[1] = ConfigData.at("MAX_INFLECTION_POINT_ANGLE"); 
    JSON_TrackConfigData.BendPointIdentifyAngle[0] = ConfigData.at("MIN_BEND_POINT_ANGLE");  // 获取边线弯点角度区间
    JSON_TrackConfigData.BendPointIdentifyAngle[1] = ConfigData.at("MAX_BEND_POINT_ANGLE"); 

    JSON_TrackConfigData.TrackWidth = ConfigData.at("TRACK_WIDTH");   // 获取赛道宽度参数
    JSON_TrackConfigData.CircleOutWidth = ConfigData.at("CIRCLE_OUT_WIDTH");    // 获取圆环出环补线时终点离中线距离

    JSON_TrackConfigData.CommonMotorSpeed[0] = ConfigData.at("STRIGHT_TRACK_MOTOR_SPEED");  // 获取直道电机速度
    JSON_TrackConfigData.CommonMotorSpeed[1] = ConfigData.at("LITTLE_ANGLE_BEND_TRACK_MOTOR_SPEED"); // 小角度弯道电机速度
    JSON_TrackConfigData.CommonMotorSpeed[2] = ConfigData.at("BIG_ANGLE_BEND_TRACK_MOTOR_SPEED"); // 大角度弯道电机速度
    JSON_TrackConfigData.CommonMotorSpeed[3] = ConfigData.at("ACROSS_TRACK_MOTOR_SPEED"); // 十字赛道电机速度
    JSON_TrackConfigData.CommonMotorSpeed[4] = ConfigData.at("CIRCLE_TRACK_MOTOR_SPEED_OUTSIDE"); // 圆环外赛道电机速度
    JSON_TrackConfigData.CommonMotorSpeed[5] = ConfigData.at("CIRCLE_TRACK_MOTOR_SPEED_INSIDE"); // 圆环内赛道电机速度
    JSON_TrackConfigData.BridgeZoneMotorSpeed = ConfigData.at("BRIDGE_ZONE_MOTOR_SPEED"); // 桥梁区域电机速度
    JSON_TrackConfigData.CrosswalkZoneMotorSpeed = ConfigData.at("CROSSWALK_ZONE_MOTOR_SPEED_STOP_PREPARE"); // 斑马线区域准备停车电机速度
    JSON_TrackConfigData.Circle_In_Prepare_Time = ConfigData.at("CIRCLE_IN_PREPARE_TIME");  // 准备入环限定时间

    if (ConfigData.contains("TRANSITION_MIN_AREA")) {
        JSON_TrackConfigData.TransitionMinArea = ConfigData.at("TRANSITION_MIN_AREA");
    }

    // 同步配置到运行时容器（覆盖旧值，保持单配置生效）。
    Function_EN_p->JSON_FunctionConfigData_v.clear();
    Data_Path_p->JSON_TrackConfigData_v.clear();
    Function_EN_p->JSON_FunctionConfigData_v.push_back(JSON_FunctionConfigData);
    Data_Path_p->JSON_TrackConfigData_v.push_back(JSON_TrackConfigData);

    std::cout << "[Config] Function 配置数量: " << Function_EN_p->JSON_FunctionConfigData_v.size()
              << ", Track 配置数量: " << Data_Path_p->JSON_TrackConfigData_v.size() << std::endl;
    std::cout << "[Config] Camera_EN=" << static_cast<int>(JSON_FunctionConfigData.Camera_EN)
              << ", Forward=" << JSON_TrackConfigData.Forward
              << ", PathSearch=[" << JSON_TrackConfigData.Path_Search_Start
              << ", " << JSON_TrackConfigData.Path_Search_End << "]" << std::endl;

    cout << "<---------------------JSON参数获取成功--------------------->" << endl;
}


/*
    DataPrint说明
    打印数据
    程序参数：1.前瞻点 2.寻边线起始点 3.寻边线结束点 4.边线断点起始点 5.边线断点结束点 6.比赛状态
    运动参数：1.舵机方向 2.舵机角度 3.点击速度
*/
void DataPrint(Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    

    if(JSON_FunctionConfigData.DataPrint_EN == true)
    {
        cout << "\033c";    // 每次打印前清屏
        if(JSON_FunctionConfigData.Uart_EN == true)
        {
            cout << "<---------------------比赛模式--------------------->" << endl;
            cout << endl;
        }
        else
        {
            cout << "<---------------------调试模式--------------------->" << endl;
            cout << endl;
        }

        // 打印程序参数
        cout << "<---------------------程序参数--------------------->" << endl;
        cout << " 前瞻点：" << JSON_TrackConfigData.Forward << endl;
        cout << " 路径线起始点：" << JSON_TrackConfigData.Path_Search_Start << endl; 
        cout << " 路径线结束点：" << JSON_TrackConfigData.Path_Search_End << endl; 
        cout << " 边线起始点：" << JSON_TrackConfigData.Side_Search_Start << endl; 
        cout << " 边线结束点：" << JSON_TrackConfigData.Side_Search_End << endl; 
        cout << " 比赛状态：";
        switch(Function_EN_p -> Game_EN)
        {
            case true:{ cout << "开始" << endl; break; }
            case false:{ cout << "结束" << endl; break; }
        }
        cout << "<-------------------------------------------------->" << endl;
        cout << endl;

        // 打印运动参数
        cout << "<---------------------运动参数--------------------->" << endl;
        cout << "舵机方向：";
        cout << Data_Path_p -> ServoDir << endl;
        cout << "舵机角度：";
        cout << Data_Path_p -> ServoAngle << endl;
        cout <<  "电机速度：";
        cout << Data_Path_p -> MotorSpeed << endl;
        cout <<  "赛道类型：";
        switch(Data_Path_p -> Track_Kind)
        {
            case STRIGHT_TRACK:{ cout << "直赛道" << endl; break; }
            case BEND_TRACK:{ cout << "弯赛道" << endl; break; }
            case R_CIRCLE_TRACK_OUTSIDE:
            { 
                cout << "右圆环赛道：准备入环" << endl; 
                break;
            }
            case L_CIRCLE_TRACK_OUTSIDE:
            { 
                cout << "左圆环赛道：准备入环" << endl; 
                break;
            }
            case R_CIRCLE_TRACK_INSIDE:
            { 
                cout << "右圆环赛道："; 
                switch(Data_Path_p -> Circle_Track_Step)
                {
                    case IN:{ cout << "入环" << endl; break; }
                    case OUT_PREPARE:{ cout << "准备出环" << endl; break; }
                    case OUT:{ cout << "出环" << endl; break; }
                    case INIT:{ cout << "初始化" << endl; break; }
                }
                break;
            }
            case L_CIRCLE_TRACK_INSIDE:
            { 
                cout << "左圆环赛道："; 
                switch(Data_Path_p -> Circle_Track_Step)
                {
                    case IN:{ cout << "入环" << endl; break; }
                    case OUT_PREPARE:{ cout << "准备出环" << endl; break; }
                    case OUT:{ cout << "出环" << endl; break; }
                    case INIT:{ cout << "初始化" << endl; break; }
                }
                break;
            }
            case ACROSS_TRACK:{ cout << "十字赛道" << endl; break; }
        }
        cout <<  "控制使能：";
        switch(Function_EN_p -> Control_EN)
        {
            case true:{ cout << "下位机控制" << endl; break; }
            case false:{ cout << "上位机控制" << endl; break; }
        }
        cout << "<-------------------------------------------------->" << endl;
    }
}


