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

void push_track_kind_history(Data_Path* data_path, TrackKind kind) {
    if (data_path == nullptr) {
        return;
    }

    const int kind_value = static_cast<int>(kind);
    if (kind_value < STRIGHT_TRACK || kind_value > R_CIRCLE_TRACK) {
        return;
    }

    const int size = Data_Path::kTrackKindHistorySize;
    data_path->TrackKindHistory[data_path->TrackKindHistoryIndex] = kind;
    data_path->TrackKindHistoryIndex = (data_path->TrackKindHistoryIndex + 1) % size;
    if (data_path->TrackKindHistoryCount < size) {
        data_path->TrackKindHistoryCount++;
    }
}
} // namespace


/*
    对八邻域寻找到的数据和独立黑色区域寻找到的数据进行分析
*/
void Judge::Search_Data_Analysis(Img_Store* Img_Store_p,Data_Path *Data_Path_p,Function_EN* Function_EN_p)
{
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    Judge::InflectionPointSearch(Img_Store_p,Data_Path_p);
    Judge::BendPointSearch(Img_Store_p,Data_Path_p);

}


/*
    TrackKind_Judge说明
    赛道循环类型决策
*/
LoopKind Judge::TrackKind_Judge(Img_Store* Img_Store_p, Data_Path *Data_Path_p, Function_EN* Function_EN_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p->JSON_TrackConfigData_v[0];

    static int TrackKindScore[6] = {0};

    int jump_l = Data_Path_p->EdgeLineJumpNum[0];
    int jump_r = Data_Path_p->EdgeLineJumpNum[1];
    int border_l = Data_Path_p->BorderPointNum[0];
    int border_r = Data_Path_p->BorderPointNum[1];
    int inflection_l = Data_Path_p->InflectionPointNum[0];
    int inflection_r = Data_Path_p->InflectionPointNum[1];

    int circle_jump_max = JSON_TrackConfigData.CircleJumpMax;                        // 圆环判定：安静侧跳变点数量上限
    int circle_border_quiet_max = JSON_TrackConfigData.CircleBorderQuietMax;          // 圆环判定：安静侧边缘区域点数量上限
    int circle_jump_expected = JSON_TrackConfigData.CircleJumpExpected;               // 圆环判定：活跃侧期望跳变点数量
    int circle_border_active_min = JSON_TrackConfigData.CircleBorderActiveMin;        // 圆环判定：活跃侧边缘区域点数量下限
    int circle_inflection_max = JSON_TrackConfigData.CircleJudgeInflectionMax;        // 圆环判定：安静侧拐点数量上限
    int circle_partial_score = JSON_TrackConfigData.CircleJudgePartialScore;          // 圆环判定：部分条件满足每帧得分
    int cross_jump_primary = JSON_TrackConfigData.CrossJumpPrimary;                   // 十字判定：主侧期望跳变点数量
    int cross_jump_sec_min = JSON_TrackConfigData.CrossJumpSecondaryMin;              // 十字判定：副侧最小跳变点数量
    int cross_jump_sec_max = JSON_TrackConfigData.CrossJumpSecondaryMax;              // 十字判定：副侧最大跳变点数量
    int cross_border_min = JSON_TrackConfigData.CrossBorderMin;                       // 十字判定：两侧最小边缘区域点数量
    int full_score = JSON_TrackConfigData.TrackJudgeFullScore;                        // 帧累积评分：全部条件满足每帧得分
    int cross_partial_score = JSON_TrackConfigData.TrackJudgePartialScore;            // 帧累积评分：十字3/4条件满足每帧得分
    int confirm_threshold = JSON_TrackConfigData.TrackJudgeConfirmThreshold;          // 帧累积评分：确认阈值
    int count_threshold = JSON_TrackConfigData.TrackKindCountThreshold;               // 普通赛道类型计数阈值

    // 圆环必要条件（门控）：左右边界点 + 安静侧拐点 + 活跃侧必须有跳变，必须全部满足
    bool rc_gate_ok = (border_l < circle_border_quiet_max) && (border_r > circle_border_active_min) && (inflection_l <= circle_inflection_max) && (jump_r > 0);
    bool lc_gate_ok = (border_r < circle_border_quiet_max) && (border_l > circle_border_active_min) && (inflection_r <= circle_inflection_max) && (jump_l > 0);

    int rc_match = 0;  // 右圆环计分条件计数
    if (rc_gate_ok) {
        if (jump_l <= circle_jump_max) rc_match++;              // 左侧安静：跳变点≤上限
        if (jump_r == circle_jump_expected) rc_match++;         // 右侧活跃：跳变点=期望值
    }

    int lc_match = 0;  // 左圆环计分条件计数
    if (lc_gate_ok) {
        if (jump_r <= circle_jump_max) lc_match++;              // 右侧安静：跳变点≤上限
        if (jump_l == circle_jump_expected) lc_match++;         // 左侧活跃：跳变点=期望值
    }

    int la_match = 0;  // 左十字条件计数
    if (jump_l == cross_jump_primary) la_match++;               // 左侧跳变点=主侧期望值
    if (border_l >= cross_border_min) la_match++;               // 左侧边缘区域点≥下限
    if (jump_r >= cross_jump_sec_min && jump_r <= cross_jump_sec_max) la_match++;  // 右侧跳变点在副侧范围内
    if (border_r >= cross_border_min) la_match++;               // 右侧边缘区域点≥下限

    int ra_match = 0;  // 右十字条件计数
    if (jump_r == cross_jump_primary) ra_match++;               // 右侧跳变点=主侧期望值
    if (border_r >= cross_border_min) ra_match++;               // 右侧边缘区域点≥下限
    if (jump_l >= cross_jump_sec_min && jump_l <= cross_jump_sec_max) ra_match++;  // 左侧跳变点在副侧范围内
    if (border_l >= cross_border_min) ra_match++;               // 左侧边缘区域点≥下限

    int match_count = 0;     // 当前帧匹配条件数
    bool is_circle = false;  // 是否为圆环类型
    TrackKind detected_kind = STRIGHT_TRACK;  // 当前帧检测结果

    if (rc_match >= 1) {               // 右圆环：至少满足1个计分条件（共2个）
        detected_kind = R_CIRCLE_TRACK;
        match_count = rc_match;
        is_circle = true;
    } else if (lc_match >= 1) {        // 左圆环：至少满足1个计分条件（共2个）
        detected_kind = L_CIRCLE_TRACK;
        match_count = lc_match;
        is_circle = true;
    } else if (la_match >= 3) {        // 左十字：至少满足3个条件（共4个）
        detected_kind = L_ACROSS_TRACK;
        match_count = la_match;
    } else if (ra_match >= 3) {        // 右十字：至少满足3个条件（共4个）
        detected_kind = R_ACROSS_TRACK;
        match_count = ra_match;
    }

    if (detected_kind != STRIGHT_TRACK) {
        int add_score;  // 当前帧加分
        if (is_circle) {
            add_score = (match_count == 2) ? full_score : circle_partial_score;   // 2/2满分，1/2部分分
        } else {
            add_score = (match_count == 4) ? full_score : cross_partial_score;    // 4/4满分，3/4部分分
        }
        TrackKindScore[detected_kind] += add_score;

        for (int i = 0; i < 6; ++i) {
            if (i != detected_kind) {
                TrackKindScore[i] = 0;
            }
        }

        if (TrackKindScore[detected_kind] >= confirm_threshold) {
            Data_Path_p->Track_Kind = detected_kind;
            Data_Path_p->Temp_Track_Kind = detected_kind;
            if (detected_kind == R_CIRCLE_TRACK || detected_kind == L_CIRCLE_TRACK) {
                Data_Path_p->Loop_Kind = CIRCLE_TRACK_LOOP;
                Data_Path_p->Circle_Track_Step = IN_PREPARE;
            } else {
                Data_Path_p->Loop_Kind = ACROSS_TRACK_LOOP;
                Data_Path_p->Across_Track_Step = ACROSS_PREPARE;
            }
            for (int i = 0; i < 6; ++i) {
                TrackKindScore[i] = 0;
            }
        } else {
            Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
        }
    } else {
        TrackKindScore[L_ACROSS_TRACK] = 0;
        TrackKindScore[R_ACROSS_TRACK] = 0;
        TrackKindScore[L_CIRCLE_TRACK] = 0;
        TrackKindScore[R_CIRCLE_TRACK] = 0;

        bool is_bend = (Data_Path_p->BendPointNum[0] > JSON_TrackConfigData.BendPointNum[0]
            && Data_Path_p->BendPointNum[0] < JSON_TrackConfigData.BendPointNum[1]
            && Data_Path_p->BendPointNum[1] > JSON_TrackConfigData.BendPointNum[0]
            && Data_Path_p->BendPointNum[1] < JSON_TrackConfigData.BendPointNum[1]);

        if (is_bend) {
            Data_Path_p->Temp_Track_Kind = BEND_TRACK;
            TrackKindScore[BEND_TRACK]++;
            if (TrackKindScore[BEND_TRACK] > count_threshold) {
                Data_Path_p->Track_Kind = BEND_TRACK;
                Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
                for (int i = 0; i < 6; ++i) {
                    if (i != BEND_TRACK) TrackKindScore[i] = 0;
                }
            } else {
                Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
            }
        } else {
            Data_Path_p->Temp_Track_Kind = STRIGHT_TRACK;
            TrackKindScore[STRIGHT_TRACK]++;
            if (TrackKindScore[STRIGHT_TRACK] > count_threshold) {
                Data_Path_p->Track_Kind = STRIGHT_TRACK;
                Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
                for (int i = 0; i < 6; ++i) {
                    if (i != STRIGHT_TRACK) TrackKindScore[i] = 0;
                }
            } else {
                Data_Path_p->Loop_Kind = COMMON_TRACK_LOOP;
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        Data_Path_p->TrackJudgeScore[i] = TrackKindScore[i];
    }
    return Data_Path_p->Loop_Kind;
}

/*
    MotorSpeed_Judge说明
    电机速度决策
*/
void Judge::MotorSpeed_Judge(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    Data_Path_p->TargetBaseSpeedMps = JSON_TrackConfigData.CommonMotorSpeed[0];
    /*
        由于圆环步骤和赛道类型是独立的，因此会出现如下情况
        1、直道但是圆环步骤不为占位，如准备入环到入环的阶段
        2、弯道但是圆环步骤不为占位，同上
        为防止速度过快时的拐点识别率下降，因此引入直道弯道时也要考虑圆环步骤防止误判
    */
    // switch(Data_Path_p -> Track_Kind)
    // {
    //     // 直道速度决策
    //     case STRIGHT_TRACK:
    //     {
    //         Data_Path_p->TargetBaseSpeedMps = JSON_TrackConfigData.CommonMotorSpeed[0];
    //     }
    //     case BEND_TRACK:
    //     {
    //         Data_Path_p->TargetBaseSpeedMps = JSON_TrackConfigData.CommonMotorSpeed[1];
    //     }
    // }
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
    for(size_t i = 0;i < Circle.size();i++)
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
        Data_Path_p -> TargetBaseSpeedMps = 0;
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
    JSON_DifferentialPDConfigData JSON_DifferentialPDConfigData;
    JSON_AngularVelocityPIDConfigData JSON_AngularVelocityPIDConfigData;
    JSON_SpeedIncrementalPIConfigData JSON_SpeedIncrementalPIConfigData;
    JSON_VehicleConfigData JSON_VehicleConfigData;

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
        "UART_EN", "IMG_COMPRESS_EN", "CAMERA_EN", "IMAGE_SAVE_EN", "VIDEO_SHOW_EN",
        "DATA_PRINT_EN", "ACROSS_IDENTIFY_EN", "CIRCLE_IDENTIFY_EN","Track_width", "FORWARD", "FORWARD_HEIGHT_COMPENSATION_PX_PER_ROW", "PATH_SEARCH_START",
        "PATH_SEARCH_END", "SIDE_SEARCH_START", "SIDE_SEARCH_END", "POINT_DISTANCE", "LITTLE_ANGLE_BEND_POINT_NUM",
        "BIG_ANGLE_BEND_POINT_NUM", "MIN_INFLECTION_POINT_ANGLE", "MAX_INFLECTION_POINT_ANGLE", "MIN_BEND_POINT_ANGLE",
        "MAX_BEND_POINT_ANGLE", "TRACK_WIDTH", "TRACK_KIND_COUNT_THRESHOLD","CIRCLE_OUT_WIDTH", "STRIGHT_TRACK_MOTOR_SPEED",
        "LITTLE_ANGLE_BEND_TRACK_MOTOR_SPEED", "BIG_ANGLE_BEND_TRACK_MOTOR_SPEED", "ACROSS_TRACK_MOTOR_SPEED",
        "CIRCLE_TRACK_MOTOR_SPEED_OUTSIDE", "CIRCLE_TRACK_MOTOR_SPEED_INSIDE", "BRIDGE_ZONE_MOTOR_SPEED",
        "CROSSWALK_ZONE_MOTOR_SPEED_STOP_PREPARE", "CIRCLE_IN_PREPARE_TIME",
        "DIFF_OUTER_PD_KP", "DIFF_OUTER_PD_KD", "DIFF_OUTER_PD_LIMIT_P", "DIFF_OUTER_PD_LIMIT_D",
        "DIFF_OUTER_PD_LIMIT_OUTPUT",
        "DIFF_INNER_PI_KP", "DIFF_INNER_PI_KI", "DIFF_INNER_PI_KD", "DIFF_INNER_PI_LIMIT_P", "DIFF_INNER_PI_LIMIT_I",
        "DIFF_INNER_PI_LIMIT_D", "DIFF_INNER_PI_LIMIT_OUTPUT", "DIFF_INNER_PI_LIMIT_I_MIN", "DIFF_INNER_PI_ANTI_WINDUP",
        "SPEED_INCR_PI_KP", "SPEED_INCR_PI_KI", "SPEED_INCR_PI_KD", "SPEED_INCR_PI_LIMIT_OUTPUT",
        "CONTROL_PERIOD", "MOTOR_MAX_DUTY",
        "LPF_SPEED_TAU", "LPF_ANGULAR_TAU",
        "MOTOR_PWM_DEAD_ZONE_LEFT", "MOTOR_PWM_DEAD_ZONE_RIGHT",
        "COLLISION_PROTECT_ENABLE", "COLLISION_IMU_JERK_THRESHOLD",
        "COLLISION_STALL_DUTY_THRESHOLD", "COLLISION_STALL_SPEED_THRESHOLD",
        "COLLISION_STALL_CYCLES", "COLLISION_RESET_KEY", "COLLISION_BUMPER_KEY",
        "ACROSS_BORDER_EXIT_MAX",
        "ACROSS_MAX_FRAMES"
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

    // 外环差速PD参数
    JSON_DifferentialPDConfigData.Kp = ConfigData.at("DIFF_OUTER_PD_KP");
    JSON_DifferentialPDConfigData.Kd = ConfigData.at("DIFF_OUTER_PD_KD");
    JSON_DifferentialPDConfigData.limitP = ConfigData.at("DIFF_OUTER_PD_LIMIT_P");
    JSON_DifferentialPDConfigData.limitD = ConfigData.at("DIFF_OUTER_PD_LIMIT_D");
    JSON_DifferentialPDConfigData.limitOutput = ConfigData.at("DIFF_OUTER_PD_LIMIT_OUTPUT");

    // 内环角速度PI参数
    JSON_AngularVelocityPIDConfigData.Kp = ConfigData.at("DIFF_INNER_PI_KP");
    JSON_AngularVelocityPIDConfigData.Ki = ConfigData.at("DIFF_INNER_PI_KI");
    JSON_AngularVelocityPIDConfigData.Kd = ConfigData.at("DIFF_INNER_PI_KD");
    JSON_AngularVelocityPIDConfigData.limitP = ConfigData.at("DIFF_INNER_PI_LIMIT_P");
    JSON_AngularVelocityPIDConfigData.limitI = ConfigData.at("DIFF_INNER_PI_LIMIT_I");
    JSON_AngularVelocityPIDConfigData.limitD = ConfigData.at("DIFF_INNER_PI_LIMIT_D");
    JSON_AngularVelocityPIDConfigData.limitOutput = ConfigData.at("DIFF_INNER_PI_LIMIT_OUTPUT");
    JSON_AngularVelocityPIDConfigData.limitIMin = ConfigData.at("DIFF_INNER_PI_LIMIT_I_MIN");
    JSON_AngularVelocityPIDConfigData.enableAntiWindup = ConfigData.at("DIFF_INNER_PI_ANTI_WINDUP");

    // 速度环增量式PID参数
    JSON_SpeedIncrementalPIConfigData.Kp = ConfigData.at("SPEED_INCR_PI_KP");
    JSON_SpeedIncrementalPIConfigData.Ki = ConfigData.at("SPEED_INCR_PI_KI");
    JSON_SpeedIncrementalPIConfigData.Kd = ConfigData.at("SPEED_INCR_PI_KD");
    JSON_SpeedIncrementalPIConfigData.limitOutput = ConfigData.at("SPEED_INCR_PI_LIMIT_OUTPUT");

    // 车辆控制参数
    JSON_VehicleConfigData.controlPeriod = ConfigData.at("CONTROL_PERIOD");
    JSON_VehicleConfigData.motorMaxDuty = ConfigData.at("MOTOR_MAX_DUTY");

    // 低通滤波参数
    JSON_VehicleConfigData.lpfSpeedTau = ConfigData.at("LPF_SPEED_TAU");
    JSON_VehicleConfigData.lpfAngularTau = ConfigData.at("LPF_ANGULAR_TAU");

    // 电机死区参数
    JSON_VehicleConfigData.motorPwmDeadZoneLeft = ConfigData.at("MOTOR_PWM_DEAD_ZONE_LEFT");
    JSON_VehicleConfigData.motorPwmDeadZoneRight = ConfigData.at("MOTOR_PWM_DEAD_ZONE_RIGHT");

    // 碰撞保护参数
    JSON_VehicleConfigData.collisionProtectEnable = ConfigData.at("COLLISION_PROTECT_ENABLE");
    JSON_VehicleConfigData.collisionImuJerkThreshold = ConfigData.at("COLLISION_IMU_JERK_THRESHOLD");
    JSON_VehicleConfigData.collisionStallDutyThreshold = ConfigData.at("COLLISION_STALL_DUTY_THRESHOLD");
    JSON_VehicleConfigData.collisionStallSpeedThreshold = ConfigData.at("COLLISION_STALL_SPEED_THRESHOLD");
    JSON_VehicleConfigData.collisionStallCycles = ConfigData.at("COLLISION_STALL_CYCLES");
    JSON_VehicleConfigData.collisionResetKey = ConfigData.at("COLLISION_RESET_KEY");
    JSON_VehicleConfigData.collisionBumperKey = ConfigData.at("COLLISION_BUMPER_KEY");

    JSON_FunctionConfigData.Uart_EN = ConfigData.at("UART_EN");    // 获取串口使能参数
    JSON_FunctionConfigData.ImgCompress_EN = ConfigData.at("IMG_COMPRESS_EN");  // 获取图像压缩使能参数
    JSON_FunctionConfigData.Camera_EN = CameraKind(ConfigData.at("CAMERA_EN"));   // 获取摄像头使能参数
    JSON_FunctionConfigData.ImageSave_EN = ConfigData.at("IMAGE_SAVE_EN");  // 图像存储使能
    JSON_FunctionConfigData.VideoShow_EN = ConfigData.at("VIDEO_SHOW_EN"); // 获取图像显示使能参数
    JSON_FunctionConfigData.DataPrint_EN = ConfigData.at("DATA_PRINT_EN");  // 获取数据显示使能参数
    JSON_FunctionConfigData.AcrossIdentify_EN = ConfigData.at("ACROSS_IDENTIFY_EN");   // 获取十字识别使能参数
    JSON_FunctionConfigData.CircleIdentify_EN = ConfigData.at("CIRCLE_IDENTIFY_EN");   // 获取圆环识别使能参数

    JSON_TrackConfigData.Track_width = ConfigData.at("Track_width");

    JSON_TrackConfigData.Forward = ConfigData.at("FORWARD"); // 获取前瞻点
    JSON_TrackConfigData.Default_Forward = ConfigData.at("FORWARD"); // 获取默认前瞻点
    JSON_TrackConfigData.ForwardHeightCompensationPxPerRow = ConfigData.at("FORWARD_HEIGHT_COMPENSATION_PX_PER_ROW"); // 获取巡线前瞻点高度补偿
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
    JSON_TrackConfigData.TrackKindCountThreshold = ConfigData.at("TRACK_KIND_COUNT_THRESHOLD");   // 获取赛道类型计数阈值
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

    if (ConfigData.contains("CIRCLE_JUMP_MAX")) {
        JSON_TrackConfigData.CircleJumpMax = ConfigData.at("CIRCLE_JUMP_MAX");
    }
    if (ConfigData.contains("CIRCLE_BORDER_QUIET_MAX")) {
        JSON_TrackConfigData.CircleBorderQuietMax = ConfigData.at("CIRCLE_BORDER_QUIET_MAX");
    }
    if (ConfigData.contains("CIRCLE_JUMP_EXPECTED")) {
        JSON_TrackConfigData.CircleJumpExpected = ConfigData.at("CIRCLE_JUMP_EXPECTED");
    }
    if (ConfigData.contains("CIRCLE_BORDER_ACTIVE_MIN")) {
        JSON_TrackConfigData.CircleBorderActiveMin = ConfigData.at("CIRCLE_BORDER_ACTIVE_MIN");
    }
    if (ConfigData.contains("CIRCLE_JUDGE_INFLECTION_MAX")) {
        JSON_TrackConfigData.CircleJudgeInflectionMax = ConfigData.at("CIRCLE_JUDGE_INFLECTION_MAX");
    }
    if (ConfigData.contains("CIRCLE_JUDGE_PARTIAL_SCORE")) {
        JSON_TrackConfigData.CircleJudgePartialScore = ConfigData.at("CIRCLE_JUDGE_PARTIAL_SCORE");
    }
    if (ConfigData.contains("CROSS_JUMP_PRIMARY")) {
        JSON_TrackConfigData.CrossJumpPrimary = ConfigData.at("CROSS_JUMP_PRIMARY");
    }
    if (ConfigData.contains("CROSS_JUMP_SECONDARY_MIN")) {
        JSON_TrackConfigData.CrossJumpSecondaryMin = ConfigData.at("CROSS_JUMP_SECONDARY_MIN");
    }
    if (ConfigData.contains("CROSS_JUMP_SECONDARY_MAX")) {
        JSON_TrackConfigData.CrossJumpSecondaryMax = ConfigData.at("CROSS_JUMP_SECONDARY_MAX");
    }
    if (ConfigData.contains("CROSS_BORDER_MIN")) {
        JSON_TrackConfigData.CrossBorderMin = ConfigData.at("CROSS_BORDER_MIN");
    }
    if (ConfigData.contains("ACROSS_BORDER_PREPARE_MAX")) {
        JSON_TrackConfigData.AcrossBorderPrepareMax = ConfigData.at("ACROSS_BORDER_PREPARE_MAX");
    }
    if (ConfigData.contains("ACROSS_BORDER_OUT_MIN")) {
        JSON_TrackConfigData.AcrossBorderOutMin = ConfigData.at("ACROSS_BORDER_OUT_MIN");
    }
    JSON_TrackConfigData.AcrossBorderExitMax = ConfigData.at("ACROSS_BORDER_EXIT_MAX");
    JSON_TrackConfigData.AcrossMaxFrames = ConfigData.at("ACROSS_MAX_FRAMES");
    std::cout << "[Config] ACROSS_MAX_FRAMES=" << JSON_TrackConfigData.AcrossMaxFrames
              << " ACROSS_BORDER_EXIT_MAX=" << JSON_TrackConfigData.AcrossBorderExitMax << std::endl;
    if (ConfigData.contains("TRACK_JUDGE_FULL_SCORE")) {
        JSON_TrackConfigData.TrackJudgeFullScore = ConfigData.at("TRACK_JUDGE_FULL_SCORE");
    }
    if (ConfigData.contains("TRACK_JUDGE_PARTIAL_SCORE")) {
        JSON_TrackConfigData.TrackJudgePartialScore = ConfigData.at("TRACK_JUDGE_PARTIAL_SCORE");
    }
    if (ConfigData.contains("TRACK_JUDGE_CONFIRM_THRESHOLD")) {
        JSON_TrackConfigData.TrackJudgeConfirmThreshold = ConfigData.at("TRACK_JUDGE_CONFIRM_THRESHOLD");
    }

    // 同步配置到运行时容器（覆盖旧值，保持单配置生效）。
    Function_EN_p->JSON_FunctionConfigData_v.clear();
    Data_Path_p->JSON_TrackConfigData_v.clear();
    Data_Path_p->JSON_DifferentialPDConfigData_v.clear();
    Data_Path_p->JSON_AngularVelocityPIDConfigData_v.clear();
    Data_Path_p->JSON_SpeedIncrementalPIConfigData_v.clear();
    Data_Path_p->JSON_VehicleConfigData_v.clear();
    Function_EN_p->JSON_FunctionConfigData_v.push_back(JSON_FunctionConfigData);
    Data_Path_p->JSON_TrackConfigData_v.push_back(JSON_TrackConfigData);
    Data_Path_p->JSON_DifferentialPDConfigData_v.push_back(JSON_DifferentialPDConfigData);
    Data_Path_p->JSON_AngularVelocityPIDConfigData_v.push_back(JSON_AngularVelocityPIDConfigData);
    Data_Path_p->JSON_SpeedIncrementalPIConfigData_v.push_back(JSON_SpeedIncrementalPIConfigData);
    Data_Path_p->JSON_VehicleConfigData_v.push_back(JSON_VehicleConfigData);

    std::cout << "[Config] Function 配置数量: " << Function_EN_p->JSON_FunctionConfigData_v.size()
              << ", Track 配置数量: " << Data_Path_p->JSON_TrackConfigData_v.size() << std::endl;
    std::cout << "[Config] Camera_EN=" << static_cast<int>(JSON_FunctionConfigData.Camera_EN)
              << ", Forward=" << JSON_TrackConfigData.Forward
              << ", ForwardHeightCompensationPxPerRow=" << JSON_TrackConfigData.ForwardHeightCompensationPxPerRow
              << ", PathSearch=[" << JSON_TrackConfigData.Path_Search_Start
              << ", " << JSON_TrackConfigData.Path_Search_End << "]" << std::endl;

    cout << "<---------------------JSON参数获取成功--------------------->" << endl;
}

std::string SYNC::GetConfigFilePath() const
{
    int JSON_FileNum = jsonnum;
    if (JSON_FileNum < 0 || JSON_FileNum > 2)
    {
        JSON_FileNum = 0;
    }

    switch (JSON_FileNum)
    {
        case 0: return "config/config_0.json";
        case 1: return "config/config_1.json";
        case 2: return "config/config_2.json";
        default: return "config/config_0.json";
    }
}


/*
    DataPrint说明
    打印数据
    程序参数：1.前瞻点 2.寻边线起始点 3.寻边线结束点 4.边线断点起始点 5.边线断点结束点 6.比赛状态
    运动参数：1.舵机方向 2.舵机角度 3.电机速度
*/
void DataPrint(Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];
    

    if(JSON_FunctionConfigData.DataPrint_EN == true)
    {

    }
}

