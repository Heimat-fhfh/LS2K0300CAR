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

    // 第一步：从当前帧提取拐点和弯点信息，作为状态机输入。
    Judge::InflectionPointSearch(Img_Store_p,Data_Path_p);
    Judge::BendPointSearch(Img_Store_p,Data_Path_p);
}
/*
    TrackKind_Judge说明
    赛道循环类型决策
*/
LoopKind Judge::TrackKind_Judge(Img_Store* Img_Store_p,Data_Path *Data_Path_p,Function_EN* Function_EN_p)
{
    JSON_FunctionConfigData JSON_FunctionConfigData = Function_EN_p -> JSON_FunctionConfigData_v[0];
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    static int Judge_Num = 0;    // 决策循环帧数计数器 
    
    /**
     * 如果存在左/右侧独立黑块，并且右/左侧存在拐点，则判定为十字赛道循环；
     * 如果存在左/右侧独立黑块, 并且右/左侧不存在拐点，则判定为圆环赛道循环；
     * 否则判定为普通赛道循环。
     */
    
    if ((Data_Path_p->black_left_found && Data_Path_p->InflectionPointNum[1] > 1))
    {
        Data_Path_p->Temp_Track_Kind = L_ACROSS_TRACK;
    }
    else if ((Data_Path_p->black_right_found && Data_Path_p->InflectionPointNum[0] > 1))
    {
        Data_Path_p->Temp_Track_Kind = R_ACROSS_TRACK;
    }
    else if (Data_Path_p->black_left_found)
    {
        Data_Path_p->Temp_Track_Kind = L_CIRCLE_TRACK;
    }
    else if (Data_Path_p->black_right_found)
    {
        Data_Path_p->Temp_Track_Kind = R_CIRCLE_TRACK;
    }
    else
    {
        Data_Path_p->Temp_Track_Kind = STRIGHT_TRACK;
    }

    return Data_Path_p->Loop_Kind;
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
        "DATA_PRINT_EN", "ACROSS_IDENTIFY_EN", "CIRCLE_IDENTIFY_EN","Track_width", "FORWARD", "PATH_SEARCH_START",
        "PATH_SEARCH_END", "SIDE_SEARCH_START", "SIDE_SEARCH_END", "POINT_DISTANCE", "LITTLE_ANGLE_BEND_POINT_NUM",
        "BIG_ANGLE_BEND_POINT_NUM", "MIN_INFLECTION_POINT_ANGLE", "MAX_INFLECTION_POINT_ANGLE", "MIN_BEND_POINT_ANGLE",
        "MAX_BEND_POINT_ANGLE", "TRACK_WIDTH", "CIRCLE_OUT_WIDTH", "STRIGHT_TRACK_MOTOR_SPEED",
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

    JSON_TrackConfigData.Track_width = ConfigData.at("Track_width");

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


