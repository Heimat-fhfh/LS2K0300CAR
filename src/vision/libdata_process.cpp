#include "common/common_system.h"
#include "common/common_program.h"
#include "vision/libdata_store.h"
#include <cerrno>
#include <cstring>

using namespace std;
using namespace cv;






/*
    MotorSpeed_Judge说明
    电机速度决策
*/
void Judge::MotorSpeed_Judge(Img_Store *Img_Store_p,Data_Path *Data_Path_p)
{
    JSON_TrackConfigData JSON_TrackConfigData = Data_Path_p -> JSON_TrackConfigData_v[0];

    Data_Path_p->TargetBaseSpeedMps = JSON_TrackConfigData.CommonMotorSpeed;
    
}














int jsonnum = 0; // 选择的json文件
bool changetimes = 0;

/*
    ConfigData_SYNC说明
    车辆上位机设置文件数据同步
*/
void SYNC::ConfigData_SYNC(Data_Path *Data_Path_p,Function_EN *Function_EN_p)
{
    if (Data_Path_p == nullptr || Function_EN_p == nullptr)
    {
        std::cerr << "[Config] ConfigData_SYNC 参数非法: Data_Path_p=" << Data_Path_p
                  << ", Function_EN_p=" << Function_EN_p << std::endl;
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
        cin >> JSON_FileNum;
        if (cin.fail())
        {
            cin.clear();
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cerr << "[Config] 输入不是有效数字，回退到 config_0.jsonc" << std::endl;
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
        std::cerr << "[Config] 非法配置编号: " << JSON_FileNum << "，回退到 config_0.jsonc" << std::endl;
        JSON_FileNum = 0;
        jsonnum = 0;
    }

    switch(JSON_FileNum)
    {
        case 0:{ ConfigFilePath = "config/config_0.jsonc"; break; }
        case 1:{ ConfigFilePath = "config/config_1.jsonc"; break; }
        case 2:{ ConfigFilePath = "config/config_2.jsonc"; break; }
        default:{ ConfigFilePath = "config/config_0.jsonc"; break; }
    }


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

    if (file_size <= 0)
    {
        std::cerr << "[Config] 配置文件为空: " << ConfigFilePath << std::endl;
        return;
    }

    nlohmann::json ConfigData;
    try
    {
        ConfigData = nlohmann::json::parse(ConfigFile, nullptr, true, true);
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
        "DIFF_OUTER_PD_KP", "DIFF_OUTER_PD_KD", "DIFF_OUTER_PD_LIMIT_P", "DIFF_OUTER_PD_LIMIT_D",
        "DIFF_OUTER_PD_LIMIT_OUTPUT", "DIFF_OUTER_PD_KP2",
        "DIFF_OUTPUT_RAMP_ENABLE", "DIFF_OUTPUT_RAMP_ACCEL_RATE", "DIFF_OUTPUT_RAMP_DECEL_RATE",

        "DIFF_INNER_PI_KP", "DIFF_INNER_PI_KI", "DIFF_INNER_PI_KD", "DIFF_INNER_PI_LIMIT_P", "DIFF_INNER_PI_LIMIT_I",
        "DIFF_INNER_PI_LIMIT_D", "DIFF_INNER_PI_LIMIT_OUTPUT", "DIFF_INNER_PI_LIMIT_I_MIN", "DIFF_INNER_PI_ANTI_WINDUP",
        "DIFF_INNER_PI_GKD", "DIFF_INNER_PI_GKD_LIMIT",

        "SPEED_INCR_PI_KP", "SPEED_INCR_PI_KI", "SPEED_INCR_PI_KD", "SPEED_INCR_PI_LIMIT_OUTPUT",

        "CAMERA_EN", "IMAGE_SAVE_EN",
        "ACROSS_IDENTIFY_EN", "CIRCLE_IDENTIFY_EN", "IPS200_SHOW_EN", "UDP_IMAGE_UPLOAD_EN",

        "PATH_SEARCH_START","PATH_SEARCH_END",
        
        "STRIGHT_TRACK_MOTOR_SPEED","CIRCLE_MAX_FRAMES"

        "CONTROL_PERIOD", "MOTOR_MAX_DUTY",

        "LPF_SPEED_TAU", "LPF_ANGULAR_TAU",

        "MOTOR_PWM_DEAD_ZONE_LEFT", "MOTOR_PWM_DEAD_ZONE_RIGHT",

        "COLLISION_PROTECT_ENABLE", "COLLISION_IMU_JERK_THRESHOLD",
        "COLLISION_STALL_DUTY_THRESHOLD", "COLLISION_STALL_SPEED_THRESHOLD",
        "COLLISION_STALL_CYCLES", "COLLISION_RESET_KEY", "COLLISION_BUMPER_KEY",

        "RAMP_ACCEL_RATE", "RAMP_DECEL_RATE",
        
        "CURVATURE_SPEED_GAIN","CURVATURE_SPEED_MIN",
        
        "BATTERY_LOW_THRESHOLD",

        "TARGET_BOARD_EN",
        "RED_BLOB_B_MIN", "RED_BLOB_B_MAX",
        "RED_BLOB_G_MIN", "RED_BLOB_G_MAX",
        "RED_BLOB_R_MIN", "RED_BLOB_R_MAX",
        "RED_BLOB_MIN_AREA", "RED_BLOB_Y_GATE",
        "TARGET_CONFIRM_FRAMES",
        "TARGET_ROI_OFFSET_X", "TARGET_ROI_OFFSET_Y",
        "TARGET_ROI_W", "TARGET_ROI_H",
        "TARGET_OVERRIDE_TIMEOUT_FRAMES",
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


    // 外环差速PD参数
    JSON_DifferentialPDConfigData.Kp = ConfigData.at("DIFF_OUTER_PD_KP");
    JSON_DifferentialPDConfigData.Kd = ConfigData.at("DIFF_OUTER_PD_KD");
    JSON_DifferentialPDConfigData.limitP = ConfigData.at("DIFF_OUTER_PD_LIMIT_P");
    JSON_DifferentialPDConfigData.limitD = ConfigData.at("DIFF_OUTER_PD_LIMIT_D");
    JSON_DifferentialPDConfigData.limitOutput = ConfigData.at("DIFF_OUTER_PD_LIMIT_OUTPUT");
    JSON_DifferentialPDConfigData.Kp2 = ConfigData.at("DIFF_OUTER_PD_KP2");

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
    JSON_AngularVelocityPIDConfigData.Gkd = ConfigData.at("DIFF_INNER_PI_GKD");
    JSON_AngularVelocityPIDConfigData.GkdLimit = ConfigData.at("DIFF_INNER_PI_GKD_LIMIT");

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

    // 斜坡控制参数
    JSON_VehicleConfigData.rampAccelRate = ConfigData.at("RAMP_ACCEL_RATE");
    JSON_VehicleConfigData.rampDecelRate = ConfigData.at("RAMP_DECEL_RATE");

    // 外环PD输出斜坡控制参数
    JSON_VehicleConfigData.diffOutputRampEnable = ConfigData.at("DIFF_OUTPUT_RAMP_ENABLE");
    JSON_VehicleConfigData.diffOutputRampAccelRate = ConfigData.at("DIFF_OUTPUT_RAMP_ACCEL_RATE");
    JSON_VehicleConfigData.diffOutputRampDecelRate = ConfigData.at("DIFF_OUTPUT_RAMP_DECEL_RATE");

    JSON_VehicleConfigData.curvatureSpeedGain = ConfigData.at("CURVATURE_SPEED_GAIN");
    JSON_VehicleConfigData.curvatureSpeedMin = ConfigData.at("CURVATURE_SPEED_MIN");
    JSON_VehicleConfigData.batteryLowThreshold = ConfigData.at("BATTERY_LOW_THRESHOLD");

    JSON_FunctionConfigData.Camera_EN = CameraKind(ConfigData.at("CAMERA_EN"));
    JSON_FunctionConfigData.ImageSave_EN = ConfigData.at("IMAGE_SAVE_EN");
    JSON_FunctionConfigData.AcrossIdentify_EN = ConfigData.at("ACROSS_IDENTIFY_EN");
    JSON_FunctionConfigData.CircleIdentify_EN = ConfigData.at("CIRCLE_IDENTIFY_EN");
    JSON_FunctionConfigData.IPS200_Show_EN = ConfigData.at("IPS200_SHOW_EN");
    JSON_FunctionConfigData.UDP_Image_Upload_EN = ConfigData.at("UDP_IMAGE_UPLOAD_EN");

    JSON_TargetBoardConfigData JSON_TargetBoardConfigData;
    JSON_TargetBoardConfigData.enable = ConfigData.at("TARGET_BOARD_EN");
    JSON_TargetBoardConfigData.bMin = ConfigData.at("RED_BLOB_B_MIN");
    JSON_TargetBoardConfigData.bMax = ConfigData.at("RED_BLOB_B_MAX");
    JSON_TargetBoardConfigData.gMin = ConfigData.at("RED_BLOB_G_MIN");
    JSON_TargetBoardConfigData.gMax = ConfigData.at("RED_BLOB_G_MAX");
    JSON_TargetBoardConfigData.rMin = ConfigData.at("RED_BLOB_R_MIN");
    JSON_TargetBoardConfigData.rMax = ConfigData.at("RED_BLOB_R_MAX");
    JSON_TargetBoardConfigData.minArea = ConfigData.at("RED_BLOB_MIN_AREA");
    JSON_TargetBoardConfigData.yGate = ConfigData.at("RED_BLOB_Y_GATE");
    JSON_TargetBoardConfigData.confirmFrames = ConfigData.at("TARGET_CONFIRM_FRAMES");
    JSON_TargetBoardConfigData.roiOffsetX = ConfigData.at("TARGET_ROI_OFFSET_X");
    JSON_TargetBoardConfigData.roiOffsetY = ConfigData.at("TARGET_ROI_OFFSET_Y");
    JSON_TargetBoardConfigData.roiW = ConfigData.at("TARGET_ROI_W");
    JSON_TargetBoardConfigData.roiH = ConfigData.at("TARGET_ROI_H");
    JSON_TargetBoardConfigData.overrideTimeoutFrames = ConfigData.at("TARGET_OVERRIDE_TIMEOUT_FRAMES");

    JSON_TrackConfigData.Path_Search_Start = ConfigData.at("PATH_SEARCH_START");
    JSON_TrackConfigData.Path_Search_End = ConfigData.at("PATH_SEARCH_END");

    JSON_TrackConfigData.CommonMotorSpeed = ConfigData.at("STRIGHT_TRACK_MOTOR_SPEED");

    JSON_TrackConfigData.CircleMaxFrames = ConfigData.at("CIRCLE_MAX_FRAMES");

    // 同步配置到运行时容器（覆盖旧值，保持单配置生效）。
    Function_EN_p->JSON_FunctionConfigData_v.clear();
    Data_Path_p->JSON_TrackConfigData_v.clear();
    Data_Path_p->JSON_DifferentialPDConfigData_v.clear();
    Data_Path_p->JSON_AngularVelocityPIDConfigData_v.clear();
    Data_Path_p->JSON_SpeedIncrementalPIConfigData_v.clear();
    Data_Path_p->JSON_VehicleConfigData_v.clear();
    Data_Path_p->JSON_TargetBoardConfigData_v.clear();
    Function_EN_p->JSON_FunctionConfigData_v.push_back(JSON_FunctionConfigData);
    Data_Path_p->JSON_TrackConfigData_v.push_back(JSON_TrackConfigData);
    Data_Path_p->JSON_DifferentialPDConfigData_v.push_back(JSON_DifferentialPDConfigData);
    Data_Path_p->JSON_AngularVelocityPIDConfigData_v.push_back(JSON_AngularVelocityPIDConfigData);
    Data_Path_p->JSON_SpeedIncrementalPIConfigData_v.push_back(JSON_SpeedIncrementalPIConfigData);
    Data_Path_p->JSON_VehicleConfigData_v.push_back(JSON_VehicleConfigData);
    Data_Path_p->JSON_TargetBoardConfigData_v.push_back(JSON_TargetBoardConfigData);

    std::cout << "[配置] 加载 " << ConfigFilePath
              << " | 摄像头: VIDEO" << static_cast<int>(JSON_FunctionConfigData.Camera_EN)
              << " | CIRCLE_MAX_FRAMES: " << JSON_TrackConfigData.CircleMaxFrames << std::endl;
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
        case 0: return "config/config_0.jsonc";
        case 1: return "config/config_1.jsonc";
        case 2: return "config/config_2.jsonc";
        default: return "config/config_0.jsonc";
    }
}




