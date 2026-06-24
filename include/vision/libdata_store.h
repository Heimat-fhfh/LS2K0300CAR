#include "common/common_system.h"
#include "common/common_program.h"
#include "control/PID.hpp"
#include "vision/AAAdefine.h"

#ifndef _LIBDATA_STORE_H_
#define _LIBDATA_STORE_H_

#define PI 3.1415926    // 圆周率

/*
    相机类型
*/
typedef enum CameraKind
{
    DEMO_VIDEO = 0, // 演示视频
    VIDEO_0 = 1,  // USB摄像头1
}CameraKind;

/*
    主函数循环类型(状态机)
*/
typedef enum LoopKind
{
    CAMERA_CATCH_LOOP = 0,    // 图像循环
    JUDGE_LOOP = 1,    // 决策循环
    COMMON_TRACK_LOOP = 2,   // 普通赛道循环
    ACROSS_TRACK_LOOP = 3,   // 十字赛道循环
    CIRCLE_TRACK_LOOP = 4,   // 圆环赛道循环
}LoopKind;



/*
    跳变扫描检测结果类型
*/
typedef enum TransitionElementKind
{
    TRANSITION_ELEMENT_NONE = 0,
    TRANSITION_ELEMENT_CIRCLE = 1,
    TRANSITION_ELEMENT_CROSS = 2,
} TransitionElementKind;

/* 
    十字步骤    
*/
typedef enum AcrossTrackStep
{
    ACROSS_PREPARE = 0, // 准备进入十字
    ACROSS = 1, // 十字内
    ACROSS_OUT = 2, // 出十字
    ACROSS_OUT_2 = 3, // 出十字2
    INIT_ACROSS = 4,   // 占位
} AcrossTrackStep;

/*
    圆环入环步骤
*/
typedef enum CircleTrackStep
{
    IN_PREPARE = 0, // 准备入环
    IN_PREPARE_2 = 1, // 准备入环2，增加一个准备阶段提高识别率
    IN = 2, // 入环,
    IN_CIRCLE = 3, // 圆环内
    OUT_PREPARE = 4,     // 准备出环
    OUT_STRIGHT = 5,  // 出环转直道
    OUT = 6,    // 出环
    INIT_CIRCLE = 7,   // 占位
}CircleTrackStep;




/*
    JSON文件存储的差速外环PD参数
*/
typedef struct JSON_DifferentialPDConfigData
{
    double Kp;
    double Kd;
    double limitP;
    double limitD;
    double limitOutput;
} JSON_DifferentialPDConfigData;

/*
    JSON文件存储的速度环增量式PID参数
*/
typedef struct JSON_SpeedIncrementalPIConfigData
{
    double Kp;
    double Ki;
    double Kd;
    double limitOutput;
} JSON_SpeedIncrementalPIConfigData;

/*
    JSON文件存储的角速度PID参数
*/
typedef struct JSON_AngularVelocityPIDConfigData
{
    double Kp;
    double Ki;
    double Kd;
    double limitP;
    double limitI;
    double limitD;
    double limitOutput;
    double limitIMin;
    bool enableAntiWindup;
} JSON_AngularVelocityPIDConfigData;

/*
    JSON文件存储的车辆控制参数
*/
typedef struct JSON_VehicleConfigData
{
    double controlPeriod;
    double motorMaxDuty;
    double lpfSpeedTau;
    double lpfAngularTau;
    double motorPwmDeadZoneLeft;    // 左电机PWM占空比死区 (0.0~1.0)
    double motorPwmDeadZoneRight;   // 右电机PWM占空比死区 (0.0~1.0)
    bool collisionProtectEnable;     // 碰撞保护总使能
    double collisionImuJerkThreshold;// IMU冲击检测阈值 (g)，默认 3.0
    double collisionStallDutyThreshold; // 堵转检测占空比阈值 (0~1)
    double collisionStallSpeedThreshold;// 堵转检测速度阈值 (m/s)
    int collisionStallCycles;        // 堵转确认周期数 (×控制周期)
    int collisionResetKey;           // 复位按键 (0=KEY0, 1=KEY1, 2=KEY2, 3=KEY3)
    int collisionBumperKey;          // 碰撞开关按键 (-1=禁用, 0=KEY0, ...)
    double rampAccelRate;            // 电机输出加速斜坡限制 (%/s)
    double rampDecelRate;            // 电机输出减速斜坡限制 (%/s)
    bool diffOutputRampEnable;       // 外环PD输出斜坡使能
    double diffOutputRampAccelRate;  // 外环PD输出角加速度限幅 (rad/s²)
    double diffOutputRampDecelRate;  // 外环PD输出角减速度限幅 (rad/s²)
    double curvatureSpeedGain;       // 曲率自适应降速增益 (0=禁用, 0.3~0.5=推荐)
    double curvatureSpeedMin;        // 曲率自适应降速下限 (目标速度的最小比例, 推荐0.1)
    double batteryLowThreshold;      // 电池低电压阈值(mV), 低于此值蜂鸣器长响3声警告
} JSON_VehicleConfigData;


/*
    JSON文件存储的工程功能设置参数
*/
typedef struct JSON_FunctionConfigData
{
    CameraKind Camera_EN;   // 相机使能
    bool ImageSave_EN;  // 图像存储使能
    bool AcrossIdentify_EN;    // 十字特征点识别使能
    bool CircleIdentify_EN;    // 圆环特征点识别使能
    bool IPS200_Show_EN;       // IPS200 屏幕显示使能（my_zf 80x60）
    bool UDP_Image_Upload_EN;  // UDP 图像上传使能
}JSON_FunctionConfigData;

/*
    JSON文件存储的赛道识别设置参数
*/
typedef struct JSON_TrackConfigData
{
    int Track_width;
    int Path_Search_Start;  // 寻路径起始点
    int Path_Search_End;    // 寻路径结束点
    int Side_Search_End; // 寻边线结束点
    double CommonMotorSpeed = 0;    // 电机速度
    int TransitionMinArea = 1000; // 独立黑色区域最小面积
    int CircleMaxFrames = 300;         // 圆环最大帧数限制
}JSON_TrackConfigData;

struct InversePerspectiveMap {
    int local_x;
    int local_y;
};

/*
    图像存储
*/
typedef struct Img_Store
{
    cv::Mat Img_CaptureBuffer[2];   // 摄像头双缓冲
    int Img_WriteIndex = 0;         // 生产者写入索引
    int Img_ReadIndex = 0;          // 消费者读取索引
    bool Img_BufferReady[2] = {false, false}; // 双缓冲有效标志
    uint64_t Img_FrameSeq = 0;      // 最新帧序号(生产者递增)
    uint64_t Img_LastReadSeq = 0;   // 消费者已读取帧序号
    bool CameraThreadRunning = false; // 摄像头采集线程运行状态

    int ImgNum = 0;

    uint8 bin_image[image_h][image_w];

    uint8 PerImg_ip[RESULT_ROW][RESULT_COL];    // 逆透视
    std::vector<std::vector<InversePerspectiveMap>> mapping;

    cv::Mat Img_Color; 
    cv::Mat Img_Color_Unpivot = cv::Mat(RESULT_ROW, RESULT_COL, CV_8UC3);
    cv::Mat Img_Gray;    
    cv::Mat Img_Gray_Unpivot; 
    cv::Mat Img_OTSU;    
    cv::Mat Img_OTSU_Unpivot;
    cv::Mat Img_Track;   
    cv::Mat Img_Track_Unpivot = cv::Mat(RESULT_ROW, RESULT_COL, CV_8UC3); 
    cv::Mat Img_Text;  
    cv::Mat Img_All;   

    Img_Store() = default;
    // 从数组加载数据
    // ...existing code...

void LoadData(cv::Mat img, const uint8 src[RESULT_ROW][RESULT_COL]) {
    // 检查 img 是否为空
    if (img.empty()) {
        std::cerr << "Error: img is empty in LoadData" << std::endl;
        return;
    }

    // 将源数组转换为 cv::Mat
    cv::Mat srcMat(RESULT_ROW, RESULT_COL, CV_8UC1, (void*)src);

    // 检查尺寸是否匹配
    if (img.rows != RESULT_ROW || img.cols != RESULT_COL || img.type() != CV_8UC1) {
        std::cerr << "Error: img size or type mismatch in LoadData" << std::endl;
        return;
    }

    // 使用 copyTo 方法复制数据
    srcMat.copyTo(img);
}
}Img_Store;

/*
    函数使能
*/
typedef struct Function_EN
{
    std::vector<JSON_FunctionConfigData> JSON_FunctionConfigData_v;   // JSON文件存储的工程功能设置参数
    bool Game_EN;   // 比赛开始
    bool Gyroscope_EN;    // 陀螺仪状态使能：当陀螺仪积分到一定角度时出环
    bool Control_EN = false; // 控制权转移使能
}Function_EN;


/*
    路径相关数据
*/
typedef struct Data_Path
{
    std::vector<JSON_TrackConfigData> JSON_TrackConfigData_v; // JSON文件存储的赛道识别设置参数
    std::vector<JSON_DifferentialPDConfigData> JSON_DifferentialPDConfigData_v; // JSON文件存储的外环差速PD参数
    std::vector<JSON_AngularVelocityPIDConfigData> JSON_AngularVelocityPIDConfigData_v; // JSON文件存储的内环角速度PI参数
    std::vector<JSON_SpeedIncrementalPIConfigData> JSON_SpeedIncrementalPIConfigData_v; // JSON文件存储的速度环增量式PI参数
    std::vector<JSON_VehicleConfigData> JSON_VehicleConfigData_v; // JSON文件存储的车辆控制参数
    
    
    int NumSearch[2] = {0}; // 左右八邻域寻线坐标数量
    uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
    uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
    int SideCoordinate_Eight[(uint16)USE_num][4] = {0};   // 左右边线坐标(八邻域)

    static constexpr int kEdgeLineColorBlockMax = 7;
    int EdgeLineColorBlockNum[2] = {0}; // 左右边线有效同色段数量，最多记录3段
    int EdgeLineJumpNum[2] = {0}; // 左右边线有效同色段之间的跳变次数
    int BorderPointNum[2] = {0};  // 左右边线中位于图像左右边界区域的点的个数
    int EdgeLineColorBlockColor[2][kEdgeLineColorBlockMax] = {{0}}; // 有效同色段颜色
    int EdgeLineColorBlockLength[2][kEdgeLineColorBlockMax] = {{0}}; // 有效同色段长度
    cv::Point EdgeLineColorBlockStart[2][kEdgeLineColorBlockMax] = {}; // 有效同色段起点
    cv::Point EdgeLineColorBlockEnd[2][kEdgeLineColorBlockMax] = {}; // 有效同色段终点

    uint16 l_border[image_h];            //左线数组
    uint16 r_border[image_h];            //右线数组
    int SideCoordinate[(uint16)USE_num][4] = {0};   // 左右边线坐标(中线寻线法)

    uint16 center_line[image_h];         //中线数组
    int TrackCoordinate[(uint16)USE_num][2] = {0};   // 路径线坐标

    uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
    uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向

    uint16 search_print_h_max = 0;//最高点
    int forword_line_h = 0; // 前瞻点高度

    int SeedSearchFailCount = 0; // 起始点搜索连续失败帧数计数(出界保护用)

    int InflectionPointCoordinate[(uint16)USE_num][4] = {0};  // 左右边线元素拐点坐标
    int BendPointCoordinate[(uint16)USE_num][4] = {0};  // 左右边线弯点坐标

    int Vector_Add_Unit_Dir[2];   // 左右拐点上下两向量纵坐标加和方向
    int InflectionPointNum[2] = {0};    // 元素拐点数量
    int BendPointNum[2] = {0};    // 边线弯点数量
    
    bool black_left_found = false;   // 独立黑色区域寻找到左边标志
    bool black_right_found = false;  // 独立黑色区域寻找到右边标志
    int TransitionDetectKind;       // 当前确认的检测结果 (TransitionElementKind)
    int TransitionDetectSide;       // 检测到的元素侧 (0=none, 1=left, 2=right)
    std::vector<std::vector<cv::Point>> TransitionContours; // 存储跳变扫描检测到的轮廓坐标
    std::vector<cv::Vec4i> TransitionHierarchy; // 存储跳变扫描检测到的轮廓层级信息
    cv::Point leftmost_point ; // 独立黑块最左侧位置
    cv::Point rightmost_point; // 独立黑块最右侧位置



    // 差速控制目标量（由上层视觉控制计算，下发给 MotorControlTask）
    int SteerErrorPx = 0;                 // 带符号的横向误差（像素）
    double TargetBaseSpeedMps = 0;        // 目标线速度（m/s）
    double TargetDifferentialSpeed = 0;   // 外环PD输出的期望差速

    // 控制参数
    int ServoDir = 0;  // 舵机方向
    int ServoAngle = 0;    // 舵机角度
    

}Data_Path;



#endif
