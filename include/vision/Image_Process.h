#ifndef IMAGE_MY_ZF_H
#define IMAGE_MY_ZF_H

#include <cstdint>
#include <opencv2/opencv.hpp>

#define LCDH 60
#define LCDW 80
#define LimitL(L) (L = ((L < 1) ? 1 : L))
#define LimitH(H) (H = ((H > 78) ? 78 : H))
#define ImageSensorMid 49

extern int ImageScanInterval;
extern int ImageScanInterval_Cross;
extern uint8_t Image_Use[LCDH][LCDW];
extern uint8_t Pixle[LCDH][LCDW];

typedef struct {
  int point;
  uint8_t type;
} JumpPointtypedef;

typedef struct {
  uint8_t IsRightFind;         // 右侧边线搜索状态：'T'=找到, 'W'=丢失/白线, 'H'=未找到, 'F'=未搜索
  uint8_t IsLeftFind;          // 左侧边线搜索状态：'T'=找到, 'W'=丢失/白线, 'H'=未找到, 'F'=未搜索
  uint8_t isBlackFind;         // [预留] 黑色像素查找标志，当前未使用
  int Wide;                    // 当前行赛道宽度 = RightBorder - LeftBorder
  int LeftBorder;              // 左侧赛道边界 X 坐标（主要边线）
  int RightBorder;             // 右侧赛道边界 X 坐标（主要边线）
  int close_LeftBorder;        // [预留] 邻近左侧边界，仅在帧初始化时赋值为 0，未参与逻辑
  int close_RightBorder;       // [预留] 邻近右侧边界，仅在帧初始化时赋值为 79，未参与逻辑
  int opp_LeftBorder;          // [预留] 对侧左侧边界，当前未使用
  int opp_RightBorder;         // [预留] 对侧右侧边界，当前未使用
  int Center;                  // 当前行赛道中心 X 坐标 = (LeftBorder + RightBorder) / 2，核心输出，用于舵机控制
  int RightTemp;               // [预留] 右侧边界临时值，仅在帧初始化时赋值为 79，未参与逻辑
  int LeftTemp;                // [预留] 左侧边界临时值，仅在帧初始化时赋值为 0，未参与逻辑
  int CenterTemp;              // [预留] 中心临时值，当前未使用（RouteFilter 中同名局部变量非此字段）
  int Black_Point;             // [预留] 黑色像素点坐标，当前未使用
  int LeftBoundary_First;      // 左侧 8 邻域边界跟踪首次命中点的 X 坐标（每行仅写一次，用于环岛入口判断）
  int RightBoundary_First;     // 右侧 8 邻域边界跟踪首次命中点的 X 坐标（每行仅写一次，用于环岛入口判断）
  int LeftBoundary;            // 左侧 8 邻域边界跟踪当前命中点的 X 坐标（跟踪过程中持续更新）
  int RightBoundary;           // 右侧 8 邻域边界跟踪当前命中点的 X 坐标（跟踪过程中持续更新）
} ImageDealDatatypedef;

typedef enum {
  Normol,                      // 普通赛道
  Straight,                    // 直道
  Cross,                       // 十字路口
  Ramp,                        // 坡道
  LeftCirque,                  // 左圆环/左环岛
  RightCirque,                 // 右圆环/右环岛
  Forkin,                      // 岔路入库
  Forkout,                     // 岔路出库
  Barn_out,                    // 车库出库
  Barn_in,                     // 车库入库
  Cross_ture,                  // 真实十字（斜入十字）
} RoadType_e;

typedef struct {
  int TowPoint;                // 固定前瞻点预设值，每帧初始化为 26
  int TowPointAdjust_v;        // [预留] 前瞻点动态调整值，当前未使用
  int TowPoint_True;           // 实际计算使用的前瞻点行号，GetDet()中根据赛道类型和环岛状态动态决定
  int TowPoint_Gain;           // [预留] 前瞻点增益系数，当前未使用
  int TowPoint_Offset_Max;     // [预留] 前瞻点偏移上限，当前未使用
  int TowPoint_Offset_Min;     // [预留] 前瞻点偏移下限，当前未使用
  int Det_True;                // 实际赛道中心偏差值，加权平均计算后用于舵机控制（0~79，中心为 39.5）
  int Det_all;                 // [预留] 总偏差值，当前未使用
  float Det_all_k;             // [预留] 总偏差系数，当前未使用
  uint8_t Threshold;           // 当前帧 OTSU 大津法二值化阈值，以 Threshold_static 为下限钳制
  uint32_t Threshold_static;   // 二值化阈值静态下限（固定 70），防止过暗场景阈值过低
  uint8_t Threshold_detach;    // OTSU 大津法阈值搜索的灰度级上限（固定 180），排除过亮噪声
  uint8_t MiddleLine;          // [预留] 中线检测标志，当前未使用
  int Foresight;               // [预留] 前瞻距离系数，当前未使用
  uint8_t Left_Line;           // 左侧边线丢失（类型 W）行计数，用于环岛元素判断
  uint8_t Right_Line;          // 右侧边线丢失（类型 W）行计数，用于环岛元素判断
  uint8_t OFFLine;             // 赛道有效边线最大行号（丢线行），该行以上边线不可靠
  uint8_t WhiteLine;           // 左右双侧同时丢线（W）的行计数，用于判断是否延长边线
  float ExpectCur;             // [预留] 期望曲率，当前未使用
  float White_Ritio;           // [预留] 赛道白色像素比例，当前未使用
  int Black_Pro_ALL;           // [预留] 全图黑色像素比例，当前未使用
  float Piriod_P;              // [预留] 周期 P 参数，当前未使用
  float MU_P;                  // [预留] MU 摩擦参数，当前未使用
  RoadType_e Road_type;        // 当前赛道类型（直道/十字/坡道/左圆环/右圆环/岔路入库/岔路出库/车库出库/车库入库/真十字）
  uint8_t IsCinqueOutIn;       // [预留] 环岛出入标志，当前未使用
  uint8_t CirquePass;          // [预留] 环岛通过标志，当前未使用
  uint8_t CirqueOut;           // [预留] 环岛驶出标志，当前未使用
  uint8_t CirqueOff;           // 环岛脱离标志（T/F），影响前瞻点选择和右边界延长逻辑
  int16_t WhiteLine_L;         // 左侧边界贴边白线计数（X < 3 的白像素行数），边缘跟踪时统计
  int16_t WhiteLine_R;         // 右侧边界贴边白线计数（X > LCDW-3 的白像素行数），边缘跟踪时统计
  int16_t OFFLineBoundary;     // 边界跟踪搜线时的丢线行号（左右边界跟踪交汇的行号）
  int Pass_Lenth;              // [预留] 通过路径长度，当前未使用
  int Cirque1lenth;            // [预留] 环岛第一阶段长度，当前未使用
  int Cirque2lenth;            // [预留] 环岛第二阶段长度，当前未使用
  int Out_Lenth;               // [预留] 出库长度，当前未使用
  int Fork_Out_Len;            // [预留] 岔路驶出长度，当前未使用
  int Dowm_lenth;              // [预留] 下坡通过长度，当前未使用
  int Cross_Lenth;             // [预留] 十字路口长度，当前未使用
  int Cross_ture_lenth;        // [预留] 真实十字长度，当前未使用
  int Sita;                    // [预留] 角度/方位参数，当前未使用
  int pansancha_Lenth;         // [预留] 盘三岔通过长度，当前未使用
  int Barn_Flag;               // [预留] 车库标志位，当前未使用
  int Barn_Lenth;              // [预留] 车库通过长度，当前未使用
  int sanchaju;                // [预留] 三岔距，当前未使用
  int Stop_lenth;              // [预留] 停车长度，当前未使用
  int Ramp_lenth;              // [预留] 坡道长度，当前未使用
  int variance;                // [预留] 方差值，当前未使用
  int straight_acc;            // [预留] 直道累积计数，当前未使用
  int variance_acc;            // [预留] 方差累积值，当前未使用
  int ramptestlenth;           // [预留] 坡道测试长度，当前未使用
  int rukuwait_lenth;          // [预留] 入库等待长度，当前未使用
  int rukuwait_flag;           // [预留] 入库等待标志，当前未使用
  int newblue_flag;            // [预留] 新蓝布标志，当前未使用
} ImageStatustypedef;

typedef struct {
  int16_t image_element_rings;
  int16_t ring_big_small;
  int16_t image_element_rings_flag;
  int16_t straight_long;
} ImageFlagtypedef;

extern ImageStatustypedef ImageStatus;
extern ImageDealDatatypedef ImageDeal[60];
extern ImageFlagtypedef ImageFlag;

void ImageProcess(cv::Mat& gray_80x60);

extern bool g_circle_identify_en;
extern bool g_across_identify_en;

extern uint8_t Ring_Help_Flag;
extern int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;
extern int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite;
extern int Point_Xsite, Point_Ysite;
extern int Repair_Point_Xsite, Repair_Point_Ysite;
extern uint8_t ExtenLFlag;
extern uint8_t ExtenRFlag;
extern uint8_t Half_Road_Wide[60];
extern uint8_t Half_Bend_Wide[60];

#endif
