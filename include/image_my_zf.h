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
  uint8_t IsRightFind;
  uint8_t IsLeftFind;
  uint8_t isBlackFind;
  int Wide;
  int LeftBorder;
  int RightBorder;
  int close_LeftBorder;
  int close_RightBorder;
  int opp_LeftBorder;
  int opp_RightBorder;
  int Center;
  int RightTemp;
  int LeftTemp;
  int CenterTemp;
  int Black_Point;
  int LeftBoundary_First;
  int RightBoundary_First;
  int LeftBoundary;
  int RightBoundary;
} ImageDealDatatypedef;

typedef enum {
  Normol,
  Straight,
  Cross,
  Ramp,
  LeftCirque,
  RightCirque,
  Forkin,
  Forkout,
  Barn_out,
  Barn_in,
  Cross_ture,
} RoadType_e;

typedef struct {
  int TowPoint;
  int TowPointAdjust_v;
  int TowPoint_True;
  int TowPoint_Gain;
  int TowPoint_Offset_Max;
  int TowPoint_Offset_Min;
  int Det_True;
  int Det_all;
  float Det_all_k;
  uint8_t Threshold;
  uint32_t Threshold_static;
  uint8_t Threshold_detach;
  uint8_t MiddleLine;
  int Foresight;
  uint8_t Left_Line;
  uint8_t Right_Line;
  uint8_t OFFLine;
  uint8_t WhiteLine;
  float ExpectCur;
  float White_Ritio;
  int Black_Pro_ALL;
  float Piriod_P;
  float MU_P;
  RoadType_e Road_type;
  uint8_t IsCinqueOutIn;
  uint8_t CirquePass;
  uint8_t CirqueOut;
  uint8_t CirqueOff;
  int16_t WhiteLine_L;
  int16_t WhiteLine_R;
  int16_t OFFLineBoundary;
  int Pass_Lenth;
  int Cirque1lenth;
  int Cirque2lenth;
  int Out_Lenth;
  int Fork_Out_Len;
  int Dowm_lenth;
  int Cross_Lenth;
  int Cross_ture_lenth;
  int Sita;
  int pansancha_Lenth;
  int Barn_Flag;
  int Barn_Lenth;
  int sanchaju;
  int Stop_lenth;
  int Ramp_lenth;
  int variance;
  int straight_acc;
  int variance_acc;
  int ramptestlenth;
  int rukuwait_lenth;
  int rukuwait_flag;
  int newblue_flag;
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

float my_abs(float x);
void ImageProcess_my_zf(cv::Mat& gray_80x60);

extern bool g_circle_identify_en;
extern bool g_across_identify_en;

#endif
