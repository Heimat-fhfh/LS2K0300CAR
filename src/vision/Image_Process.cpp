#include "vision/Image_Process.h"
#include "vision/target_board.h"  // g_target_override (目标板循迹线 override)

void Element_Judgment_Left_Rings();
void Element_Judgment_Right_Rings();
void Element_Handle_Left_Rings();
void Element_Handle_Right_Rings();

// 环岛/元素识别使能开关
// g_circle_identify_en: 圆环（环岛）元素识别总开关，由外部（如按键/拨码）控制
// g_across_identify_en:  十字路口元素识别总开关，由外部（如按键/拨码）控制
bool g_circle_identify_en = true;
bool g_across_identify_en = true;

// 边线搜索区间半径（像素），用于从上一行的左右边界向外扩展搜索范围
// 普通赛道用 ImageScanInterval（±2像素），十字路口用 ImageScanInterval_Cross（±2像素）
int ImageScanInterval = 2;
int ImageScanInterval_Cross = 2;

// Image_Use: 原始灰度图像缓存（60行×80列），由摄像头采集后缩放而来，取值范围 0~255
// Pixle:     二值化后的图像缓存（60行×80列），0=黑色（赛道），1=白色（边线/背景）
uint8_t Image_Use[LCDH][LCDW];
uint8_t Pixle[LCDH][LCDW];

// ---- 静态全局工作变量（函数间共享） ----
// Ysite/Xsite:      当前处理的行/列坐标，几乎所有函数共用
// PicTemp:          指向 Pixle 当前处理行首地址的指针，加速逐像素访问
// IntervalLow/IntervalHigh: 边线搜索的列坐标区间 [Low, High]
// ytemp:            临时行号变量，用于循环和控制
// TFSite/FTSite:    延长线计算中的"起点行"和"终点行"（From/To）
// DetR/DetL:        右侧/左侧边界延长线斜率（列偏移/行）
// BottomBorderRight/BottomBorderLeft: 图像底部（第59行）的左右赛道边界 X 坐标
// BottomCenter:     图像底部的赛道中心 X 坐标
static int Ysite = 0, Xsite = 0;
static uint8_t *PicTemp;
static int IntervalLow = 0, IntervalHigh = 0;
static int ytemp = 0;
static int TFSite = 0, FTSite = 0;
static float DetR = 0, DetL = 0;
static int BottomBorderRight = 79;
static int BottomBorderLeft = 0;
static int BottomCenter = 0;

// ImageDeal:    60行逐行图像处理结果，存储每行的边界、中心、宽度等信息
// ImageStatus:  全局图像处理状态（赛道类型、阈值、丢线行、前瞻点等）
// ImageFlag:    全局图像标志（环岛标志、直道长度等）
ImageDealDatatypedef ImageDeal[60];
ImageStatustypedef ImageStatus;
ImageFlagtypedef ImageFlag;

// 加权系数表（10级），用于 GetDet() 计算前瞻中心偏差的加权平均
// 距前瞻点越近的行权重越大（最大值0.96），越远权重越小（最小值0.47）
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};

// ExtenLFlag/ExtenRFlag: 左/右侧延长线标志，'F'=不延长，'T'=可延长
// Ring_Help_Flag:        环岛辅助标志
// Left_RingsFlag_Point*/Right_RingsFlag_Point*: 左右环岛入口/出口参考点坐标
// Point_Xsite/Point_Ysite:           通用元素参考点坐标
// Repair_Point_Xsite/Repair_Point_Ysite: 边界修补参考点坐标
uint8_t ExtenLFlag = 0;
uint8_t ExtenRFlag = 0;
uint8_t Ring_Help_Flag = 0;
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite;
int Point_Xsite, Point_Ysite;
int Repair_Point_Xsite, Repair_Point_Ysite;

// Half_Road_Wide: 直道/普通弯道各行的半宽（像素），与行号 Ysite 对应
// 值从 4（远行，窄）递增到 32（近行，宽），用于环岛入口判断
uint8_t Half_Road_Wide[60] = {  
    4, 5, 5, 6, 6, 6, 7, 7, 8, 8,
    9, 9,10,10,10,11,12,12,13,13,
    13,14,14,15,15,16,16,17,17,17,
    18,18,19,19,20,20,20,21,21,22,
    23,23,23,24,24,25,25,25,26,26,
    27,28,28,28,29,30,31,31,31,32,
};

// Half_Bend_Wide: 急弯/环岛区域各行的半宽（像素），与行号 Ysite 对应
// 值从 33（远行，允许更大宽度容差）递减到 21 再回升到 33，用于环岛和急弯判断
uint8_t Half_Bend_Wide[60] = {   
    33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    28,27,27,26,26,25,25,24,24,23,
    22,21,21,22,22,22,23,24,24,24,
    25,25,25,26,26,26,27,27,28,28,
    28,29,29,30,30,31,31,32,32,33,
};

/*
 * compressimage - 将 OpenCV Mat 灰度图像数据提取到全局 Image_Use 数组中
 *
 * 参数: gray_80x60 - 已缩放为 80×60 的单通道灰度 Mat（CV_8UC1）
 *
 * 功能: 逐像素将 Mat 的灰度值复制到 Image_Use[行][列]，
 *       作为后续大津法二值化（Get01change_dajin）的输入数据源。
 */
void compressimage(cv::Mat &gray_80x60)
{
  for (int i = 0; i < gray_80x60.rows; i++)
  {
    for (int j = 0; j < gray_80x60.cols; j++)
    {
      Image_Use[i][j] = gray_80x60.at<uint8_t>(i, j);
    }
  }
}

/*
 * Threshold_deal - 大津法（OTSU）自适应阈值计算（带灰度上限裁剪）
 *
 * 参数:
 *   image           - 灰度图像数据指针（一维数组，行优先存储）
 *   col             - 图像列数（宽度）
 *   row             - 图像行数（高度）
 *   pixel_threshold - 灰度搜索上限（截断值），只搜索 [0, pixel_threshold) 灰度区间，
 *                     超出部分不参与类间方差计算，用于排除高亮噪声
 *
 * 返回: 最优二值化阈值（uint8_t，0~255）
 *
 * 算法:
 *   标准 OTSU 大津法 + 提前退出优化：
 *   1. 统计每个灰度级 [0, 255] 的像素数 pixelCount 和灰度总和 gray_sum
 *   2. 计算每个灰度级的概率 pixelPro[i] = count[i] / total
 *   3. 在 [0, pixel_threshold) 范围内，对每个候选阈值 j 计算类间方差:
 *        sigma^2 = w0*(u0 - u)^2 + w1*(u1 - u)^2
 *        其中 w0/w1 是前/背景像素占比，u0/u1 是前/背景灰度均值，u 是全局均值
 *   4. 选择类间方差最大的灰度级作为阈值
 *   5. 一旦类间方差开始下降（deltaTmp < deltaMax），立即退出循环
 *      （类间方差函数是单峰的，峰值之后不再需要后续计算）
 *
 * 设计意图:
 *   通过 pixel_threshold 限制搜索范围的高端，可以避免过亮的噪声像素干扰阈值计算，
 *   同时利用 OTSU 类间方差的单峰特性提前退出以加速。
 */
uint8_t Threshold_deal(uint8_t *image, uint16_t col, uint16_t row, uint32_t pixel_threshold)
{
#define GrayScale 256
  uint16_t width = col;
  uint16_t height = row;
  int pixelCount[GrayScale]; // 各灰度级的像素计数
  float pixelPro[GrayScale]; // 各灰度级的像素概率（0~1）
  int i, j, pixelSum = width * height;
  uint8_t threshold = 0;
  uint8_t *data = image;
  // 初始化像素计数和概率数组
  for (i = 0; i < GrayScale; i++)
  {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }
  // 第一遍扫描：统计直方图和灰度总和
  uint32_t gray_sum = 0;
  for (i = 0; i < height; i += 1)
  {
    for (j = 0; j < width; j += 1)
    {
      pixelCount[(int)data[i * width + j]]++;
      gray_sum += (int)data[i * width + j];
    }
  }
  // 计算各灰度级的概率分布
  for (i = 0; i < GrayScale; i++)
  {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  // OTSU 类间方差计算变量
  // w0: 前景（低于阈值）像素占比；w1: 背景（高于阈值）像素占比
  // u0tmp: 前景加权灰度累加；u1tmp: 背景加权灰度累加
  // u0: 前景均值；u1: 背景均值；u: 全局均值
  // deltaTmp: 当前阈值下的类间方差；deltaMax: 最大类间方差
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  // 在 [0, pixel_threshold) 范围搜索最佳阈值
  for (j = 0; (uint32_t)j < pixel_threshold; j++)
  {
    w0 += pixelPro[j];                   // 前景像素累积占比
    u0tmp += j * pixelPro[j];            // 前景加权灰度累加
    w1 = 1 - w0;                         // 背景像素占比
    u1tmp = gray_sum / pixelSum - u0tmp; // 背景加权灰度 = 全局均值 - 前景加权灰度
    u0 = u0tmp / w0;                     // 前景灰度均值
    u1 = u1tmp / w1;                     // 背景灰度均值
    u = u0tmp + u1tmp;                   // 全局灰度均值
    // 类间方差 = w0*(u0 - u)^2 + w1*(u1 - u)^2
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax)
    {
      deltaMax = deltaTmp;
      threshold = j; // 记录当前最佳阈值
    }
    // 类间方差为单峰函数，一旦下降说明已越过峰值，可提前退出
    if (deltaTmp < deltaMax)
      break;
  }
  return threshold;
}

/*
 * Get01change_dajin - 对全局灰度图像 Image_Use 进行二值化，结果存入 Pixle
 *
 * 输入:  Image_Use[60][80] — 灰度图像（0~255）
 * 输出:  Pixle[60][80]     — 二值图像（0=黑色/赛道，1=白色/边线/背景）
 *
 * 处理流程:
 *   1. 调用 Threshold_deal() 计算全局 OTSU 阈值
 *   2. 将 OTSU 阈值与静态下限 ImageStatus.Threshold_static 比较，
 *      若计算值过低（暗光场景），则取静态下限作为安全兜底
 *   3. 逐像素二值化，灰度 > 阈值 → 1（白），否则 → 0（黑）
 *   4. 对图像边缘区域（左 0~15 列、右 65~79 列）使用降低 10 的阈值，
 *      目的是在图像边缘更容易检测到白色边线，补偿镜头边缘变暗效应
 */
void Get01change_dajin()
{
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8_t i, j = 0;
  uint8_t thre;
  for (i = 0; i < LCDH; i++)
  { // 遍历 60 行
    for (j = 0; j < LCDW; j++)
    { // 遍历 80 列
      // 图像左右边缘区域使用更低的阈值（阈值 - 10），使白边更容易被检出
      if (j <= 15) // 左侧 0~15 列
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75)) // 右侧 71~75 列
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65) // 右侧 65~79 列
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold; // 中间区域使用正常阈值
      if (Image_Use[i][j] > (thre))
        Pixle[i][j] = 1; // 白色像素
      else
        Pixle[i][j] = 0; // 黑色像素
    }
  }
}

/*
 * GetJumpPointFromDet - 在二值图像某行的指定区间内搜索黑白跳变点（赛道边界）
 *
 * 参数:
 *   p    - 指向 Pixle 当前行首元素的指针
 *   type - 搜索方向: 'L'=从右向左搜索左边界（黑→白跳变），'R'=从左向右搜索右边界（白→黑跳变）
 *   L    - 搜索区间左端点（列坐标）
 *   H    - 搜索区间右端点（列坐标）
 *   Q    - 输出参数，存储搜索到的跳变点信息（列坐标 + 类型）
 *
 * 搜索逻辑（以 type='L' 为例，从右向左搜索左侧赛道边界）:
 *   从 H 向 L 方向扫描，寻找第一个满足以下条件的列 i:
 *     *(p+i) == 1 && *(p+i-1) != 1  即 当前像素为白且左侧像素为黑 → 黑变白的上升沿
 *   若找到 → Q->point=i, Q->type='T'（正常找到）
 *   若扫描到区间起点 (L+1) 仍未找到 → 进入兜底逻辑:
 *     - 若区间中点 (L+H)/2 处不是黑像素 → Q->point=中点, Q->type='W'（白线/丢失）
 *     - 否则 → Q->point=H, Q->type='H'（没找到，用区间上界兜底）
 *
 * 对于 type='R'（从左向右搜索右侧赛道边界）:
 *   从 L 向 H 方向扫描，寻找 *(p+i)==1 && *(p+i+1)!=1（白变黑的下降沿）
 *   兜底逻辑为区间中点和 L。
 *
 * 返回值说明:
 *   'T' (True):  正常找到了边界跳变点
 *   'W' (White): 区间内可能全是白线，用区间中点近似
 *   'H' (Help):  完全没找到有效边界，用区间端点兜底
 */
void GetJumpPointFromDet(uint8_t *p, uint8_t type, int L, int H, JumpPointtypedef *Q)
{
  int i = 0;
  if (type == 'L')
  {
    // ========== 搜索左边界（从右向左扫描） ==========
    for (i = H; i >= L; i--)
    {
      // 找到上升沿: 当前白(1) 且 左侧黑(!=1)
      if (*(p + i) == 1 && *(p + i - 1) != 1)
      {
        Q->point = i;
        Q->type = 'T'; // 正常找到跳变点
        break;
      }
      else if (i == (L + 1))
      {
        // 扫描到区间起点附近仍未找到 → 兜底处理
        if (*(p + (L + H) / 2) != 0)
        {
          // 区间中点不是纯黑 → 判定为白线区域，取中点
          Q->point = (L + H) / 2;
          Q->type = 'W'; // 白线/边界丢失
          break;
        }
        else
        {
          // 区间中点也是黑色 → 完全没找到，用区间上界兜底
          Q->point = H;
          Q->type = 'H'; // 未找到
          break;
        }
      }
    }
  }
  else if (type == 'R')
  {
    // ========== 搜索右边界（从左向右扫描） ==========
    for (i = L; i <= H; i++)
    {
      // 找到下降沿: 当前白(1) 且 右侧黑(!=1)
      if (*(p + i) == 1 && *(p + i + 1) != 1)
      {
        Q->point = i;
        Q->type = 'T'; // 正常找到跳变点
        break;
      }
      else if (i == (H - 1))
      {
        // 扫描到区间终点附近仍未找到 → 兜底处理
        if (*(p + (L + H) / 2) != 0)
        {
          Q->point = (L + H) / 2;
          Q->type = 'W'; // 白线/边界丢失
          break;
        }
        else
        {
          Q->point = L;
          Q->type = 'H'; // 未找到
          break;
        }
      }
    }
  }
}

/*
 * DrawLinesFirst - 初步扫描图像底部 5 行（第 59~55 行），确定赛道初始边界
 *
 * 返回值: 始终返回 'T'（已无实际用途，保留兼容）
 *
 * 算法步骤:
 *   1. 以第 59 行（最底部）为基准行，以 ImageSensorMid(49) 为中心点
 *   2. 若中心点处为黑色（赛道内）:
 *      - 向左右两侧扩展寻找最近的白色像素（边线），确定 BottomBorderLeft/BottomBorderRight
 *      - 再向内收缩找到连续两个黑色像素作为可靠的边界起算点
 *   3. 若中心点处为白色（已在赛道外）:
 *      - 从两端向中间搜索连续两个白色像素来确定 BottomBorderLeft/BottomBorderRight
 *   4. 计算 BottomCenter（底部中心）并向第 54~55 行自底向上逐行传递边界信息
 *      - 从上一行的 Center 向外搜索连续两个白色像素确定边界
 *      - 若找不到则用上一行 Center 兜底
 *   5. 将结果写入 ImageDeal[59]~ImageDeal[55]
 *
 * 设计意图:
 *   赛道底部 5 行通常最清晰可靠，先在此区间建立准确的边界参考，
 *   为后续 DrawLinesProcess 的逐行向上搜索提供可靠的起算数据。
 */
static uint8_t DrawLinesFirst(void)
{
  PicTemp = Pixle[59]; // 指向第 59 行（图像底部）
  if (*(PicTemp + ImageSensorMid) == 0)
  {
    // ---- 情况1: 中心点在赛道内（黑色），向两侧寻找边线 ----
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0) // 左侧找到白像素
        break;
      if (*(PicTemp + ImageSensorMid + Xsite) != 0) // 右侧找到白像素
        break;
    }
    if (*(PicTemp + ImageSensorMid - Xsite) != 0)
    {
      // 先找到左侧白像素 → 确定右边界，再向左收缩找左边界
      BottomBorderRight = ImageSensorMid - Xsite + 1;
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)
      {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0)
        {
          BottomBorderLeft = Xsite; // 找到连续两个黑像素 → 边界
          break;
        }
        else if (Xsite == 1)
        {
          BottomBorderLeft = 0; // 到达最左端
          break;
        }
      }
    }
    else if (*(PicTemp + ImageSensorMid + Xsite) != 0)
    {
      // 先找到右侧白像素 → 确定左边界，再向右收缩找右边界
      BottomBorderLeft = ImageSensorMid + Xsite - 1;
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)
      {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0)
        {
          BottomBorderRight = Xsite; // 找到连续两个黑像素 → 边界
          break;
        }
        else if (Xsite == 78)
        {
          BottomBorderRight = 79; // 到达最右端
          break;
        }
      }
    }
  }
  else
  {
    // ---- 情况2: 中心点在赛道外（白色），从两端向中间搜索边线 ----
    for (Xsite = 79; Xsite > ImageSensorMid; Xsite--)
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1)
      {
        BottomBorderRight = Xsite; // 找到右侧连续白像素 → 右边界
        break;
      }
      else if (Xsite == 40)
      {
        BottomBorderRight = 39; // 兜底：默认边界
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1)
      {
        BottomBorderLeft = Xsite; // 找到左侧连续白像素 → 左边界
        break;
      }
      else if (Xsite == 38)
      {
        BottomBorderLeft = 39; // 兜底：默认边界
        break;
      }
    }
  }
  // 计算底部第 59 行的赛道信息
  BottomCenter = (BottomBorderLeft + BottomBorderRight) / 2;
  ImageDeal[59].LeftBorder = BottomBorderLeft;
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  // 从第 58 行向上处理到第 55 行，基于下一行的 Center 搜索当前行边界
  for (Ysite = 58; Ysite > 54; Ysite--)
  {
    PicTemp = Pixle[Ysite];
    // 搜索右边界: 从最右侧向左扫描，以 (上一行 Center) 为搜索左限
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center; Xsite--)
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1)
      {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      }
      else if (Xsite == (ImageDeal[Ysite + 1].Center + 1))
      {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center; // 兜底：用上一行中心
        break;
      }
    }
    // 搜索左边界: 从最左侧向右扫描，以 (上一行 Center) 为搜索右限
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center; Xsite++)
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1)
      {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      }
      else if (Xsite == (ImageDeal[Ysite + 1].Center - 1))
      {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center; // 兜底：用上一行中心
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
  }
  return 'T';
}

/*
 * DrawLinesProcess - 主边界跟踪函数：从第 54 行向上逐行搜索赛道左右边界
 *
 * 功能:
 *   本函数是图像处理的核心，以 DrawLinesFirst 建立的底部边界为起点，
 *   从第 54 行向上逐行搜索至 OFFLine（丢线行）为止，同时进行:
 *     - 跳变点检测（GetJumpPointFromDet）
 *     - 边界兜底恢复
 *     - 丢失边界的延长线外推
 *     - 白线计数（用于元素识别）
 *     - 丢线条件判断
 *
 * 搜索策略:
 *   每一行的左右边界搜索区间基于上一行的边界位置 ± ImageScanInterval 张成。
 *   使用 GetJumpPointFromDet 寻找黑白跳变沿。
 *   - 'T': 正常定位 → 使用跳变点作为新边界
 *   - 'W': 边界丢失（全白区域）→ 保持上一行边界不变（后续由 DrawExtensionLine 修复）
 *   - 'H': 找不到跳变点 → 在当前左右边界之间重新扫描寻找，宽度≤7 则丢线
 *
 * 边界延长外推:
 *   当某一侧连续多行出现 'W'（边界丢失）时，在 'W' 区域的上方寻找可靠的 'T' 行，
 *   使用线性插值填充丢失区域的边界值（计算斜率 D_L/D_R）。
 *
 * 丢线条件（满足其一即停止向上搜索）:
 *   1. 当前行赛道宽度 ≤ 7 像素
 *   2. 右边界 ≤ 10（左边线压到最左）或左边界 ≥ 70（右边线压到最右）
 *   3. 左右边界都显示 'H'（找不到跳变点）且宽度过窄
 *
 * 同时统计:
 *   - WhiteLine: 左右同时丢线（'W'）的行数
 *   - Left_Line:  左边界丢线的行数（用于环岛判断）
 *   - Right_Line: 右边界丢线的行数（用于环岛判断）
 */
static void DrawLinesProcess(void)
{
  uint8_t L_Found_T = 'F';  // 左边界延长线是否已计算标志
  uint8_t Get_L_line = 'F'; // 是否已触发左边界延长线搜索
  uint8_t R_Found_T = 'F';  // 右边界延长线是否已计算标志
  uint8_t Get_R_line = 'F'; // 是否已触发右边界延长线搜索
  float D_L = 0;            // 左边界延长线斜率（列偏移/行）
  float D_R = 0;            // 右边界延长线斜率（列偏移/行）
  int ytemp_W_L = 0;        // 左边界延长线的参考起始行
  int ytemp_W_R = 0;        // 右边界延长线的参考起始行
  ExtenRFlag = 0;
  ExtenLFlag = 0;
  ImageStatus.Left_Line = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.Right_Line = 0;

  for (Ysite = 54; Ysite > ImageStatus.OFFLine; Ysite--)
  {
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2]; // [0]=左跳变点, [1]=右跳变点

    // ---- 确定右边界搜索区间 ----
    if (ImageStatus.Road_type != Cross_ture)
    {
      IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval; // 默认 ±2
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;
    }
    else
    {
      IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval_Cross; // 十字 ±2
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;
    }
    LimitL(IntervalLow);  // 钳制区间下界 ≥ 1
    LimitH(IntervalHigh); // 钳制区间上界 ≤ 78
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

    // ---- 确定左边界搜索区间 ----
    IntervalLow = ImageDeal[Ysite + 1].LeftBorder - ImageScanInterval;
    IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + ImageScanInterval;
    LimitL(IntervalLow);
    LimitH(IntervalHigh);
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

    // ---- 根据跳变点类型设置当前行边界 ----
    if (JumpPoint[0].type == 'W')
    {
      ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].LeftBorder; // 左边界丢失，保持上一行值
    }
    else
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;
    }
    if (JumpPoint[1].type == 'W')
    {
      ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].RightBorder; // 右边界丢失，保持上一行值
    }
    else
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;
    }
    ImageDeal[Ysite].IsLeftFind = JumpPoint[0].type;
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    // ---- 处理 'H'（未找到）类型的兜底扫描 ----
    if ((ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H'))
    {
      // 左边界 'H': 在左右边界之间从左向右重新扫描，找黑→白跳变
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1); Xsite++)
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0))
          {
            ImageDeal[Ysite].LeftBorder = Xsite;
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].RightBorder - 1))
          {
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      }
      // 宽度过窄（左右边界间距 ≤ 7）→ 丢线
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <= 7)
      {
        ImageStatus.OFFLine = Ysite + 1;
        break;
      }
      // 右边界 'H': 在左右边界之间从右向左重新扫描，找白→黑跳变
      if (ImageDeal[Ysite].IsRightFind == 'H')
      {
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--)
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
          else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
      }
    }

    int ysite = 0;
    uint8_t L_found_point = 0; // 向上一段距离内找到 'T' 的行数（左边界）
    uint8_t R_found_point = 0; // 向上一段距离内找到 'T' 的行数（右边界）

    // ---- 边界延长线外推（仅非坡道时） ----
    if (ImageStatus.Road_type != Ramp)
    {
      // --- 右边界延长线 ---
      if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite > 10 && Ysite < 50 &&
          ImageStatus.Road_type != Barn_in)
      {
        if (Get_R_line == 'F')
        {
          Get_R_line = 'T';
          ytemp_W_R = Ysite + 2; // 延长线参考起始行（当前位置上方2行）
          // 向上统计接下来 15 行内连续找到 'T' 的次数
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
          {
            if (ImageDeal[ysite].IsRightFind == 'T')
              R_found_point++;
          }
          // 如果上方有足够多的有效边界点 (>8)，计算延长线斜率
          if (R_found_point > 8)
          {
            D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder -
                           ImageDeal[Ysite + 3].RightBorder)) /
                  ((float)(R_found_point - 3));
            if (D_R > 0)
            {
              R_Found_T = 'T'; // 斜率正向（赛道正常变宽），可用
            }
            else
            {
              R_Found_T = 'F'; // 斜率非正向，不延长
              if (D_R < 0)
                ExtenRFlag = 'F';
            }
          }
        }
        // 使用已计算的斜率延长右边界
        if (R_Found_T == 'T')
          ImageDeal[Ysite].RightBorder =
              ImageDeal[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
        LimitL(ImageDeal[Ysite].RightBorder); // 钳制 ≥ 1
        LimitH(ImageDeal[Ysite].RightBorder); // 钳制 ≤ 78
      }
      // --- 左边界延长线（同上逻辑，仅左右和 R/L 互换） ---
      if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&
          ImageStatus.Road_type != Barn_in)
      {
        if (Get_L_line == 'F')
        {
          Get_L_line = 'T';
          ytemp_W_L = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++)
          {
            if (ImageDeal[ysite].IsLeftFind == 'T')
              L_found_point++;
          }
          if (L_found_point > 8)
          {
            D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -
                           ImageDeal[Ysite + L_found_point].LeftBorder)) /
                  ((float)(L_found_point - 3));
            if (D_L > 0)
            {
              L_Found_T = 'T';
            }
            else
            {
              L_Found_T = 'F';
              if (D_L < 0)
                ExtenLFlag = 'F';
            }
          }
        }
        if (L_Found_T == 'T')
          ImageDeal[Ysite].LeftBorder =
              ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);
        LimitL(ImageDeal[Ysite].LeftBorder);
        LimitH(ImageDeal[Ysite].LeftBorder);
      }
    }

    // ---- 丢线统计（用于元素/环岛识别） ----
    if (ImageDeal[Ysite].IsLeftFind == 'W' && ImageDeal[Ysite].IsRightFind == 'W')
    {
      ImageStatus.WhiteLine++; // 左右同时丢线
    }
    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite < 55)
    {
      ImageStatus.Left_Line++; // 左侧丢线（环岛判断依据）
    }
    if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite < 55)
    {
      ImageStatus.Right_Line++; // 右侧丢线（环岛判断依据）
    }

    // 钳制边界值在有效范围 [1, 78] 内
    LimitL(ImageDeal[Ysite].LeftBorder);
    LimitH(ImageDeal[Ysite].LeftBorder);
    LimitL(ImageDeal[Ysite].RightBorder);
    LimitH(ImageDeal[Ysite].RightBorder);

    // 计算当前行赛道宽度和中心
    ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    // ---- 丢线条件判断 ----
    if (ImageDeal[Ysite].Wide <= 7)
    {
      ImageStatus.OFFLine = Ysite + 1; // 赛道宽度 ≤ 7 → 丢线
      break;
    }
    else if (ImageDeal[Ysite].RightBorder <= 10 ||
             ImageDeal[Ysite].LeftBorder >= 70)
    {
      ImageStatus.OFFLine = Ysite + 1; // 边界跑出图像范围 → 丢线
      break;
    }
  }
}

/*
 * DrawExtensionLine - 对 DrawLinesProcess 中标记为 'W'（丢失）的边界进行线性外推延长
 *
 * 功能:
 *   DrawLinesProcess 中当某一侧边界丢失（'W'）时保持了上一行的值不变，
 *   本函数在丢失区域的上下两端找到可靠的 'T' 边界点，用线性插值重新填充中间行。
 *
 * 处理条件:
 *   - 非 Barn_in（车库入库）、非 Ramp（坡道）、非环岛区域
 *   - ExtenLFlag / ExtenRFlag 不为 'F'（不能被延长）时跳过
 *   - WhiteLine（左右同时丢线）数量足够多（≥ TowPoint_True - 15）时才触发
 *
 * 左边界延长处理:
 *   1. 从第 54 行向下扫描，找到第一个 'W' 行
 *   2. 继续向下搜索连续的 3 个 'T' 行（FTSite）作为终点参考
 *   3. 计算斜率 DetL = (LeftBorder[FTSite] - LeftBorder[TFSite]) / (FTSite - TFSite)
 *   4. 用斜率线性插值填充 TFSite 到 FTSite 之间的所有行
 *
 * 右边界延长处理:
 *   对右边界执行相同的逻辑（上述 Left/Right、DetL/DetR 互换）。
 *
 * 最后:
 *   重新计算所有行（从第 59 行到 OFFLine）的 Center 和 Wide。
 */
static void DrawExtensionLine(void)
{
  // 非特殊赛道类型且非环岛才进行延长线处理
  if ((ImageStatus.Road_type != Barn_in && ImageStatus.Road_type != Ramp) &&
      ImageStatus.Road_type != LeftCirque &&
      ImageStatus.Road_type != RightCirque)
  {
    // ---- 左边界延长 ----
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    if (ExtenLFlag != 'F')
    {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
      {
        PicTemp = Pixle[Ysite];
        if (ImageDeal[Ysite].IsLeftFind == 'W')
        {
          // 如果上一行左边界已经 ≥ 70（跑到图像右侧），直接丢线
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;
          }
          // 从 'W' 行继续向下搜索，找到连续 3 个 'T' 行作为终点
          while (Ysite >= (ImageStatus.OFFLine + 4))
          {
            Ysite--;
            if (ImageDeal[Ysite].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 1].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].LeftBorder > 0 &&
                ImageDeal[Ysite - 2].LeftBorder < 70)
            {
              FTSite = Ysite - 2; // 终点行
              break;
            }
          }
          // 计算延长线斜率: (终点边界 - 起点边界) / (终点行号 - 起点行号)
          DetL = ((float)(ImageDeal[FTSite].LeftBorder - ImageDeal[TFSite].LeftBorder)) /
                 ((float)(FTSite - TFSite));
          // 线性插值填充丢失行
          if (FTSite > ImageStatus.OFFLine)
          {
            for (ytemp = TFSite; ytemp >= FTSite; ytemp--)
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite].LeftBorder;
            }
          }
        }
        else
          TFSite = Ysite + 2; // 当前行不是 'W'，更新起点行
      }
    }

    // ---- 右边界延长 ----
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55; // 右环岛脱离状态下强制使用行 55 为起点

    if (ExtenRFlag != 'F')
    {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)
      {
        PicTemp = Pixle[Ysite];
        if (ImageDeal[Ysite].IsRightFind == 'W')
        {
          // 如果上一行右边界已经 ≤ 10（跑到图像左侧），直接丢线
          if (ImageDeal[Ysite + 1].RightBorder <= 10)
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;
          }
          // 向下搜索连续 3 个 'T' 行作为终点
          while (Ysite >= (ImageStatus.OFFLine + 4))
          {
            Ysite--;
            if (ImageDeal[Ysite].IsRightFind == 'T' &&
                ImageDeal[Ysite - 1].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].RightBorder < 70 &&
                ImageDeal[Ysite - 2].RightBorder > 10)
            {
              FTSite = Ysite - 2; // 终点行
              break;
            }
          }
          // 计算延长线斜率
          DetR = ((float)(ImageDeal[FTSite].RightBorder -
                          ImageDeal[TFSite].RightBorder)) /
                 ((float)(FTSite - TFSite));
          // 线性插值填充
          if (FTSite > ImageStatus.OFFLine)
          {
            for (ytemp = TFSite; ytemp >= FTSite; ytemp--)
            {
              ImageDeal[ytemp].RightBorder =
                  (int)(DetR * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite].RightBorder;
            }
          }
        }
        else
          TFSite = Ysite + 2; // 当前行不是 'W'，更新起点行
      }
    }
  }

  // 重新计算所有有效行的 Center 和 Wide
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
  {
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
    ImageDeal[Ysite].Wide =
        -ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder;
  }
}

/*
 * Search_Bottom_Line_OTSU - 在大津法二值化结果中搜索指定行的赛道左右边界
 *
 * 参数:
 *   imageInput  - 二值化图像（Pixle）
 *   Row         - 图像行数（60）
 *   Col         - 图像列数（80）
 *   Bottonline  - 需要搜索的目标行号
 *
 * 功能:
 *   以图像中心 Col/2 为起点，向左右两侧搜索黑白跳变沿来确定赛道边界:
 *   - 左边界: 从左向右扫描，寻找 黑(1)→白(0) 的下降沿
 *   - 右边界: 从右向左扫描，寻找 白(0)→黑(1) 的上升沿
 *   结果存入 ImageDeal[Bottonline].LeftBoundary / .RightBoundary
 *
 * 此函数为 Search_Border_OTSU 的底部边界初始化步骤。
 */
void Search_Bottom_Line_OTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row,
                             uint8_t Col, uint8_t Bottonline)
{
  // 搜索左边界: 从中心向左扫描
  for (int Xsite = Col / 2 - 2; Xsite > 1; Xsite--)
  {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite - 1] == 0)
    {
      ImageDeal[Bottonline].LeftBoundary = Xsite; // 黑→白的跳变点
      break;
    }
  }
  // 搜索右边界: 从中心向右扫描
  for (int Xsite = Col / 2 + 2; Xsite < LCDW - 1; Xsite++)
  {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite + 1] == 0)
    {
      ImageDeal[Bottonline].RightBoundary = Xsite; // 白→黑的跳变点
      break;
    }
  }
}

void Search_Left_and_Right_Lines(uint8_t imageInput[LCDH][LCDW], uint8_t Row,
                                 uint8_t Col, uint8_t Bottonline)
{
  int Left_Rule[2][8] = {
      {0, -1, 1, 0, 0, 1, -1, 0},
      {-1, -1, 1, -1, 1, 1, -1, 1}};
  int Right_Rule[2][8] = {
      {0, -1, 1, 0, 0, 1, -1, 0},
      {1, -1, 1, 1, -1, 1, -1, -1}};
  int num = 0;
  uint8_t Left_Ysite = Bottonline;
  uint8_t Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
  uint8_t Left_Rirection = 0;
  uint8_t Pixel_Left_Ysite = Bottonline;
  uint8_t Pixel_Left_Xsite = 0;

  uint8_t Right_Ysite = Bottonline;
  uint8_t Right_Xsite = ImageDeal[Bottonline].RightBoundary;
  uint8_t Right_Rirection = 0;
  uint8_t Pixel_Right_Ysite = Bottonline;
  uint8_t Pixel_Right_Xsite = 0;
  uint8_t YsiteLocal = Bottonline;
  ImageStatus.OFFLineBoundary = 5;
  while (1)
  {
    num++;
    if (num > 400)
    {
      ImageStatus.OFFLineBoundary = YsiteLocal;
      break;
    }
    if (YsiteLocal >= Pixel_Left_Ysite && YsiteLocal >= Pixel_Right_Ysite)
    {
      if (YsiteLocal < ImageStatus.OFFLineBoundary)
      {
        ImageStatus.OFFLineBoundary = YsiteLocal;
        break;
      }
      else
      {
        YsiteLocal--;
      }
    }
    if ((Pixel_Left_Ysite > YsiteLocal) ||
        YsiteLocal == ImageStatus.OFFLineBoundary)
    {
      Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
      Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
      if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)
      {
        if (Left_Rirection == 3)
          Left_Rirection = 0;
        else
          Left_Rirection++;
      }
      else
      {
        Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
        Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
        if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)
        {
          Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
          Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
          if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
          {
            ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
            ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
          }
        }
        else
        {
          Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
          Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
          if (ImageDeal[Left_Ysite].LeftBoundary_First == 0)
            ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
          ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
          if (Left_Rirection == 0)
            Left_Rirection = 3;
          else
            Left_Rirection--;
        }
      }
    }
    if ((Pixel_Right_Ysite > YsiteLocal) ||
        YsiteLocal == ImageStatus.OFFLineBoundary)
    {
      Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
      Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
      if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)
      {
        if (Right_Rirection == 0)
          Right_Rirection = 3;
        else
          Right_Rirection--;
      }
      else
      {
        Pixel_Right_Ysite =
            Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
        Pixel_Right_Xsite =
            Right_Xsite + Right_Rule[1][2 * Right_Rirection];
        if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)
        {
          Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
          Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
          if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
            ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
          ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
        }
        else
        {
          Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
          Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
          if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
            ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
          ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
          if (Right_Rirection == 3)
            Right_Rirection = 0;
          else
            Right_Rirection++;
        }
      }
    }
    if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)
    {
      ImageStatus.OFFLineBoundary = YsiteLocal;
      break;
    }
  }
}

void Search_Border_OTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row, uint8_t Col,
                        uint8_t Bottonline)
{
  ImageStatus.WhiteLine_L = 0;
  ImageStatus.WhiteLine_R = 0;
  for (int Xsite = 0; Xsite < LCDW; Xsite++)
  {
    imageInput[0][Xsite] = 0;
    imageInput[Bottonline + 1][Xsite] = 0;
  }
  for (int Ysite = 0; Ysite < LCDH; Ysite++)
  {
    ImageDeal[Ysite].LeftBoundary_First = 0;
    ImageDeal[Ysite].RightBoundary_First = 79;
    imageInput[Ysite][0] = 0;
    imageInput[Ysite][LCDW - 1] = 0;
  }
  Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
  Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
  for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
  {
    if (ImageDeal[Ysite].LeftBoundary < 3)
    {
      ImageStatus.WhiteLine_L++;
    }
    if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
    {
      ImageStatus.WhiteLine_R++;
    }
  }
}

/*
 * Element_Test - 赛道元素（环岛、十字）检测入口
 *
 * 功能:
 *   根据全局开关（g_circle_identify_en / g_across_identify_en）和当前赛道状态，
 *   决定是否调用环岛/元素判断函数。
 *
 * 逻辑:
 *   - 若两个开关都关闭 → 直接返回，不做任何检测
 *   - 右环岛和左环岛赛道不进行环岛检测（已在环岛内部，无需重复检测）
 *   - 若 g_circle_identify_en 使能且非环岛路段 → 调用左右环岛判断函数
 *
 * 注意:
 *   Element_Judgment_Left_Rings/Element_Judgment_Right_Rings 在另一个文件中实现，
 *   利用像素跳变数、边界丢失行数（Left_Line/Right_Line）等特征来判断环岛元素。
 */
void Element_Test(void)
{
  if (!g_circle_identify_en && !g_across_identify_en)
    return;
  if (ImageStatus.Road_type != RightCirque &&
      ImageStatus.Road_type != LeftCirque)
  {
    if (g_circle_identify_en)
    {
      Element_Judgment_Left_Rings();   // 左环岛元素判断
      Element_Judgment_Right_Rings();  // 右环岛元素判断
    }
  }
}

/*
 * Element_Handle - 赛道元素处理入口（环岛/圆环的元素处理）
 *
 * 功能:
 *   根据 ImageFlag.image_element_rings 记录的元素类型标志进行分流处理:
 *   - image_element_rings == 1 → 左环岛处理 (Element_Handle_Left_Rings)
 *   - image_element_rings == 2 → 右环岛处理 (Element_Handle_Right_Rings)
 *
 * 注意:
 *   Element_Handle_Left_Rings/Element_Handle_Right_Rings 在另一个文件中实现，
 *   负责环岛元素的边界补线、中心线修正等特殊处理逻辑。
 */
void Element_Handle()
{
  if (ImageFlag.image_element_rings == 1)
    Element_Handle_Left_Rings();   // 左环岛元素处理
  else if (ImageFlag.image_element_rings == 2)
    Element_Handle_Right_Rings();  // 右环岛元素处理
}

/*
 * RouteFilter - 赛道中心线滤波平滑
 *
 * 功能:
 *   对 DrawExtensionLine 之后的中心线进行平滑处理，解决两侧均为白线时的抖动问题。
 *
 * 处理逻辑:
 *   1. 从第 58 行向下扫描到 (OFFLine + 5)
 *   2. 若连续两行（Ysite 和 Ysite-1）左右均为 'W'（白线丢失）且 Ysite ≤ 45:
 *      - 向上搜索到连续出现 'T'（正常边界）的行
 *      - 计算从底部 'W' 区域到上方 'T' 区域的中心线斜率
 *      - 用线性插值重新填充中间所有行的 Center
 *   3. 其余行: 对 Center 执行 EMA 指数平滑
 *      Center_current = (Center_prev + 2 * Center_current) / 3
 *      即 1/3 前一行 + 2/3 当前行的加权平均
 *
 * 效果:
 *   减少中心线跳变，使舵机控制更平滑稳定。
 */
static void RouteFilter(void)
{
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--)
  {
    // 检测到连续两行左右均为白线（边界丢失区域）
    if (ImageDeal[Ysite].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'W' && Ysite <= 45 &&
        ImageDeal[Ysite - 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite - 1].IsRightFind == 'W')
    {
      ytemp = Ysite;
      // 向上搜索找到左右都恢复正常（'T'）的行
      while (ytemp >= (ImageStatus.OFFLine + 5))
      {
        ytemp--;
        if (ImageDeal[ytemp].IsLeftFind == 'T' &&
            ImageDeal[ytemp].IsRightFind == 'T')
        {
          // 计算中心线斜率: 从白线区底部到上方正常区的中心偏移/行差
          DetR = (float)(ImageDeal[ytemp - 1].Center -
                         ImageDeal[Ysite + 2].Center) /
                 (float)(ytemp - 1 - Ysite - 2);
          int CenterTemp = ImageDeal[Ysite + 2].Center;  // 起点中心
          int LineTemp = Ysite + 2;                       // 起点行号
          // 线性插值填充白线区域各行的中心
          while (Ysite >= ytemp)
          {
            ImageDeal[Ysite].Center =
                (int)(CenterTemp + DetR * (float)(Ysite - LineTemp));
            Ysite--;
          }
          break;
        }
      }
    }
    // EMA 平滑: 1/3 前一行 + 2/3 当前行 → 滤除高频抖动
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;
  }
}

/*
 * GetDet - 前瞻点加权平均值计算，得到用于舵机控制的赛道中心偏差
 *
 * 功能:
 *   根据赛道类型和环岛状态动态选择前瞻点行号（TowPoint），
 *   以该行为中心取 ±5 行的中心线 X 坐标进行加权平均。
 *
 * 前瞻点选择策略:
 *   - 左/右环岛（CirqueOff != 'T'）: TowPoint = 30
 *   - 直道:                       TowPoint = 30
 *   - 真十字:                     TowPoint = 22  （更近的前瞻，避免误判）
 *   - 环岛标志激活:               TowPoint = 30
 *   - 默认（普通弯道等）:         TowPoint = 26
 *
 *   钳制: TowPoint 必须在 [OFFLine+1, 49] 范围内
 *
 * 加权平均策略（3 种分支，按可用数据量降级）:
 *   1. TowPoint-5 ≥ OFFLine（数据充足）:
 *      取 [TowPoint-5, TowPoint+5] 共 11 行的中心值加权平均，
 *      权重 Weighting[k] 距前瞻点越近越大。
 *
 *   2. TowPoint > OFFLine 但差不足 5（数据不足）:
 *      非对称取范围 [OFFLine, TowPoint + (TowPoint-OFFLine)] 加权平均。
 *
 *   3. OFFLine < 49（只剩底部少量有效行）:
 *      仅取 [OFFLine, OFFLine+3] 底部几行加权平均。
 *
 *   4. 兜底（OFFLine ≥ 49，几乎全图无效）:
 *      保持上一帧的 Det_True 不变。
 *
 * 输出:
 *   ImageStatus.Det_True      - 最终偏差值（0~79，单位像素，39.5 为中心）
 *   ImageStatus.TowPoint_True - 实际使用的前瞻点行号
 */
void GetDet()
{
  // ============================================================
  //  目标板 override: 已确认 WEAPON -> 用左边界作为循迹中线
  //                  已确认 MATERIALS -> 用右边界作为循迹中线
  //                  TRAFFIC -> 保持原中线 (直行压过)
  //  直接改写 ImageDeal[].Center, 供后续加权 + UDP 显示使用
  // ============================================================
  if (g_target_override.active &&
      (g_target_override.kind == TargetKind::WEAPON ||
       g_target_override.kind == TargetKind::MATERIALS))
  {
    for (int r = ImageStatus.OFFLine; r < 60; ++r)
    {
      if (g_target_override.kind == TargetKind::WEAPON)
        ImageDeal[r].Center = ImageDeal[r].LeftBorder;
      else
        ImageDeal[r].Center = ImageDeal[r].RightBorder;
    }
  }

  float DetTemp = 0;      // 加权平均累加值（分子）
  int TowPoint = 0;       // 前瞻点行号（动态选择）
  float UnitAll = 0;      // 权重累加值（分母）

  // ---- 根据赛道类型动态选择前瞻点 ----
  if ((ImageStatus.Road_type == RightCirque ||
       ImageStatus.Road_type == LeftCirque) &&
      ImageStatus.CirqueOff == 'F')
    TowPoint = 30;                    // 环岛未脱离，用较远前瞻点
  else if (ImageStatus.Road_type == Straight)
    TowPoint = 30;                    // 直道用较远前瞻点
  else if (ImageStatus.Road_type == Cross_ture)
  {
    TowPoint = 22;                    // 十字路口用较近前瞻点
  }
  else if (ImageFlag.image_element_rings_flag == 1 ||
           ImageFlag.image_element_rings_flag == 2)
  {
    TowPoint = 30;                    // 环岛标志激活用较远前瞻点
  }
  else
    TowPoint = 26;                    // 默认：普通弯道

  // 钳制: 不能低于丢线行，不能超过 49
  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1;
  if (TowPoint >= 49)
    TowPoint = 49;

  // ---- 分支1: 数据充足（±5 行都在有效范围内） ----
  if ((TowPoint - 5) >= ImageStatus.OFFLine)
  {
    // 前瞻点下方 5 行
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++)
    {
      DetTemp = DetTemp +
                Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    // 前瞻点上方 5 行
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--)
    {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);
  }
  // ---- 分支2: 前瞻点下方数据不足（非对称取范围） ----
  else if (TowPoint > ImageStatus.OFFLine)
  {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++)
    {
      DetTemp +=
          Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--)
    {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  }
  // ---- 分支3: 只剩底部少量有效行 ----
  else if (ImageStatus.OFFLine < 49)
  {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--)
    {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp =
        (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);
  }
  // ---- 分支4: 全图几乎无效，保持上一帧偏差值不变 ----
  else
    DetTemp = ImageStatus.Det_True;

  ImageStatus.Det_True = DetTemp;        // 更新最终偏差值（舵机控制输入）
  ImageStatus.TowPoint_True = TowPoint;  // 记录实际使用的前瞻点行号
}

/*
 * ImageProcess - 图像处理主入口（每帧调用一次）
 *
 * 参数:
 *   gray_80x60 - 摄像头采集并缩放后的 80×60 灰度图（CV_8UC1）
 *
 * 功能:
 *   这是整个视觉处理流水线的顶层调度函数，按固定顺序编排以下处理阶段:
 *
 *   阶段1: compressimage     → 将 OpenCV Mat 灰度数据提取到 Image_Use 数组
 *   阶段2: Get01change_dajin  → 大津法 OTSU 自适应二值化，生成 Pixle 二值图
 *   阶段3: DrawLinesFirst     → 初步扫描底部 5 行，建立赛道起始边界
 *   阶段4: DrawLinesProcess   → 逐行向上跟踪左右边界，同时进行丢线检测与延长线外推
 *   阶段5: Search_Border_OTSU → 8 邻域边界跟踪（备份/辅助路径），统计白线计数
 *   阶段6: Element_Test       → 环岛/十字等赛道元素检测判断
 *   阶段7: DrawExtensionLine  → 对丢失的边界进行延长线线性插值填充
 *   阶段8: RouteFilter        → 中心线 EMA 平滑滤波，消除抖动
 *   阶段9: Element_Handle     → 环岛元素的边界补线处理
 *   阶段10: GetDet            → 加权平均计算前瞻点偏差（舵机控制最终输入值）
 *
 * 输出:
 *   所有处理结果写入全局变量 ImageStatus 和 ImageDeal 数组中:
 *   - ImageStatus.Det_True:     舵机控制偏差值（0~79）
 *   - ImageStatus.OFFLine:      赛道有效行号上限
 *   - ImageStatus.Road_type:    赛道类型
 *   - ImageStatus.WhiteLine:    白线统计
 *   - ImageDeal[0..59].Center:  各行的赛道中心 X 坐标
 *   - ImageDeal[0..59].LeftBorder/RightBorder: 各行的左右边界
 */
void ImageProcess(cv::Mat &gray_80x60)
{
  compressimage(gray_80x60);

  // 初始化全局状态: 丢线行=2（几乎全图有效），白线计数清零
  ImageStatus.OFFLine = 2;
  ImageStatus.WhiteLine = 0;

  // 初始化每行的边界数据（覆盖全部 59 行到 OFFLine 即可，但此处初始化全范围）
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--)
  {
    ImageDeal[Ysite].IsLeftFind = 'F';       // 左侧未搜索
    ImageDeal[Ysite].IsRightFind = 'F';      // 右侧未搜索
    ImageDeal[Ysite].LeftBorder = 0;         // 左边界默认 0
    ImageDeal[Ysite].RightBorder = 79;       // 右边界默认 79
    ImageDeal[Ysite].LeftTemp = 0;           // 临时边界清零
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].close_LeftBorder = 0;    // 邻近边界清零
    ImageDeal[Ysite].close_RightBorder = 79;
  }

  // 默认参数设置
  ImageStatus.TowPoint = 26;           // 默认前瞻点行号
  ImageStatus.Threshold_detach = 180;  // OTSU 灰度搜索上限（排除高亮噪声）
  ImageStatus.Threshold_static = 70;   // OTSU 阈值静态下限（暗光兜底）

  // ---- 流水线处理阶段 ----
  Get01change_dajin();                                    // 阶段2: 二值化
  DrawLinesFirst();                                       // 阶段3: 底部边界初始化
  DrawLinesProcess();                                     // 阶段4: 主边界跟踪
  Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);        // 阶段5: 8邻域边界跟踪

  Element_Test();                                         // 阶段6: 元素检测
  DrawExtensionLine();                                    // 阶段7: 延长线补全
  RouteFilter();                                          // 阶段8: 中心线滤波
  Element_Handle();                                       // 阶段9: 元素处理
  GetDet();                                               // 阶段10: 前瞻偏差计算
}
