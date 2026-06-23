#include "image_my_zf.h"

bool g_circle_identify_en = true;
bool g_across_identify_en = true;

int ImageScanInterval = 2;
int ImageScanInterval_Cross = 2;

uint8_t Image_Use[LCDH][LCDW];
uint8_t Pixle[LCDH][LCDW];

static int Ysite = 0, Xsite = 0;
static uint8_t* PicTemp;
static int IntervalLow = 0, IntervalHigh = 0;
static int ytemp = 0;
static int TFSite = 0, FTSite = 0;
static float DetR = 0, DetL = 0;
static int BottomBorderRight = 79;
static int BottomBorderLeft = 0;
static int BottomCenter = 0;

ImageDealDatatypedef ImageDeal[60];
ImageStatustypedef ImageStatus;
ImageStatustypedef ImageData;
ImageFlagtypedef ImageFlag;
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};

uint8_t ExtenLFlag = 0;
uint8_t ExtenRFlag = 0;
uint8_t Ring_Help_Flag = 0;
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite;
int Point_Xsite, Point_Ysite;
int Repair_Point_Xsite, Repair_Point_Ysite;

uint8_t Half_Road_Wide[60] = {
    4,  5,  5,  6,  6,  6,  7,  7,  8,  8,
    9,  9, 10, 10, 10, 11, 12, 12, 13, 13,
   13, 14, 14, 15, 15, 16, 16, 17, 17, 17,
   18, 18, 19, 19, 20, 20, 20, 21, 21, 22,
   23, 23, 23, 24, 24, 25, 25, 25, 26, 26,
   27, 28, 28, 28, 29, 30, 31, 31, 31, 32,
};

uint8_t Half_Bend_Wide[60] = {
   33, 33, 33, 33, 33, 33, 33, 33, 33, 33,
   33, 33, 32, 32, 30, 30, 29, 29, 28, 27,
   28, 27, 27, 26, 26, 25, 25, 24, 24, 23,
   22, 21, 21, 22, 22, 22, 23, 24, 24, 24,
   25, 25, 25, 26, 26, 26, 27, 27, 28, 28,
   28, 29, 29, 30, 30, 31, 31, 32, 32, 33,
};

float my_abs(float x) {
  if (x < 0) {
    x = -x;
  }
  return x;
}

void compressimage(cv::Mat& gray_80x60) {
  for (int i = 0; i < gray_80x60.rows; i++) {
    for (int j = 0; j < gray_80x60.cols; j++) {
      Image_Use[i][j] = gray_80x60.at<uint8_t>(i, j);
    }
  }
}

uint8_t Threshold_deal(uint8_t* image, uint16_t col, uint16_t row, uint32_t pixel_threshold) {
#define GrayScale 256
  uint16_t width = col;
  uint16_t height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8_t threshold = 0;
  uint8_t* data = image;
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }
  uint32_t gray_sum = 0;
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      pixelCount[(int)data[i * width + j]]++;
      gray_sum += (int)data[i * width + j];
    }
  }
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; (uint32_t)j < pixel_threshold; j++) {
    w0 += pixelPro[j];
    u0tmp += j * pixelPro[j];
    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;
    u0 = u0tmp / w0;
    u1 = u1tmp / w1;
    u = u0tmp + u1tmp;
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) break;
  }
  return threshold;
}

void Get01change_dajin() {
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8_t i, j = 0;
  uint8_t thre;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65)
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold;
      if (Image_Use[i][j] > (thre))
        Pixle[i][j] = 1;
      else
        Pixle[i][j] = 0;
    }
  }
}

void Pixle_Filter() {
  int nr;
  int nc;
  for (nr = 10; nr < 40; nr++) {
    for (nc = 10; nc < 70; nc = nc + 1) {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                                       Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >=
                                   3)) {
        Pixle[nr][nc] = 1;
      }
    }
  }
}

void GetJumpPointFromDet(uint8_t* p, uint8_t type, int L, int H, JumpPointtypedef* Q) {
  int i = 0;
  if (type == 'L') {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1) {
        Q->point = i;
        Q->type = 'T';
        break;
      } else if (i == (L + 1)) {
        if (*(p + (L + H) / 2) != 0) {
          Q->point = (L + H) / 2;
          Q->type = 'W';
          break;
        } else {
          Q->point = H;
          Q->type = 'H';
          break;
        }
      }
    }
  } else if (type == 'R') {
    for (i = L; i <= H; i++) {
      if (*(p + i) == 1 && *(p + i + 1) != 1) {
        Q->point = i;
        Q->type = 'T';
        break;
      } else if (i == (H - 1)) {
        if (*(p + (L + H) / 2) != 0) {
          Q->point = (L + H) / 2;
          Q->type = 'W';
          break;
        } else {
          Q->point = L;
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

static uint8_t DrawLinesFirst(void) {
  PicTemp = Pixle[59];
  if (*(PicTemp + ImageSensorMid) == 0) {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++) {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)
        break;
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }
    if (*(PicTemp + ImageSensorMid - Xsite) != 0) {
      BottomBorderRight = ImageSensorMid - Xsite + 1;
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--) {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite - 1) == 0) {
          BottomBorderLeft = Xsite;
          break;
        } else if (Xsite == 1) {
          BottomBorderLeft = 0;
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0) {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++) {
        if (*(PicTemp + Xsite) == 0 && *(PicTemp + Xsite + 1) == 0) {
          BottomBorderRight = Xsite;
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79;
          break;
        }
      }
    }
  } else {
    for (Xsite = 79; Xsite > ImageSensorMid; Xsite--) {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        BottomBorderRight = Xsite;
        break;
      } else if (Xsite == 40) {
        BottomBorderRight = 39;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++) {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        BottomBorderLeft = Xsite;
        break;
      } else if (Xsite == 38) {
        BottomBorderLeft = 39;
        break;
      }
    }
  }
  BottomCenter = (BottomBorderLeft + BottomBorderRight) / 2;
  ImageDeal[59].LeftBorder = BottomBorderLeft;
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--) {
    PicTemp = Pixle[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center; Xsite--) {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center + 1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center; Xsite++) {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center - 1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
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

static void DrawLinesProcess(void) {
  uint8_t L_Found_T = 'F';
  uint8_t Get_L_line = 'F';
  uint8_t R_Found_T = 'F';
  uint8_t Get_R_line = 'F';
  float D_L = 0;
  float D_R = 0;
  int ytemp_W_L = 0;
  int ytemp_W_R = 0;
  ExtenRFlag = 0;
  ExtenLFlag = 0;
  ImageStatus.Left_Line = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.Right_Line = 0;

  for (Ysite = 54; Ysite > ImageStatus.OFFLine; Ysite--) {
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];

    if (ImageStatus.Road_type != Cross_ture) {
      IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval;
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;
    } else {
      IntervalLow = ImageDeal[Ysite + 1].RightBorder - ImageScanInterval_Cross;
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;
    }
    LimitL(IntervalLow);
    LimitH(IntervalHigh);
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh, &JumpPoint[1]);

    IntervalLow = ImageDeal[Ysite + 1].LeftBorder - ImageScanInterval;
    IntervalHigh = ImageDeal[Ysite + 1].LeftBorder + ImageScanInterval;
    LimitL(IntervalLow);
    LimitH(IntervalHigh);
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh, &JumpPoint[0]);

    if (JumpPoint[0].type == 'W') {
      ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].LeftBorder;
    } else {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;
    }
    if (JumpPoint[1].type == 'W') {
      ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].RightBorder;
    } else {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;
    }
    ImageDeal[Ysite].IsLeftFind = JumpPoint[0].type;
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    if ((ImageDeal[Ysite].IsLeftFind == 'H' || ImageDeal[Ysite].IsRightFind == 'H')) {
      if (ImageDeal[Ysite].IsLeftFind == 'H') {
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1); Xsite++) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder = Xsite;
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].RightBorder - 1)) {
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <= 7) {
        ImageStatus.OFFLine = Ysite + 1;
        break;
      }
      if (ImageDeal[Ysite].IsRightFind == 'H') {
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0)) {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1)) {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
      }
    }

    int ysite = 0;
    uint8_t L_found_point = 0;
    uint8_t R_found_point = 0;

    if (ImageStatus.Road_type != Ramp) {
      if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite > 10 && Ysite < 50 &&
          ImageStatus.Road_type != Barn_in) {
        if (Get_R_line == 'F') {
          Get_R_line = 'T';
          ytemp_W_R = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
            if (ImageDeal[ysite].IsRightFind == 'T') R_found_point++;
          }
          if (R_found_point > 8) {
            D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder -
                           ImageDeal[Ysite + 3].RightBorder)) /
                  ((float)(R_found_point - 3));
            if (D_R > 0) {
              R_Found_T = 'T';
            } else {
              R_Found_T = 'F';
              if (D_R < 0) ExtenRFlag = 'F';
            }
          }
        }
        if (R_Found_T == 'T')
          ImageDeal[Ysite].RightBorder =
              ImageDeal[ytemp_W_R].RightBorder - D_R * (ytemp_W_R - Ysite);
        LimitL(ImageDeal[Ysite].RightBorder);
        LimitH(ImageDeal[Ysite].RightBorder);
      }
      if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&
          ImageStatus.Road_type != Barn_in) {
        if (Get_L_line == 'F') {
          Get_L_line = 'T';
          ytemp_W_L = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
            if (ImageDeal[ysite].IsLeftFind == 'T')
              L_found_point++;
          }
          if (L_found_point > 8) {
            D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -
                           ImageDeal[Ysite + L_found_point].LeftBorder)) /
                  ((float)(L_found_point - 3));
            if (D_L > 0) {
              L_Found_T = 'T';
            } else {
              L_Found_T = 'F';
              if (D_L < 0) ExtenLFlag = 'F';
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

    if (ImageDeal[Ysite].IsLeftFind == 'W' && ImageDeal[Ysite].IsRightFind == 'W') {
      ImageStatus.WhiteLine++;
    }
    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite < 55) {
      ImageStatus.Left_Line++;
    }
    if (ImageDeal[Ysite].IsRightFind == 'W' && Ysite < 55) {
      ImageStatus.Right_Line++;
    }

    LimitL(ImageDeal[Ysite].LeftBorder);
    LimitH(ImageDeal[Ysite].LeftBorder);
    LimitL(ImageDeal[Ysite].RightBorder);
    LimitH(ImageDeal[Ysite].RightBorder);

    ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7) {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    } else if (ImageDeal[Ysite].RightBorder <= 10 ||
               ImageDeal[Ysite].LeftBorder >= 70) {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }
  }
}

static void DrawExtensionLine(void) {
  if ((ImageStatus.Road_type != Barn_in && ImageStatus.Road_type != Ramp) &&
      ImageStatus.Road_type != LeftCirque &&
      ImageStatus.Road_type != RightCirque) {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15) TFSite = 55;
    if (ExtenLFlag != 'F') {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--) {
        PicTemp = Pixle[Ysite];
        if (ImageDeal[Ysite].IsLeftFind == 'W') {
          if (ImageDeal[Ysite + 1].LeftBorder >= 70) {
            ImageStatus.OFFLine = Ysite + 1;
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4)) {
            Ysite--;
            if (ImageDeal[Ysite].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 1].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].IsLeftFind == 'T' &&
                ImageDeal[Ysite - 2].LeftBorder > 0 &&
                ImageDeal[Ysite - 2].LeftBorder < 70) {
              FTSite = Ysite - 2;
              break;
            }
          }
          DetL = ((float)(ImageDeal[FTSite].LeftBorder - ImageDeal[TFSite].LeftBorder)) /
                 ((float)(FTSite - TFSite));
          if (FTSite > ImageStatus.OFFLine) {
            for (ytemp = TFSite; ytemp >= FTSite; ytemp--) {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite].LeftBorder;
            }
          }
        } else
          TFSite = Ysite + 2;
      }
    }

    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15) TFSite = 55;
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;

    if (ExtenRFlag != 'F') {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--) {
        PicTemp = Pixle[Ysite];
        if (ImageDeal[Ysite].IsRightFind == 'W') {
          if (ImageDeal[Ysite + 1].RightBorder <= 10) {
            ImageStatus.OFFLine = Ysite + 1;
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4)) {
            Ysite--;
            if (ImageDeal[Ysite].IsRightFind == 'T' &&
                ImageDeal[Ysite - 1].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].IsRightFind == 'T' &&
                ImageDeal[Ysite - 2].RightBorder < 70 &&
                ImageDeal[Ysite - 2].RightBorder > 10) {
              FTSite = Ysite - 2;
              break;
            }
          }
          DetR = ((float)(ImageDeal[FTSite].RightBorder -
                          ImageDeal[TFSite].RightBorder)) /
                 ((float)(FTSite - TFSite));
          if (FTSite > ImageStatus.OFFLine) {
            for (ytemp = TFSite; ytemp >= FTSite; ytemp--) {
              ImageDeal[ytemp].RightBorder =
                  (int)(DetR * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite].RightBorder;
            }
          }
        } else
          TFSite = Ysite + 2;
      }
    }
  }

  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
    ImageDeal[Ysite].Wide =
        -ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder;
  }
}

void Search_Bottom_Line_OTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row,
                             uint8_t Col, uint8_t Bottonline) {
  for (int Xsite = Col / 2 - 2; Xsite > 1; Xsite--) {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite - 1] == 0) {
      ImageDeal[Bottonline].LeftBoundary = Xsite;
      break;
    }
  }
  for (int Xsite = Col / 2 + 2; Xsite < LCDW - 1; Xsite++) {
    if (imageInput[Bottonline][Xsite] == 1 &&
        imageInput[Bottonline][Xsite + 1] == 0) {
      ImageDeal[Bottonline].RightBoundary = Xsite;
      break;
    }
  }
}

void Search_Left_and_Right_Lines(uint8_t imageInput[LCDH][LCDW], uint8_t Row,
                                 uint8_t Col, uint8_t Bottonline) {
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
  while (1) {
    num++;
    if (num > 400) {
      ImageStatus.OFFLineBoundary = YsiteLocal;
      break;
    }
    if (YsiteLocal >= Pixel_Left_Ysite && YsiteLocal >= Pixel_Right_Ysite) {
      if (YsiteLocal < ImageStatus.OFFLineBoundary) {
        ImageStatus.OFFLineBoundary = YsiteLocal;
        break;
      } else {
        YsiteLocal--;
      }
    }
    if ((Pixel_Left_Ysite > YsiteLocal) ||
        YsiteLocal == ImageStatus.OFFLineBoundary) {
      Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
      Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
      if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) {
        if (Left_Rirection == 3)
          Left_Rirection = 0;
        else
          Left_Rirection++;
      } else {
        Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
        Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
        if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0) {
          Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
          Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
          if (ImageDeal[Left_Ysite].LeftBoundary_First == 0) {
            ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
            ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
          }
        } else {
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
        YsiteLocal == ImageStatus.OFFLineBoundary) {
      Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
      Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
      if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0) {
        if (Right_Rirection == 0)
          Right_Rirection = 3;
        else
          Right_Rirection--;
      } else {
        Pixel_Right_Ysite =
            Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
        Pixel_Right_Xsite =
            Right_Xsite + Right_Rule[1][2 * Right_Rirection];
        if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0) {
          Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
          Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
          if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
            ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
          ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
        } else {
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
    if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3) {
      ImageStatus.OFFLineBoundary = YsiteLocal;
      break;
    }
  }
}

void Search_Border_OTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row, uint8_t Col,
                        uint8_t Bottonline) {
  ImageStatus.WhiteLine_L = 0;
  ImageStatus.WhiteLine_R = 0;
  for (int Xsite = 0; Xsite < LCDW; Xsite++) {
    imageInput[0][Xsite] = 0;
    imageInput[Bottonline + 1][Xsite] = 0;
  }
  for (int Ysite = 0; Ysite < LCDH; Ysite++) {
    ImageDeal[Ysite].LeftBoundary_First = 0;
    ImageDeal[Ysite].RightBoundary_First = 79;
    imageInput[Ysite][0] = 0;
    imageInput[Ysite][LCDW - 1] = 0;
  }
  Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
  Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
  for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--) {
    if (ImageDeal[Ysite].LeftBoundary < 3) {
      ImageStatus.WhiteLine_L++;
    }
    if (ImageDeal[Ysite].RightBoundary > LCDW - 3) {
      ImageStatus.WhiteLine_R++;
    }
  }
}

void Element_Judgment_Left_Rings() {
  if (ImageStatus.Right_Line > 8 || ImageStatus.Left_Line < 13 ||
      ImageStatus.OFFLine > 8 || ImageStatus.WhiteLine > 3 ||
      ImageDeal[55].IsLeftFind == 'W' || ImageDeal[56].IsLeftFind == 'W' ||
      ImageDeal[57].IsLeftFind == 'W' || ImageDeal[58].IsLeftFind == 'W')
    return;
  int ring_ysite = 25;
  Left_RingsFlag_Point1_Ysite = 0;
  Left_RingsFlag_Point2_Ysite = 0;
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].LeftBoundary_First -
            ImageDeal[Ysite - 1].LeftBoundary_First >
        4) {
      Left_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4) {
      Left_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine;
       Ysite--) {
    if (ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
        ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder) {
      Ring_Help_Flag = 1;
      break;
    }
  }
  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 0) {
    if (ImageStatus.Left_Line > 7) Ring_Help_Flag = 1;
  }
  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0) {
    ImageFlag.image_element_rings = 1;
    ImageFlag.image_element_rings_flag = 1;
    ImageFlag.ring_big_small = 1;
    ImageStatus.Road_type = LeftCirque;
  }
  Ring_Help_Flag = 0;
}

void Element_Judgment_Right_Rings() {
  if (ImageStatus.Left_Line > 8 || ImageStatus.Right_Line < 13 ||
      ImageStatus.OFFLine > 8 || ImageStatus.WhiteLine > 3 ||
      ImageDeal[53].IsRightFind == 'W' || ImageDeal[54].IsRightFind == 'W' ||
      ImageDeal[55].IsRightFind == 'W' || ImageDeal[56].IsRightFind == 'W' ||
      ImageDeal[57].IsRightFind == 'W' || ImageDeal[58].IsRightFind == 'W')
    return;
  int ring_ysite = 25;
  Right_RingsFlag_Point1_Ysite = 0;
  Right_RingsFlag_Point2_Ysite = 0;
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite - 1].RightBoundary_First -
            ImageDeal[Ysite].RightBoundary_First >
        4) {
      Right_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary >
        4) {
      Right_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }
  for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--) {
    if (ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder) {
      Ring_Help_Flag = 1;
      break;
    }
  }
  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 0) {
    if (ImageStatus.Right_Line > 7) Ring_Help_Flag = 1;
  }
  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0) {
    ImageFlag.image_element_rings = 2;
    ImageFlag.image_element_rings_flag = 1;
    ImageFlag.ring_big_small = 1;
    ImageStatus.Road_type = RightCirque;
  }
  Ring_Help_Flag = 0;
}

void Element_Handle_Left_Rings() {
  int num = 0;
  for (int Ysite = 55; Ysite > 30; Ysite--) {
    if (ImageDeal[Ysite].IsLeftFind == 'W') num++;
    if (ImageDeal[Ysite + 3].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsLeftFind == 'T')
      break;
  }
  if (ImageFlag.image_element_rings_flag == 1 && num > 10) {
    ImageFlag.image_element_rings_flag = 2;
  }
  if (ImageFlag.image_element_rings_flag == 2 && num < 8) {
    ImageFlag.image_element_rings_flag = 5;
  }
  if (ImageFlag.image_element_rings_flag == 5 && ImageStatus.Right_Line > 15) {
    ImageFlag.image_element_rings_flag = 6;
  }
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line < 4) {
    ImageFlag.image_element_rings_flag = 7;
  }
  if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7) {
    Point_Ysite = 0;
    Point_Xsite = 0;
    for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 3].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 3].RightBorder) {
        Point_Xsite = ImageDeal[Ysite].RightBorder;
        Point_Ysite = Ysite;
        break;
      }
    }
    if (Point_Ysite > 20) {
      ImageFlag.image_element_rings_flag = 8;
    }
  }
  if (ImageFlag.image_element_rings_flag == 8) {
    if (ImageStatus.Right_Line < 7 && ImageStatus.OFFLine < 6) {
      ImageFlag.image_element_rings_flag = 9;
    }
  }
  if (ImageFlag.image_element_rings_flag == 9) {
    int num2 = 0;
    for (int Ysite = 45; Ysite > 8; Ysite--) {
      if (ImageDeal[Ysite].IsLeftFind == 'W') num2++;
    }
    if (num2 < 5) {
      ImageStatus.Road_type = Normol;
      ImageFlag.image_element_rings_flag = 0;
      ImageFlag.image_element_rings = 0;
      ImageFlag.ring_big_small = 0;
    }
  }

  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 5;
    }
  }
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Rings = 0;
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          Slope_Rings =
              (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    if (flag_Ysite_1 == 0) {
      for (Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++) {
        if (ImageDeal[Ysite].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 1].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
            abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 2].LeftBorder) >
                10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].LeftBorder;
          ImageStatus.OFFLine = Ysite;
          Slope_Rings =
              (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    if (flag_Ysite_1 != 0) {
      for (Ysite = flag_Ysite_1; Ysite < 60; Ysite++) {
        ImageDeal[Ysite].RightBorder =
            flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center < 4) ImageDeal[Ysite].Center = 4;
      }
      ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;
      for (Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) {
        for (Xsite = ImageDeal[Ysite + 1].RightBorder - 10;
             Xsite < ImageDeal[Ysite + 1].RightBorder + 2; Xsite++) {
          if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0) {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
            if (ImageDeal[Ysite].Center < 4) ImageDeal[Ysite].Center = 4;
            ImageDeal[Ysite].Wide =
                ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
            break;
          }
        }
        if (ImageDeal[Ysite].Wide > 8 &&
            ImageDeal[Ysite].RightBorder < ImageDeal[Ysite + 2].RightBorder) {
          continue;
        } else {
          ImageStatus.OFFLine = Ysite + 2;
          break;
        }
      }
    }
  }
  if (ImageFlag.image_element_rings_flag == 8 &&
      ImageFlag.ring_big_small == 1) {
    Repair_Point_Ysite = 7;
    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) {
      ImageDeal[Ysite].RightBorder =
          ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
      if (ImageDeal[Ysite].RightBorder > 77) {
        ImageDeal[Ysite].RightBorder = 77;
      }
      ImageDeal[Ysite].Center =
          ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
    }
  }
  if (ImageFlag.image_element_rings_flag == 9 ||
      ImageFlag.image_element_rings_flag == 10) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
    }
  }
}

void Element_Handle_Right_Rings() {
  int num = 0;
  for (int Ysite = 55; Ysite > 30; Ysite--) {
    if (ImageDeal[Ysite].IsRightFind == 'W') {
      num++;
    }
    if (ImageDeal[Ysite + 3].IsRightFind == 'W' &&
        ImageDeal[Ysite + 2].IsRightFind == 'W' &&
        ImageDeal[Ysite + 1].IsRightFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'T')
      break;
  }
  if (ImageFlag.image_element_rings_flag == 1 && num > 10) {
    ImageFlag.image_element_rings_flag = 2;
  }
  if (ImageFlag.image_element_rings_flag == 2 && num < 8) {
    ImageFlag.image_element_rings_flag = 5;
  }
  if (ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line > 15) {
    ImageFlag.image_element_rings_flag = 6;
  }
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line < 4) {
    ImageFlag.image_element_rings_flag = 7;
  }
  if (ImageFlag.image_element_rings_flag == 7) {
    Point_Xsite = 0;
    Point_Ysite = 0;
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder) {
        Point_Xsite = ImageDeal[Ysite].LeftBorder;
        Point_Ysite = Ysite;
        break;
      }
    }
    if (Point_Ysite > 18) {
      ImageFlag.image_element_rings_flag = 8;
    } else if (ImageDeal[18].RightBoundary_First -
                   ImageDeal[18].LeftBoundary_First >
               70) {
      ImageFlag.image_element_rings_flag = 8;
    }
  }
  if (ImageFlag.image_element_rings_flag == 8) {
    if (ImageStatus.Left_Line < 5 && ImageStatus.OFFLine < 8) {
      ImageFlag.image_element_rings_flag = 9;
    }
  }
  if (ImageFlag.image_element_rings_flag == 9) {
    int num2 = 0;
    for (int Ysite = 45; Ysite > 10; Ysite--) {
      if (ImageDeal[Ysite].IsRightFind == 'W') {
        num2++;
      }
    }
    if (num2 < 5) {
      ImageStatus.Road_type = Normol;
      ImageFlag.image_element_rings_flag = 0;
      ImageFlag.image_element_rings = 0;
      ImageFlag.ring_big_small = 0;
    }
  }

  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
    }
  }
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Right_Rings = 0;
    for (Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    if (flag_Ysite_1 == 0) {
      for (Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++) {
        if (ImageDeal[Ysite].IsRightFind == 'T' &&
            ImageDeal[Ysite + 1].IsRightFind == 'T' &&
            ImageDeal[Ysite + 2].IsRightFind == 'W' &&
            abs(ImageDeal[Ysite].RightBorder -
                ImageDeal[Ysite + 2].RightBorder) > 10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].RightBorder;
          ImageStatus.OFFLine = Ysite;
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    if (flag_Ysite_1 != 0) {
      for (Ysite = flag_Ysite_1; Ysite < 58; Ysite++) {
        ImageDeal[Ysite].LeftBorder =
            flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center > 79) ImageDeal[Ysite].Center = 79;
      }
      ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;
      for (Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) {
        for (Xsite = ImageDeal[Ysite + 1].LeftBorder + 8;
             Xsite > ImageDeal[Ysite + 1].LeftBorder - 4; Xsite--) {
          if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite - 1] == 0) {
            ImageDeal[Ysite].LeftBorder = Xsite;
            ImageDeal[Ysite].Wide =
                ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
            if (ImageDeal[Ysite].Center > 79) ImageDeal[Ysite].Center = 79;
            if (ImageDeal[Ysite].Center < 5) ImageDeal[Ysite].Center = 5;
            break;
          }
        }
        if (ImageDeal[Ysite].Wide > 8 &&
            ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite + 2].LeftBorder) {
          continue;
        } else {
          ImageStatus.OFFLine = Ysite + 2;
          break;
        }
      }
    }
  }
  if (ImageFlag.image_element_rings_flag == 8) {
    Repair_Point_Ysite = 7;
    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) {
      ImageDeal[Ysite].LeftBorder =
          ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
      if (ImageDeal[Ysite].LeftBorder < 3) {
        ImageDeal[Ysite].LeftBorder = 3;
      }
      ImageDeal[Ysite].Center =
          (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
    }
  }
  if (ImageFlag.image_element_rings_flag == 9) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
    }
  }
}

void Element_Test(void) {
  if (!g_circle_identify_en && !g_across_identify_en) return;
  if (ImageStatus.Road_type != RightCirque &&
      ImageStatus.Road_type != LeftCirque) {
    if (g_circle_identify_en) {
      Element_Judgment_Left_Rings();
      Element_Judgment_Right_Rings();
    }
  }
}

void Element_Handle() {
  if (ImageFlag.image_element_rings == 1)
    Element_Handle_Left_Rings();
  else if (ImageFlag.image_element_rings == 2)
    Element_Handle_Right_Rings();
}

static void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5); Ysite--) {
    if (ImageDeal[Ysite].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'W' && Ysite <= 45 &&
        ImageDeal[Ysite - 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite - 1].IsRightFind == 'W') {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine + 5)) {
        ytemp--;
        if (ImageDeal[ytemp].IsLeftFind == 'T' &&
            ImageDeal[ytemp].IsRightFind == 'T') {
          DetR = (float)(ImageDeal[ytemp - 1].Center -
                         ImageDeal[Ysite + 2].Center) /
                 (float)(ytemp - 1 - Ysite - 2);
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =
                (int)(CenterTemp + DetR * (float)(Ysite - LineTemp));
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) / 3;
  }
}

void GetDet() {
  float DetTemp = 0;
  int TowPoint = 0;
  float SpeedGain = 0;
  float UnitAll = 0;

  if ((ImageStatus.Road_type == RightCirque ||
       ImageStatus.Road_type == LeftCirque) &&
      ImageStatus.CirqueOff == 'F')
    TowPoint = 30;
  else if (ImageStatus.Road_type == Straight)
    TowPoint = 30;
  else if (ImageStatus.Road_type == Cross_ture) {
    TowPoint = 22;
  } else if (ImageFlag.image_element_rings_flag == 1 ||
             ImageFlag.image_element_rings_flag == 2) {
    TowPoint = 30;
  } else
    TowPoint = 26;

  if (TowPoint < ImageStatus.OFFLine) TowPoint = ImageStatus.OFFLine + 1;
  if (TowPoint >= 49) TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) {
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp +
                Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);
  } else if (TowPoint > ImageStatus.OFFLine) {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp +=
          Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp +=
          Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp =
        (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);
  } else
    DetTemp = ImageStatus.Det_True;

  ImageStatus.Det_True = DetTemp;
  ImageStatus.TowPoint_True = TowPoint;
}

void ImageProcess_my_zf(cv::Mat& gray_80x60) {
  compressimage(gray_80x60);
  ImageStatus.OFFLine = 2;
  ImageStatus.WhiteLine = 0;
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].IsLeftFind = 'F';
    ImageDeal[Ysite].IsRightFind = 'F';
    ImageDeal[Ysite].LeftBorder = 0;
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;
    ImageDeal[Ysite].RightTemp = 79;
    ImageDeal[Ysite].close_LeftBorder = 0;
    ImageDeal[Ysite].close_RightBorder = 79;
  }

  ImageStatus.TowPoint = 26;
  ImageStatus.Threshold_detach = 180;
  ImageStatus.Threshold_static = 70;

  Get01change_dajin();
  DrawLinesFirst();
  DrawLinesProcess();
  Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);

  Element_Test();
  DrawExtensionLine();
  RouteFilter();
  Element_Handle();
  GetDet();
}
