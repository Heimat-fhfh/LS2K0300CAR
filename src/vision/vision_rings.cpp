#include "vision/image_my_zf.h"

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
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (int Xsite = ImageDeal[Ysite].LeftBorder + 1;
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
      for (int Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++) {
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
      for (int Ysite = flag_Ysite_1; Ysite < 60; Ysite++) {
        ImageDeal[Ysite].RightBorder =
            flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center < 4) ImageDeal[Ysite].Center = 4;
      }
      ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;
      for (int Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) {
        for (int Xsite = ImageDeal[Ysite + 1].RightBorder - 10;
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
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (int Xsite = ImageDeal[Ysite].LeftBorder + 1;
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
      for (int Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++) {
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
      for (int Ysite = flag_Ysite_1; Ysite < 58; Ysite++) {
        ImageDeal[Ysite].LeftBorder =
            flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center > 79) ImageDeal[Ysite].Center = 79;
      }
      ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;
      for (int Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite--) {
        for (int Xsite = ImageDeal[Ysite + 1].LeftBorder + 8;
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
