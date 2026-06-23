#include "vision/Image_Process.h"

#include "vision/Image_Process.h"

/*
 * Element_Judgment_Left_Rings - 左环岛元素进入判断
 *
 * 功能:
 *   基于大津法二值化和 8 邻域边界跟踪结果，检测是否进入了左环岛（LeftCirque）赛道元素。
 *   环岛判断依据是赛道边界在环岛入口处产生的特征性跳变:
 *     - LeftBoundary_First（左侧首次命中的 X 坐标）在入口处有明显阶跃
 *     - LeftBorder（主要左边线）在入口处向外凸出形成局部极值
 *
 * 前置条件（全部满足才继续）:
 *   1. Right_Line ≤ 8       — 右侧边界丢失不能太多（确保右侧稳定）
 *   2. Left_Line ≥ 13       — 左侧必须有足够的丢线行（环岛左侧白线特征）
 *   3. OFFLine ≤ 8          — 丢线行不能太高（赛道有效区域足够）
 *   4. WhiteLine ≤ 3        — 双侧丢线不能太多
 *   5. 底部 55~58 行的左边界不能全是 'W'（底部左边界必须稳定存在）
 *
 * 检测步骤:
 *   步骤1: 在第 58 行到 ring_ysite(25) 之间扫描 LeftBoundary_First 的阶跃点。
 *          LeftBoundary_First 仅在每行首次命中时写入一次，
 *          若当前行比上一行的 First 坐标大 4+ 像素，记录为 Flag_Point1。
 *
 *   步骤2: 同样区间扫描 LeftBoundary 的阶跃点。
 *          LeftBoundary 在跟踪过程中持续更新，
 *          若下一行比当前行的 Boundary 坐标大 4+ 像素，记录为 Flag_Point2。
 *
 *   步骤3: 从 Flag_Point1 向下扫描，检查 LeftBorder 是否存在"局部凸出最大值"特征
 *          （相邻行对比，中间行 LeftBorder > 上下各 3 行的 LeftBorder），
 *          若有则置 Ring_Help_Flag = 1。
 *
 *   步骤4: 综合判断——若 Flag_Point2 > Flag_Point1+1 且 Ring_Help_Flag==1
 *          且 image_element_rings_flag==0（非已处理状态），则确认左环岛:
 *          - image_element_rings = 1      (标记为左环岛)
 *          - image_element_rings_flag = 1 (状态机进入阶段1)
 *          - ring_big_small = 1           (大环标志)
 *          - Road_type = LeftCirque       (赛道类型为左环岛)
 *
 *   终止清理: Ring_Help_Flag = 0（每次判断后复位辅助标志）
 */
void Element_Judgment_Left_Rings() {
  // ---- 前置条件检查: 不满足任一条件则直接退出 ----
  if (ImageStatus.Right_Line > 8 || ImageStatus.Left_Line < 13 ||
      ImageStatus.OFFLine > 8 || ImageStatus.WhiteLine > 3 ||
      ImageDeal[55].IsLeftFind == 'W' || ImageDeal[56].IsLeftFind == 'W' ||
      ImageDeal[57].IsLeftFind == 'W' || ImageDeal[58].IsLeftFind == 'W')
    return;
  int ring_ysite = 25;                                  // 环岛搜索上限行号（距顶部 25 行）
  Left_RingsFlag_Point1_Ysite = 0;
  Left_RingsFlag_Point2_Ysite = 0;

  // ---- 步骤1: 搜索左边界首次命中点（LeftBoundary_First）的阶跃 ----
  // LeftBoundary_First 只在每行首次命中时写入，能捕捉到环岛入口处的边界突变
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].LeftBoundary_First -
            ImageDeal[Ysite - 1].LeftBoundary_First >
        4) {                                              // 上跳超过 4 像素 → 疑似环岛入口
      Left_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  // ---- 步骤2: 搜索左边界持续跟踪点（LeftBoundary）的阶跃 ----
  // LeftBoundary 在跟踪中持续更新，能捕捉到环岛入口后的连续变化
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4) {
      Left_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }
  // ---- 步骤3: 验证左边界是否存在"局部凸出极值"特征 ----
  // 环岛入口处左边界会向外凸出，形成一个局部最大值
  for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine;
       Ysite--) {
    if (ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
        ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite + 3].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder &&
        ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder) {
      Ring_Help_Flag = 1;                                 // 确认局部凸出特征
      break;
    }
  }
  // ---- 步骤4: 兜底判断——即使未找到局部极值，若左侧丢线足够多也算 ----
  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 0) {
    if (ImageStatus.Left_Line > 7) Ring_Help_Flag = 1;   // 左侧丢线行 > 7 → 放宽条件
  }
  // ---- 步骤5: 确认左环岛并设置状态 ----
  if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0) {
    ImageFlag.image_element_rings = 1;                    // 元素类型: 左环岛
    ImageFlag.image_element_rings_flag = 1;               // 处理状态机: 阶段1
    ImageFlag.ring_big_small = 1;                         // 环岛大小: 大环
    ImageStatus.Road_type = LeftCirque;                   // 赛道类型: 左环岛
  }
  Ring_Help_Flag = 0;  // 复位辅助标志
}

/*
 * Element_Judgment_Right_Rings - 右环岛元素进入判断
 *
 * 功能:
 *   与 Element_Judgment_Left_Rings 镜像对称，检测是否进入了右环岛（RightCirque）赛道元素。
 *   利用 RightBoundary_First 和 RightBoundary 的阶跃特征来判断。
 *
 * 前置条件（与左侧镜像）:
 *   1. Left_Line ≤ 8        — 左侧边界丢失不能太多
 *   2. Right_Line ≥ 13      — 右侧必须有足够的丢线行
 *   3. OFFLine ≤ 8 / WhiteLine ≤ 3 — 丢线不能太高 / 双侧丢线不能太多
 *   4. 底部 53~58 行的右边界不能全是 'W'
 *
 * 检测步骤（与左侧镜像，Left/Right 互换，大小比较方向取反）:
 *   步骤1: 扫描 RightBoundary_First 的阶跃（向上跳变 > 4 像素）
 *   步骤2: 扫描 RightBoundary 的阶跃（向上跳变 > 4 像素）
 *   步骤3: 检查 RightBorder 是否存在"局部凸出最小值"特征（环岛入口处右边界内凹）
 *   步骤4: 兜底判断——右侧丢线 > 7 则放宽条件
 *   步骤5: 确认右环岛并设置标志:
 *          - image_element_rings = 2      (右环岛)
 *          - image_element_rings_flag = 1 (阶段1)
 *          - ring_big_small = 1           (大环)
 *          - Road_type = RightCirque      (赛道类型)
 *
 * 镜像差异说明:
 *   左环岛: 左边线向外凸出（LeftBorder 变大），形成局部 MAX
 *   右环岛: 右边线向内凹陷（RightBorder 变大），形成局部 MIN（检查时仍用 >）
 *   因为坐标系中 X 越大越靠右，左右两侧的变化对称但方向不同。
 */
void Element_Judgment_Right_Rings() {
  // ---- 前置条件检查: 不满足任一条件则直接退出 ----
  if (ImageStatus.Left_Line > 8 || ImageStatus.Right_Line < 13 ||
      ImageStatus.OFFLine > 8 || ImageStatus.WhiteLine > 3 ||
      ImageDeal[53].IsRightFind == 'W' || ImageDeal[54].IsRightFind == 'W' ||
      ImageDeal[55].IsRightFind == 'W' || ImageDeal[56].IsRightFind == 'W' ||
      ImageDeal[57].IsRightFind == 'W' || ImageDeal[58].IsRightFind == 'W')
    return;
  int ring_ysite = 25;
  Right_RingsFlag_Point1_Ysite = 0;
  Right_RingsFlag_Point2_Ysite = 0;

  // ---- 步骤1: 搜索右边界首次命中点（RightBoundary_First）的阶跃 ----
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite - 1].RightBoundary_First -
            ImageDeal[Ysite].RightBoundary_First >
        4) {                                              // 向上跳变 > 4 像素
      Right_RingsFlag_Point1_Ysite = Ysite;
      break;
    }
  }
  // ---- 步骤2: 搜索右边界持续跟踪点（RightBoundary）的阶跃 ----
  for (int Ysite = 58; Ysite > ring_ysite; Ysite--) {
    if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary >
        4) {
      Right_RingsFlag_Point2_Ysite = Ysite;
      break;
    }
  }
  // ---- 步骤3: 验证右边界是否存在"局部内凹极值"特征 ----
  // 右环岛入口处: RightBorder 呈现局部最小值（与左侧局部最大值镜像）
  for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--) {
    if (ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder &&
        ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder) {
      Ring_Help_Flag = 1;                                 // 确认局部内凹特征
      break;
    }
  }
  // ---- 步骤4: 兜底判断——即使未找到局部极值，若右侧丢线足够多也算 ----
  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 0) {
    if (ImageStatus.Right_Line > 7) Ring_Help_Flag = 1;  // 右侧丢线行 > 7 → 放宽条件
  }
  // ---- 步骤5: 确认右环岛并设置状态 ----
  if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite + 1 &&
      Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0) {
    ImageFlag.image_element_rings = 2;                    // 元素类型: 右环岛
    ImageFlag.image_element_rings_flag = 1;               // 处理状态机: 阶段1
    ImageFlag.ring_big_small = 1;                         // 环岛大小: 大环
    ImageStatus.Road_type = RightCirque;                  // 赛道类型: 右环岛
  }
  Ring_Help_Flag = 0;  // 复位辅助标志
}

/*
 * Element_Handle_Left_Rings - 左环岛元素处理状态机
 *
 * 功能:
 *   在检测到进入左环岛后，本函数通过 image_element_rings_flag 状态机管理
 *   环岛处理的各个阶段，在每个阶段进行不同的边界修正和中心线计算。
 *
 * 状态机概览（image_element_rings_flag）:
 *   ┌─────┐   num>10    ┌─────┐   num<8     ┌─────┐  Right_Line>15 ┌─────┐  Right_Line<4 ┌─────┐
 *   │  1  │ ────────→  │  2  │ ────────→  │  5  │ ────────────→ │  6  │ ───────────→ │  7  │
 *   └─────┘            └─────┘            └─────┘               └─────┘              └─────┘
 *      │                  │                                                                 │
 *      ↓                  ↓                                                      ring_big_small==1
 *   阶段1-4:            阶段1-4:                                                   找到局部最小值 → 阶段8
 *   Center =            Center =
 *   RightBorder -       RightBorder -
 *   Half_Road_Wide - 5  Half_Road_Wide - 5                                         ┌─────┐ 条件满足 ┌─────┐
 *                                                                                  │  8  │ ───────→ │  9  │ → 退出环岛
 *   阶段5-6: 在图像中搜索白色像素跳变点，计算右侧边界延长线斜率，用 Half_Bend_Wide 补偿中心线
 *   阶段7:   搜索右边界局部最小值作为环岛出口参考点 (Point_Xsite, Point_Ysite)
 *   阶段8:   底部区域右边界修复（LeftBorder + Half_Bend_Wide）
 *   阶段9:   环岛即将退出，中心线 = RightBorder - Half_Road_Wide → 恢复正常模式
 *
 * 状态转移详细说明:
 *   flag 1: 刚判断进入环岛，开始统计左边界 'W' 行数
 *   flag 1→2: num(左侧'W'行数) > 10 → 确认环岛进入
 *   flag 2→5: num < 8 → 左边界重新可见，进入环岛中段
 *   flag 5→6: Right_Line > 15 → 右侧大量丢线，环岛弯道最深处
 *   flag 6→7: Right_Line < 4 → 右侧重新找到，开始寻找环岛出口
 *   flag 7→8: 找到右边界局部最小值 (Point_Ysite > 20) → 标记出口位置
 *   flag 8→9: Right_Line < 7 且 OFFLine < 6 → 确认接近出口
 *   flag 9: 左侧 'W' 行数 < 5 → 完全退出环岛，恢复 Normol 赛道类型
 *
 * 各阶段中心线策略:
 *   阶段1-4: Center = RightBorder - Half_Road_Wide[Ysite] - 5
 *            （以右边界为基准，用预设直道半宽推算左边界后的中心）
 *
 *   阶段5-6: 搜索白→黑跳变扫描线:
 *            从第 55 行向下，在左右边界之间找到白→黑跳变点 (flag_Xsite_1, flag_Ysite_1)，
 *            计算从该点延伸到图像右下角 (79, 59) 的斜率 Slope_Rings。
 *            若无跳变点，则检查左边界是否在某一位置突然消失（T→W），
 *            并从左下角 (0, 59) 到该消失点计算斜率。
 *            使用该斜率和 Half_Bend_Wide 逐行修正 RightBorder 和 Center。
 *
 *   阶段8:   底部区域 (Ysite > Repair_Point_Ysite-3) 右边界修复:
 *            RightBorder = LeftBorder + Half_Bend_Wide[Ysite]
 *            Center = (LeftBorder + RightBorder) / 2
 *
 *   阶段9:   Center = RightBorder - Half_Road_Wide[Ysite]
 *            （恢复正常模式前的过渡）
 */
void Element_Handle_Left_Rings() {
  // ---- 统计左边界 'W' 行数，同时寻找从 'W' 恢复到 'T' 的转折行 ----
  int num = 0;  // 左边界 'W'（丢失）行计数器
  for (int Ysite = 55; Ysite > 30; Ysite--) {
    if (ImageDeal[Ysite].IsLeftFind == 'W') num++;          // 计数 'W' 行
    // 寻找转折: 连续 3 行 'W' 之后出现 'T' → 到达环岛内部可恢复区域
    if (ImageDeal[Ysite + 3].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
        ImageDeal[Ysite + 1].IsLeftFind == 'W' &&
        ImageDeal[Ysite].IsLeftFind == 'T')
      break;
  }

  // ========== 状态转移逻辑 ==========

  // flag 1→2: 左边界丢失行数 > 10，确认进入环岛深处
  if (ImageFlag.image_element_rings_flag == 1 && num > 10) {
    ImageFlag.image_element_rings_flag = 2;
  }
  // flag 2→5: 左边界重新可见（'W' 行数 < 8），进入环岛中段
  if (ImageFlag.image_element_rings_flag == 2 && num < 8) {
    ImageFlag.image_element_rings_flag = 5;
  }
  // flag 5→6: 右侧大量丢线，环岛弯道最深处
  if (ImageFlag.image_element_rings_flag == 5 && ImageStatus.Right_Line > 15) {
    ImageFlag.image_element_rings_flag = 6;
  }
  // flag 6→7: 右侧重新找到边界，环岛弯道通过，开始寻找出口
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line < 4) {
    ImageFlag.image_element_rings_flag = 7;
  }
  // flag 7→8: 大环模式下，找到右边界局部最小值作为出口参考点
  if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7) {
    Point_Ysite = 0;
    Point_Xsite = 0;
    // 扫描第 50 行到 OFFLine+3，寻找右边界局部最小值
    for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if (ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 3].RightBorder &&
          ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 3].RightBorder) {
        Point_Xsite = ImageDeal[Ysite].RightBorder;  // 出口参考点 X
        Point_Ysite = Ysite;                          // 出口参考点行号
        break;
      }
    }
    if (Point_Ysite > 20) {                             // 参考点位置足够高 → 有效出口
      ImageFlag.image_element_rings_flag = 8;
    }
  }
  // flag 8→9: 确认接近环岛出口（右侧丢线少且赛道有效区域大）
  if (ImageFlag.image_element_rings_flag == 8) {
    if (ImageStatus.Right_Line < 7 && ImageStatus.OFFLine < 6) {
      ImageFlag.image_element_rings_flag = 9;
    }
  }
  // flag 9: 完全退出环岛条件检查
  if (ImageFlag.image_element_rings_flag == 9) {
    int num2 = 0;
    for (int Ysite = 45; Ysite > 8; Ysite--) {          // 统计左边界 'W' 行数
      if (ImageDeal[Ysite].IsLeftFind == 'W') num2++;
    }
    if (num2 < 5) {                                      // 丢线行数 < 5 → 恢复正常赛道
      ImageStatus.Road_type = Normol;                    // 恢复为普通赛道
      ImageFlag.image_element_rings_flag = 0;            // 状态机复位
      ImageFlag.image_element_rings = 0;                 // 元素类型清零
      ImageFlag.ring_big_small = 0;                      // 环岛标志清零
    }
  }

  // ========== 各阶段中心线计算 ==========

  // ---- 阶段1-4: 入环阶段，以右边界为基准推算中心线 ----
  // 左边界在环岛入口处不可靠，因此用右边界减去预设半宽来估算中心
  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 5;
      // Center = 右边界 - 该行直道半宽 - 5（额外偏移补偿）
    }
  }

  // ---- 阶段5-6: 环岛中段，搜索像素跳变点补全边界 ----
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;                                // 扫描到的参考点 X 坐标
    int flag_Ysite_1 = 0;                                // 扫描到的参考点行号
    float Slope_Rings = 0;                               // 环岛边界延长线斜率

    // 方案A: 在第 55 行到 OFFLine 之间搜索白→黑跳变点
    // 在左右边界内部区间扫描，寻找 白(1)→黑(0) 的下降沿
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (int Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          // 计算从该跳变点到右下角 (79, 59) 的扫描线斜率
          Slope_Rings =
              (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    // 方案B: 若方案A未找到跳变点，检查左边界突然消失的特征
    if (flag_Ysite_1 == 0) {
      for (int Ysite = ImageStatus.OFFLine + 1; Ysite < 30; Ysite++) {
        // 找到: 连续至少 2 行左边界是 'T'，然后下一行变成 'W'，且边界突变 > 10 像素
        if (ImageDeal[Ysite].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 1].IsLeftFind == 'T' &&
            ImageDeal[Ysite + 2].IsLeftFind == 'W' &&
            abs(ImageDeal[Ysite].LeftBorder - ImageDeal[Ysite + 2].LeftBorder) >
                10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].LeftBorder;
          ImageStatus.OFFLine = Ysite;                    // 截断有效区域
          // 从左下角 (0, 59) 到该消失点计算斜率（左边界提前消失的特征线）
          Slope_Rings =
              (float)(79 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    // 若找到参考点，使用斜率补全边界
    if (flag_Ysite_1 != 0) {
      // 从参考点向下补全右边界（基于斜率外推 + Half_Bend_Wide 补偿）
      for (int Ysite = flag_Ysite_1; Ysite < 60; Ysite++) {
        ImageDeal[Ysite].RightBorder =
            flag_Xsite_1 + Slope_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center < 4) ImageDeal[Ysite].Center = 4;  // 防止中心跑出图像
      }
      ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;
      // 从参考点向上逐行精搜索右边界（在预估位置附近 ±10 列内搜索白→黑跳变）
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
        // 有效性检查: 宽度 > 8 且右边界在增长 → 继续向上搜索；否则丢线
        if (ImageDeal[Ysite].Wide > 8 &&
            ImageDeal[Ysite].RightBorder < ImageDeal[Ysite + 2].RightBorder) {
          continue;
        } else {
          ImageStatus.OFFLine = Ysite + 2;                // 标记丢线行
          break;
        }
      }
    }
  }

  // ---- 阶段8: 环岛出口，修复底部边界 ----
  if (ImageFlag.image_element_rings_flag == 8 &&
      ImageFlag.ring_big_small == 1) {
    Repair_Point_Ysite = 7;                               // 修复参考行
    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) {
      // 用弯道半宽推算右边界 = 左边界 + Half_Bend_Wide[Ysite]
      ImageDeal[Ysite].RightBorder =
          ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
      if (ImageDeal[Ysite].RightBorder > 77) {
        ImageDeal[Ysite].RightBorder = 77;               // 钳制不超出图像
      }
      ImageDeal[Ysite].Center =
          ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
    }
  }

  // ---- 阶段9-10: 环岛退出，过渡到正常模式 ----
  if (ImageFlag.image_element_rings_flag == 9 ||
      ImageFlag.image_element_rings_flag == 10) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      // 用直道半宽推算中心线
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
    }
  }
}

/*
 * Element_Handle_Right_Rings - 右环岛元素处理状态机
 *
 * 功能:
 *   与 Element_Handle_Left_Rings 镜像对称，管理右环岛处理的各个阶段。
 *   状态转移逻辑和中心线策略与左环岛对应镜像。
 *
 * 状态机概览（image_element_rings_flag）:
 *   ┌─────┐   num>10    ┌─────┐   num<8     ┌─────┐  Left_Line>15 ┌─────┐  Left_Line<4  ┌─────┐
 *   │  1  │ ────────→  │  2  │ ────────→  │  5  │ ───────────→ │  6  │ ───────────→ │  7  │
 *   └─────┘            └─────┘            └─────┘              └─────┘              └─────┘
 *      │                  │                                                                 │
 *      ↓                  ↓                                                      找到局部最大值 → 阶段8
 *   阶段1-4:            阶段1-4:
 *   Center =            Center =                                                    ┌─────┐ 条件满足 ┌─────┐
 *   LeftBorder +        LeftBorder +                                                │  8  │ ───────→ │  9  │ → 退出环岛
 *   Half_Road_Wide      Half_Road_Wide
 *
 *   阶段5-6: 搜索白→黑跳变点，计算左侧边界延长线斜率，用 Half_Bend_Wide 补偿中心线
 *   阶段7:   搜索左边界局部最大值作为出口参考点
 *   阶段8:   底部区域左边界修复（RightBorder - Half_Bend_Wide）
 *   阶段9:   环岛退出过渡，Center = LeftBorder + Half_Road_Wide
 *
 * 镜像差异说明:
 *   左环岛中心线: RightBorder - Half_Road_Wide      (以右边界为基准算左)
 *   右环岛中心线: LeftBorder  + Half_Road_Wide       (以左边界为基准算右)
 *
 *   左环岛阶段7 找右边界局部 MIN → 阶段8
 *   右环岛阶段7 找左边界局部 MAX → 阶段8
 *
 *   左环岛阶段5 斜率: (79 - flag_Xsite_1) / (59 - flag_Ysite_1)  → 参考右下角
 *   右环岛阶段5 斜率: (0  - flag_Xsite_1) / (59 - flag_Ysite_1)  → 参考左下角
 */
void Element_Handle_Right_Rings() {
  // ---- 统计右边界 'W' 行数，同时寻找从 'W' 恢复到 'T' 的转折行 ----
  int num = 0;  // 右边界 'W'（丢失）行计数器
  for (int Ysite = 55; Ysite > 30; Ysite--) {
    if (ImageDeal[Ysite].IsRightFind == 'W') {
      num++;                                                 // 计数 'W' 行
    }
    // 寻找转折: 连续 3 行 'W' 之后出现 'T'
    if (ImageDeal[Ysite + 3].IsRightFind == 'W' &&
        ImageDeal[Ysite + 2].IsRightFind == 'W' &&
        ImageDeal[Ysite + 1].IsRightFind == 'W' &&
        ImageDeal[Ysite].IsRightFind == 'T')
      break;
  }

  // ========== 状态转移逻辑（与左环岛镜像） ==========

  // flag 1→2: 右边界丢失行数 > 10，确认进入环岛深处
  if (ImageFlag.image_element_rings_flag == 1 && num > 10) {
    ImageFlag.image_element_rings_flag = 2;
  }
  // flag 2→5: 右边界重新可见，进入环岛中段
  if (ImageFlag.image_element_rings_flag == 2 && num < 8) {
    ImageFlag.image_element_rings_flag = 5;
  }
  // flag 5→6: 左侧大量丢线，环岛弯道最深处
  if (ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line > 15) {
    ImageFlag.image_element_rings_flag = 6;
  }
  // flag 6→7: 左侧重新找到边界，开始寻找出口
  if (ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line < 4) {
    ImageFlag.image_element_rings_flag = 7;
  }
  // flag 7→8: 找到左边界局部最大值作为出口参考点
  if (ImageFlag.image_element_rings_flag == 7) {
    Point_Xsite = 0;
    Point_Ysite = 0;
    // 扫描第 55 行到 OFFLine+3，寻找左边界局部最大值
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine + 3; Ysite--) {
      if (ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder &&
          ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder) {
        Point_Xsite = ImageDeal[Ysite].LeftBorder;    // 出口参考点 X
        Point_Ysite = Ysite;                           // 出口参考点行号
        break;
      }
    }
    if (Point_Ysite > 18) {                             // 参考点位置足够高 → 有效出口
      ImageFlag.image_element_rings_flag = 8;
    } else if (ImageDeal[18].RightBoundary_First -
                   ImageDeal[18].LeftBoundary_First >
               70) {                                    // 兜底: 行18的赛道宽度 > 70 → 也认为可出口
      ImageFlag.image_element_rings_flag = 8;
    }
  }
  // flag 8→9: 确认接近出口
  if (ImageFlag.image_element_rings_flag == 8) {
    if (ImageStatus.Left_Line < 5 && ImageStatus.OFFLine < 8) {
      ImageFlag.image_element_rings_flag = 9;
    }
  }
  // flag 9: 完全退出环岛条件检查
  if (ImageFlag.image_element_rings_flag == 9) {
    int num2 = 0;
    for (int Ysite = 45; Ysite > 10; Ysite--) {         // 统计右边界 'W' 行数
      if (ImageDeal[Ysite].IsRightFind == 'W') {
        num2++;
      }
    }
    if (num2 < 5) {                                      // 丢线行数 < 5 → 恢复正常赛道
      ImageStatus.Road_type = Normol;
      ImageFlag.image_element_rings_flag = 0;
      ImageFlag.image_element_rings = 0;
      ImageFlag.ring_big_small = 0;
    }
  }

  // ========== 各阶段中心线计算 ==========

  // ---- 阶段1-4: 入环阶段，以左边界为基准推算中心线 ----
  if (ImageFlag.image_element_rings_flag == 1 ||
      ImageFlag.image_element_rings_flag == 2 ||
      ImageFlag.image_element_rings_flag == 3 ||
      ImageFlag.image_element_rings_flag == 4) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
      // Center = 左边界 + 该行直道半宽
    }
  }

  // ---- 阶段5-6: 环岛中段，搜索像素跳变点补全边界 ----
  if (ImageFlag.image_element_rings_flag == 5 ||
      ImageFlag.image_element_rings_flag == 6) {
    int flag_Xsite_1 = 0;
    int flag_Ysite_1 = 0;
    float Slope_Right_Rings = 0;

    // 方案A: 在第 55 行到 OFFLine 之间搜索白→黑跳变点
    for (int Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--) {
      for (int Xsite = ImageDeal[Ysite].LeftBorder + 1;
           Xsite < ImageDeal[Ysite].RightBorder - 1; Xsite++) {
        if (Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = Xsite;
          // 计算从该跳变点到左下角 (0, 59) 的扫描线斜率
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
      if (flag_Ysite_1 != 0) {
        break;
      }
    }
    // 方案B: 检查右边界突然消失的特征
    if (flag_Ysite_1 == 0) {
      for (int Ysite = ImageStatus.OFFLine + 5; Ysite < 30; Ysite++) {
        // 找到: 连续至少 2 行右边界是 'T'，然后下一行变成 'W'，且边界突变 > 10 像素
        if (ImageDeal[Ysite].IsRightFind == 'T' &&
            ImageDeal[Ysite + 1].IsRightFind == 'T' &&
            ImageDeal[Ysite + 2].IsRightFind == 'W' &&
            abs(ImageDeal[Ysite].RightBorder -
                ImageDeal[Ysite + 2].RightBorder) > 10) {
          flag_Ysite_1 = Ysite;
          flag_Xsite_1 = ImageDeal[flag_Ysite_1].RightBorder;
          ImageStatus.OFFLine = Ysite;
          // 从左下角 (0, 59) 到该消失点计算斜率
          Slope_Right_Rings =
              (float)(0 - flag_Xsite_1) / (float)(59 - flag_Ysite_1);
          break;
        }
      }
    }
    // 若找到参考点，使用斜率补全边界
    if (flag_Ysite_1 != 0) {
      // 从参考点向下补全左边界
      for (int Ysite = flag_Ysite_1; Ysite < 58; Ysite++) {
        ImageDeal[Ysite].LeftBorder =
            flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
        if (ImageDeal[Ysite].Center > 79) ImageDeal[Ysite].Center = 79;  // 钳制
      }
      ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;
      // 从参考点向上逐行精搜索左边界（在预估位置附近 ±4~+8 列内搜索白→黑跳变）
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
        // 有效性检查: 宽度 > 8 且左边界在增长 → 继续；否则丢线
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

  // ---- 阶段8: 环岛出口，修复底部边界 ----
  if (ImageFlag.image_element_rings_flag == 8) {
    Repair_Point_Ysite = 7;
    for (int Ysite = 57; Ysite > Repair_Point_Ysite - 3; Ysite--) {
      // 用弯道半宽推算左边界 = 右边界 - Half_Bend_Wide[Ysite]
      ImageDeal[Ysite].LeftBorder =
          ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
      if (ImageDeal[Ysite].LeftBorder < 3) {
        ImageDeal[Ysite].LeftBorder = 3;                 // 钳制不超出图像
      }
      ImageDeal[Ysite].Center =
          (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
    }
  }

  // ---- 阶段9: 环岛退出，过渡到正常模式 ----
  if (ImageFlag.image_element_rings_flag == 9) {
    for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--) {
      // 用直道半宽推算中心线
      ImageDeal[Ysite].Center =
          ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
    }
  }
}
