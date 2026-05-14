#include "common_system.h"
#include "common_program.h"
#include "libdata_store.h"
#include "libdata_process.h"
#include <cstring>

using namespace std;
using namespace cv;

namespace {

// 检查边界数组在指定行范围内是否近似为直线
// 返回 true 表示 maxX - minX 偏差 <= 阈值
bool isBoundaryApproxStraight(const uint16 boundary[], int row_start, int row_end)
{
    if (row_start >= row_end) return true;

    int min_x = 10000;
    int max_x = -1;
    int valid_count = 0;

    for (int y = row_start; y <= row_end; ++y) {
        int x = static_cast<int>(boundary[y]);
        if (x > border_min && x < border_max) {
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            ++valid_count;
        }
    }

    if (valid_count < 3) return true;
    return (max_x - min_x) <= 15;
}

// 单侧横向扫描，统计跳变次数并在图像上标注跳变点
// 从 center_x 向 boundary_x 方向扫描 bin_image[row][]
// draw_img 非空时在每个有效跳变点画圆点
// 返回跳变总次数（有效跳变，过滤短噪点）
int scanOneSide(const uint8 bin_image[image_h][image_w], int row,
                int center_x, int boundary_x, int min_color_len,
                cv::Mat* draw_img = nullptr)
{
    if (row < 0 || row >= image_h) return 0;
    if (center_x < 0 || center_x >= image_w) return 0;
    if (boundary_x < 0 || boundary_x >= image_w) return 0;

    // 跳过中心点本身（不计入跳变），从中心点相邻像素开始扫描
    int step = (boundary_x > center_x) ? 1 : -1;
    int start_x = center_x + step;

    // 如果起点超出范围或已到达边界，返回0
    if ((step > 0 && start_x >= boundary_x) || (step < 0 && start_x <= boundary_x)) {
        return 0;
    }
    if (start_x < 0 || start_x >= image_w) return 0;

    int prev_val = bin_image[row][start_x];
    int color_run = 1;
    int trans_count = 0;

    for (int x = start_x + step; (step > 0 ? x <= boundary_x : x >= boundary_x); x += step) {
        if (x < 0 || x >= image_w) break;

        int cur_val = bin_image[row][x];

        if (cur_val == prev_val) {
            ++color_run;
        } else {
            // 跳变发生：prev_val -> cur_val
            // 仅在边界点之前记录跳变，边界点处的跳变不记录
            if (x != boundary_x && color_run >= min_color_len) {
                ++trans_count;
                // 标注跳变点：从哪个像素开始变
                if (draw_img != nullptr && !draw_img->empty()) {
                    // prev_val->cur_val: 白(255)→黑(0) 用红点，黑(0)→白(255) 用绿点
                    Scalar color = (prev_val == 255 && cur_val == 0) ? Scalar(0, 0, 255) : Scalar(0, 255, 0);
                    // 跳变点位于 x 位置（新值开始处）
                    circle(*draw_img, Point(x, row), 1, color, -1);
                }
            }
            prev_val = cur_val;
            color_run = 1;
        }
    }
    return trans_count;
}

// 在行范围 [row_start, row_end] 内查找连续具有恰好2次跳变的行块
// 返回 true 表示找到了符合条件的隔离块，并输出块的起止行
bool findIsolatedBlock(const int trans_counts[], int row_start, int row_end,
                       int min_run_length, int& block_start_out, int& block_end_out)
{
    int current_start = -1;

    for (int y = row_start; y >= row_end; --y) {
        if (trans_counts[y] == 2) {
            if (current_start < 0) current_start = y;
        } else {
            if (current_start >= 0) {
                int block_len = current_start - y;  // current_start >= y
                if (block_len >= min_run_length) {
                    // 检查隔离性：上方行和下方行都不存在2次跳变
                    int row_above = current_start + 1; // 更靠近图像顶部
                    int row_below = y + 1;              // 更靠近图像底部

                    bool above_ok = (row_above > row_start) ||
                                    (row_above <= row_start && trans_counts[row_above] != 2);
                    bool below_ok = (row_below < row_end) ||
                                    (row_below >= row_end && trans_counts[row_below] != 2);

                    if (above_ok && below_ok) {
                        block_start_out = current_start;
                        block_end_out = row_below;
                        return true;
                    }
                }
                current_start = -1;
            }
        }
    }

    // 检查是否块延伸到搜索范围顶部
    if (current_start >= 0) {
        int block_len = current_start - row_end + 1;
        if (block_len >= min_run_length) {
            int row_below = row_end;
            bool below_ok = true;  // 已在搜索范围边界
            if (row_below - 1 >= row_end && trans_counts[row_below - 1] == 2) {
                below_ok = false;  // 实际上行在范围内，此处保持逻辑一致
            }
            if (below_ok) {
                block_start_out = current_start;
                block_end_out = row_end;
                return true;
            }
        }
    }

    return false;
}

} // namespace

/*
    TransitionScanDetect 说明
    跳变扫描赛道元素识别。
    利用八邻域搜线结果，从图像中线向两侧扫描二值图，
    通过黑白跳变模式检测圆环入口和十字路口的独立黑色区域，
    利用对侧边线直线度进行最终分类。
*/
void Judge::TransitionScanDetect(Img_Store* Img_Store_p, Data_Path* Data_Path_p,
                                  Function_EN* Function_EN_p)
{
    if (Img_Store_p == nullptr || Data_Path_p == nullptr || Function_EN_p == nullptr) {
        return;
    }

    JSON_TrackConfigData cfg = Data_Path_p->JSON_TrackConfigData_v[0];

    // 未使能则重置状态并返回
    if (cfg.TransitionScanEnable == 0) {
        Data_Path_p->TransitionDetectKind = TRANSITION_ELEMENT_NONE;
        Data_Path_p->TransitionDetectSide = 0;
        Data_Path_p->TransitionDebounceCounter = 0;
        Data_Path_p->TransitionCandidateKind = TRANSITION_ELEMENT_NONE;
        Data_Path_p->TransitionCandidateSide = 0;
        return;
    }

    const int min_run_length = std::max(1, cfg.TransitionMinRunLength);
    const int min_color_len = std::max(1, cfg.TransitionMinColorLength);
    const int debounce_frames = std::max(1, cfg.TransitionDebounceFrames);

    // 搜索行范围（从图像底部向上）
    const int start_row = image_h - 1 - cfg.Path_Search_Start;
    const int end_row = image_h - 1 - cfg.Side_Search_End;

    if (start_row <= end_row || start_row >= image_h || end_row < 0) {
        Data_Path_p->TransitionDetectKind = TRANSITION_ELEMENT_NONE;
        return;
    }

    // 每行的左右侧跳变次数
    int left_trans[image_h];
    int right_trans[image_h];
    memset(left_trans, 0, sizeof(left_trans));
    memset(right_trans, 0, sizeof(right_trans));

    // 阶段一：逐行横向扫描，统计跳变次数并标注跳变点
    cv::Mat* draw_img = Img_Store_p->Img_Track.empty() ? nullptr : &Img_Store_p->Img_Track;
    for (int y = start_row; y >= end_row; --y) {
        int center_x = static_cast<int>(Data_Path_p->center_line[y]);
        int l_bound = static_cast<int>(Data_Path_p->l_border[y]);
        int r_bound = static_cast<int>(Data_Path_p->r_border[y]);

        // 验证数据有效性
        if (l_bound < 0) l_bound = 0;
        if (r_bound >= image_w) r_bound = image_w - 1;
        if (center_x <= l_bound || center_x >= r_bound) continue;

        // 左侧扫描：从中心向左扫描到左边界
        left_trans[y] = scanOneSide(Img_Store_p->bin_image, y, center_x, l_bound, min_color_len, draw_img);

        // 右侧扫描：从中心向右扫描到右边界
        right_trans[y] = scanOneSide(Img_Store_p->bin_image, y, center_x, r_bound, min_color_len, draw_img);
    }

    // 阶段二：分别对左右侧查找连续2跳变隔离块
    int left_block_start = -1, left_block_end = -1;
    int right_block_start = -1, right_block_end = -1;

    bool left_found = findIsolatedBlock(left_trans, start_row, end_row,
                                        min_run_length, left_block_start, left_block_end);
    bool right_found = findIsolatedBlock(right_trans, start_row, end_row,
                                         min_run_length, right_block_start, right_block_end);

    // 阶段三：分类判定
    int candidate_kind = TRANSITION_ELEMENT_NONE;
    int candidate_side = 0;

    if (left_found || right_found) {
        if (left_found && !right_found) {
            // 仅左侧有隔离块，检查右侧边线直线度
            int check_start = std::max(end_row, left_block_end - 5);
            int check_end = std::min(start_row, left_block_start + 5);
            if (isBoundaryApproxStraight(Data_Path_p->r_border, check_start, check_end)) {
                candidate_kind = TRANSITION_ELEMENT_CIRCLE;
                candidate_side = 1;  // left circle
            } else {
                candidate_kind = TRANSITION_ELEMENT_CROSS;
                candidate_side = 1;  // left side cross indicator
            }
        } else if (right_found && !left_found) {
            // 仅右侧有隔离块，检查左侧边线直线度
            int check_start = std::max(end_row, right_block_end - 5);
            int check_end = std::min(start_row, right_block_start + 5);
            if (isBoundaryApproxStraight(Data_Path_p->l_border, check_start, check_end)) {
                candidate_kind = TRANSITION_ELEMENT_CIRCLE;
                candidate_side = 2;  // right circle
            } else {
                candidate_kind = TRANSITION_ELEMENT_CROSS;
                candidate_side = 2;  // right side cross indicator
            }
        } else {
            // 双侧都有隔离块 → 十字
            candidate_kind = TRANSITION_ELEMENT_CROSS;
            candidate_side = 0;
        }
    }

    // 阶段四：帧间防抖
    if (candidate_kind == Data_Path_p->TransitionCandidateKind &&
        candidate_side == Data_Path_p->TransitionCandidateSide &&
        candidate_kind != TRANSITION_ELEMENT_NONE) {
        Data_Path_p->TransitionDebounceCounter++;
    } else {
        Data_Path_p->TransitionDebounceCounter = 0;
    }

    Data_Path_p->TransitionCandidateKind = candidate_kind;
    Data_Path_p->TransitionCandidateSide = candidate_side;

    if (Data_Path_p->TransitionDebounceCounter >= debounce_frames) {
        Data_Path_p->TransitionDetectKind = candidate_kind;
        Data_Path_p->TransitionDetectSide = candidate_side;
    } else {
        Data_Path_p->TransitionDetectKind = TRANSITION_ELEMENT_NONE;
        Data_Path_p->TransitionDetectSide = 0;
    }

    // 阶段五：可视化绘制
    if (!Img_Store_p->Img_Track.empty()) {
        // 绘制左右侧跳变行（用半透明色标记2跳变行）
        for (int y = start_row; y >= end_row; --y) {
            if (left_trans[y] == 2) {
                line(Img_Store_p->Img_Track, Point(0, y), Point(10, y),
                     Scalar(255, 200, 0), 1);
            }
            if (right_trans[y] == 2) {
                line(Img_Store_p->Img_Track, Point(image_w - 11, y),
                     Point(image_w - 1, y), Scalar(255, 200, 0), 1);
            }
        }

        // 绘制隔离块区域
        Scalar block_color;
        if (candidate_kind == TRANSITION_ELEMENT_CIRCLE) {
            block_color = Scalar(0, 255, 0);
        } else if (candidate_kind == TRANSITION_ELEMENT_CROSS) {
            block_color = Scalar(0, 255, 255);
        } else {
            block_color = Scalar(0, 0, 255);
        }

        if (left_found && left_block_start >= 0 && left_block_end >= 0) {
            int block_x = std::max(0, static_cast<int>(Data_Path_p->l_border[left_block_start]) - 15);
            rectangle(Img_Store_p->Img_Track,
                      Point(block_x, left_block_end),
                      Point(block_x + 15, left_block_start),
                      block_color, 2);
        }
        if (right_found && right_block_start >= 0 && right_block_end >= 0) {
            int block_x = std::min(image_w - 15,
                                   static_cast<int>(Data_Path_p->r_border[right_block_start]) + 1);
            rectangle(Img_Store_p->Img_Track,
                      Point(block_x, right_block_end),
                      Point(block_x + 15, right_block_start),
                      block_color, 2);
        }

        // 绘制检测结果文字
        const char* kind_text = "NONE";
        if (candidate_kind == TRANSITION_ELEMENT_CIRCLE) {
            kind_text = (candidate_side == 1) ? "L-CIRCLE" : "R-CIRCLE";
        } else if (candidate_kind == TRANSITION_ELEMENT_CROSS) {
            kind_text = "CROSS";
        }

        char info_text[64];
        snprintf(info_text, sizeof(info_text), "%s deb:%d/%d",
                 kind_text, Data_Path_p->TransitionDebounceCounter, debounce_frames);
        putText(Img_Store_p->Img_Track, info_text, Point(5, image_h - 5),
                FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1);

        // 若已确认，画更明显的标记
        if (Data_Path_p->TransitionDetectKind != TRANSITION_ELEMENT_NONE) {
            const char* confirmed_text = nullptr;
            if (Data_Path_p->TransitionDetectKind == TRANSITION_ELEMENT_CIRCLE) {
                confirmed_text = (Data_Path_p->TransitionDetectSide == 1)
                    ? "LEFT CIRCLE" : "RIGHT CIRCLE";
            } else {
                confirmed_text = "CROSS";
            }
            putText(Img_Store_p->Img_Track, confirmed_text, Point(image_w / 2 - 40, 20),
                    FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }
    }
}
