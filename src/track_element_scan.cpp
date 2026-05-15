#include "common_system.h"
#include "common_program.h"
#include "libdata_store.h"
#include "libdata_process.h"
#include <cstring>
#include <vector>

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


} // namespace

/*
    TransitionScanDetect 说明
    使用已有的 OTSU/二值图，轮廓层级检测被白色包围的黑色区域，
    再结合对侧边线直线度进行圆环/十字分类。
*/
void Judge::TransitionScanDetect(Img_Store* Img_Store_p, Data_Path* Data_Path_p,
                                  Function_EN* Function_EN_p)
{
    if (Img_Store_p == nullptr || Data_Path_p == nullptr || Function_EN_p == nullptr) {
        std::cerr << "Error: null pointer passed to TransitionScanDetect" << std::endl;
        return;
    }

    JSON_TrackConfigData cfg = Data_Path_p->JSON_TrackConfigData_v[0];

    const int min_area = std::max(10, cfg.TransitionMinArea);

    Data_Path_p->TransLeftBlockFound = false;
    Data_Path_p->TransRightBlockFound = false;
    Data_Path_p->TransLeftBlockStart = -1;
    Data_Path_p->TransLeftBlockEnd = -1;
    Data_Path_p->TransRightBlockStart = -1;
    Data_Path_p->TransRightBlockEnd = -1;

    cv::Mat binary_img;
    if (!Img_Store_p->Img_OTSU.empty()) {
        binary_img = Img_Store_p->Img_OTSU;
    } else {
        binary_img = cv::Mat(image_h, image_w, CV_8UC1, Img_Store_p->bin_image);
    }

    if (binary_img.empty()) {
        Data_Path_p->TransitionDetectKind = TRANSITION_ELEMENT_NONE;
        Data_Path_p->TransitionDetectSide = 0;
        std::cerr << "Error: binary image is empty in TransitionScanDetect" << std::endl;
        return;
    }

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    bool left_found = false;
    bool right_found = false;
    int left_ref_row = -1;
    int right_ref_row = -1;

    for (size_t i = 0; i < contours.size() && i < hierarchy.size(); ++i) {
        if (hierarchy[i][3] < 0) continue;  // skip outer contours
        double area = contourArea(contours[i]);
        if (area < min_area) continue;

        Moments mu = moments(contours[i]);
        if (mu.m00 == 0.0) continue;

        int cx = static_cast<int>(mu.m10 / mu.m00);
        int cy = static_cast<int>(mu.m01 / mu.m00);
        if (cy < 0 || cy >= image_h) continue;

        int center_x = static_cast<int>(Data_Path_p->center_line[cy]);
        if (cx < center_x) {
            left_found = true;
            if (left_ref_row < 0) left_ref_row = cy;
        } else {
            right_found = true;
            if (right_ref_row < 0) right_ref_row = cy;
        }
    }

    int candidate_kind = TRANSITION_ELEMENT_NONE;
    int candidate_side = 0;

    if (left_found || right_found) {
        if (left_found && right_found) {
            candidate_kind = TRANSITION_ELEMENT_CROSS;
            candidate_side = 0;
        } else if (left_found) {
            candidate_side = 1;
            int ref_row = (left_ref_row >= 0) ? left_ref_row : image_h / 2;
            int row_start = std::max(0, ref_row - 5);
            int row_end = std::min(image_h - 1, ref_row + 5);
            candidate_kind = isBoundaryApproxStraight(Data_Path_p->r_border, row_start, row_end)
                                 ? TRANSITION_ELEMENT_CIRCLE
                                 : TRANSITION_ELEMENT_CROSS;
        } else {
            candidate_side = 2;
            int ref_row = (right_ref_row >= 0) ? right_ref_row : image_h / 2;
            int row_start = std::max(0, ref_row - 5);
            int row_end = std::min(image_h - 1, ref_row + 5);
            candidate_kind = isBoundaryApproxStraight(Data_Path_p->l_border, row_start, row_end)
                                 ? TRANSITION_ELEMENT_CIRCLE
                                 : TRANSITION_ELEMENT_CROSS;
        }
    }
}
