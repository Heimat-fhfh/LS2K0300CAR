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
bool isBoundaryApproxStraight(const uint16 boundary[], int row_start, int row_end)
{
    if (row_start - row_end < 3) {
        cerr << "Error: row range too small for curvature check:" << row_end - row_start << endl;
        return true;
    }
    cout << "Checking boundary straightness from row " << row_start << " to " << row_end << endl;
    // 计算二阶导数（曲率）
    double max_curvature = 0;
    for (int y = row_start-1; y > row_end; --y) {
        // 一阶导数
        double first_deriv = (boundary[y+1] - boundary[y-1]) / 2.0;
        // 二阶导数（曲率的简化）
        double second_deriv = boundary[y+1] - 2*boundary[y] + boundary[y-1];
        double curvature = abs(second_deriv) / (1 + first_deriv*first_deriv);
        
        max_curvature = std::max(max_curvature, curvature);
        
        cout << "Row " << y << ": x=" << boundary[y] << " curvature=" << curvature << endl;
        if (max_curvature > 0.9) {
            return false;
        } // 曲率阈值
    }
    return true;
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

    Data_Path_p->TransitionContours.clear();
    Data_Path_p->TransitionHierarchy.clear();
    findContours(binary_img, Data_Path_p->TransitionContours, Data_Path_p->TransitionHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    bool left_found = false;
    bool right_found = false;

    for (size_t i = 0; i < Data_Path_p->TransitionContours.size() && i < Data_Path_p->TransitionHierarchy.size(); ++i) {
        if (Data_Path_p->TransitionHierarchy[i][3] < 0) continue;  // skip outer contours
        double area = contourArea(Data_Path_p->TransitionContours[i]);
        if (area < min_area) continue;

        Moments mu = moments(Data_Path_p->TransitionContours[i]);
        if (mu.m00 == 0.0) continue;

        int cx = static_cast<int>(mu.m10 / mu.m00);
        int cy = static_cast<int>(mu.m01 / mu.m00);
        if (cy < 0 || cy >= image_h) continue;

        // int center_x = static_cast<int>(Data_Path_p->center_line[cy]);
        int center_x = image_w / 2; // 使用图像中心作为参考线
        if (cx < center_x) {
            left_found = true;
        } else {
            right_found = true;
        }
    }

    int candidate_kind = TRANSITION_ELEMENT_NONE;
    int candidate_side = 0;

    if (left_found || right_found) {
        if (left_found) {
            candidate_side = 1;
            int row_start = image_h - cfg.Side_Search_Start;
            int row_end = Data_Path_p->search_print_h_max;
            candidate_kind = isBoundaryApproxStraight(Data_Path_p->r_border, row_start, row_end)
                                 ? TRANSITION_ELEMENT_CIRCLE
                                 : TRANSITION_ELEMENT_CROSS;
            
        } else if (right_found) {
            candidate_side = 2;
            int row_start = image_h - cfg.Side_Search_Start;
            int row_end = Data_Path_p->search_print_h_max;
            candidate_kind = isBoundaryApproxStraight(Data_Path_p->l_border, row_start, row_end)
                                 ? TRANSITION_ELEMENT_CIRCLE
                                 : TRANSITION_ELEMENT_CROSS;
        } else {
            cout << "TransitionScanDetect: found contour but could not determine side" << endl;
        }
    }
    string side_str = (candidate_side == 1) ? "LEFT" : (candidate_side == 2) ? "RIGHT" : "NONE";
    string kind_str = (candidate_kind == TRANSITION_ELEMENT_CIRCLE) ? "CIRCLE" : "CROSS";
    if(candidate_side == 0) kind_str = "NONE";  
    cout << "TransitionScanDetect candidate: kind=" << kind_str << " side=" << side_str << endl;
}
