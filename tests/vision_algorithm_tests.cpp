#include "libdata_process.h"
#include "npz_loader.h"

#include <iostream>

namespace {

JSON_TrackConfigData make_track_cfg()
{
    JSON_TrackConfigData cfg;
    cfg.TrackKindCountThreshold = 0;
    cfg.TrackWidth = 100;
    cfg.Default_Forward = 80;
    cfg.Path_Search_Start = 25;
    cfg.Path_Search_End = 163;
    cfg.Side_Search_Start = 25;
    cfg.Side_Search_End = 163;
    cfg.InflectionPointVectorDistance = 4;
    cfg.LCornerMinAngle = 70;
    cfg.LCornerMaxAngle = 140;
    cfg.StraightMaxAngle = 5;
    cfg.FarLineLeftX = 86;
    cfg.FarLineRightX = 280;
    cfg.CommonMotorSpeed[0] = 1.0;
    cfg.CommonMotorSpeed[3] = 0.6;
    cfg.CommonMotorSpeed[4] = 0.5;
    cfg.CommonMotorSpeed[5] = 0.4;
    return cfg;
}

Function_EN make_function_cfg()
{
    Function_EN fn;
    JSON_FunctionConfigData cfg;
    cfg.AcrossIdentify_EN = true;
    cfg.CircleIdentify_EN = true;
    fn.JSON_FunctionConfigData_v.push_back(cfg);
    return fn;
}

Data_Path make_data_path()
{
    Data_Path data;
    data.JSON_TrackConfigData_v.push_back(make_track_cfg());
    data.search_print_h_max = 80;
    return data;
}

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
        return false;
    }
    return true;
}

bool test_npz_loader()
{
    VisionCoordinateMap map;
    std::string error;
    if (!LoadVisionCoordinateMapNpz("config/vision_transform_coordinate_map.npz", map, &error)) {
        std::cerr << "FAIL: NPZ load failed: " << error << std::endl;
        return false;
    }
    return expect(map.map_x.rows == image_h && map.map_x.cols == image_w, "map_x shape") &&
           expect(map.map_y.rows == image_h && map.map_y.cols == image_w, "map_y shape") &&
           expect(map.valid_mask.type() == CV_8UC1, "valid_mask type");
}

bool test_track_judge()
{
    Judge judge;
    Function_EN fn = make_function_cfg();

    Data_Path straight = make_data_path();
    straight.LPointFound[0] = false;
    straight.LPointFound[1] = false;
    judge.TrackKind_Judge(nullptr, &straight, &fn);
    if (!expect(straight.Loop_Kind == COMMON_TRACK_LOOP, "straight stays common")) {
        return false;
    }

    Data_Path cross = make_data_path();
    cross.LPointFound[0] = true;
    cross.LPointFound[1] = true;
    judge.TrackKind_Judge(nullptr, &cross, &fn);
    if (!expect(cross.Loop_Kind == ACROSS_TRACK_LOOP, "double L triggers cross")) {
        return false;
    }

    Data_Path left_circle = make_data_path();
    left_circle.LPointFound[0] = true;
    left_circle.LPointFound[1] = false;
    left_circle.StraightSide[1] = true;
    judge.TrackKind_Judge(nullptr, &left_circle, &fn);
    if (!expect(left_circle.Loop_Kind == CIRCLE_TRACK_LOOP &&
                left_circle.Runtime_Circle_State == RUNTIME_CIRCLE_LEFT_BEGIN,
                "left L plus right straight triggers left circle")) {
        return false;
    }

    Data_Path right_circle = make_data_path();
    right_circle.LPointFound[0] = false;
    right_circle.LPointFound[1] = true;
    right_circle.StraightSide[0] = true;
    judge.TrackKind_Judge(nullptr, &right_circle, &fn);
    return expect(right_circle.Loop_Kind == CIRCLE_TRACK_LOOP &&
                  right_circle.Runtime_Circle_State == RUNTIME_CIRCLE_RIGHT_BEGIN,
                  "right L plus left straight triggers right circle");
}

bool test_center_planning()
{
    Judge judge;
    Data_Path left = make_data_path();
    left.Follow_Side = FOLLOW_LEFT_SIDE;
    for (int y = 220; y >= 80; y -= 10) {
        left.sampled_left_points.emplace_back(100.0F, static_cast<float>(y));
    }
    judge.PlanCenterLine(nullptr, &left);
    if (!expect(!left.planned_center_points.empty(), "left planning produces points")) {
        return false;
    }
    if (!expect(left.planned_center_points.front().x > 100.0F, "left line offsets toward center")) {
        return false;
    }

    Data_Path right = make_data_path();
    right.Follow_Side = FOLLOW_RIGHT_SIDE;
    for (int y = 220; y >= 80; y -= 10) {
        right.sampled_right_points.emplace_back(220.0F, static_cast<float>(y));
    }
    judge.PlanCenterLine(nullptr, &right);
    return expect(!right.planned_center_points.empty(), "right planning produces points") &&
           expect(right.planned_center_points.front().x < 220.0F, "right line offsets toward center");
}

} // namespace

int main()
{
    bool ok = true;
    ok = test_npz_loader() && ok;
    ok = test_track_judge() && ok;
    ok = test_center_planning() && ok;
    if (!ok) {
        return 1;
    }
    std::cout << "vision_algorithm_tests OK" << std::endl;
    return 0;
}
