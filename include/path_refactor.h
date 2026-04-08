#ifndef _PATH_REFACTOR_H_
#define _PATH_REFACTOR_H_

#include "common_system.h"
#include "libdata_store.h"

// 角点/弯点检测结果。
// indices: 命中的点序号（基于输入点集索引）。
// vertical_direction_sign: 纵向趋势符号，用于圆环准备入环方向判断。
//   1  表示向图像下方趋势更明显。
//  -1 表示向图像上方趋势更明显。
//   0 表示趋势不明确。
struct PathRefactorFeatureResult {
    std::vector<int> indices;
    int vertical_direction_sign = 0;
};

// 点集三角滤波。
// 用于边线去抖，kernel 会自动修正为正奇数。
// in_points: 输入边线点集。
// out_points: 输出平滑点集（长度与输入相同）。
// kernel: 滤波窗口大小，建议 3/5/7。
void triangle_blur_points(const std::vector<cv::Point2f>& in_points,
                          std::vector<cv::Point2f>& out_points,
                          int kernel);

// 点集等距采样。
// 将非均匀点列重采样为近似固定弧长间隔，便于后续角度变化率计算稳定。
// in_points: 输入点集（建议先做平滑）。
// out_points: 输出采样点集。
// distance: 采样间距（像素）。
// max_points: 输出最大点数上限，避免越界。
void resample_points_equal_distance(const std::vector<cv::Point2f>& in_points,
                                    std::vector<cv::Point2f>& out_points,
                                    float distance,
                                    size_t max_points);

// 局部角度变化率计算。
// 对每个点使用前后 dist_step 邻域向量计算转角，输出为弧度。
// points: 输入点集。
// angle_out: 每个点的角度变化率（弧度）。
// dist_step: 角度计算的前后点距离（索引步长）。
void local_angle_points(const std::vector<cv::Point2f>& points,
                        std::vector<float>& angle_out,
                        int dist_step);

// 角度绝对值非极大值抑制。
// 保留局部绝对值最大的角度响应，其余清零。
// angle_in: 输入角度序列（弧度）。
// angle_out: 抑制后的角度序列（弧度）。
// kernel: 抑制窗口大小，函数内部会自动修正为正奇数。
void nms_angle_absmax(const std::vector<float>& angle_in,
                      std::vector<float>& angle_out,
                      int kernel);

// 基于角度变化率提取拐点/弯点。
// 典型用法：
// 1) 先对边线点集做平滑和等距采样。
// 2) 再调用本函数提取满足角度阈值的点。
// points: 输入边线点集。
// min_angle_deg/max_angle_deg: 角度阈值（度），用于 conf 过滤。
// dist_step: 角度计算步长。
// index_gap: 连续命中最小索引间隔，避免密集重复点。
// border_margin: 距离图像左右边界的最小保留边距。
// is_left_side: true 左边线，false 右边线。
PathRefactorFeatureResult detect_feature_by_angle(const std::vector<cv::Point2f>& points,
                                                  float min_angle_deg,
                                                  float max_angle_deg,
                                                  int dist_step,
                                                  int index_gap,
                                                  int border_margin,
                                                  bool is_left_side);

// 边线后处理主入口。
// 对 Data_Path::points_l/points_r 执行：三角平滑 + 等距采样，回写结果和 NumSearch。
// data_path: 路径结构体。
// image_width: 图像宽度（用于边界裁剪）。
// sample_dist_px: 等距采样间距（像素）。
// blur_kernel: 三角滤波窗口。
// max_points_per_side: 单侧最大点数。
void optimize_edge_lines(Data_Path* data_path,
                         int image_width,
                         float sample_dist_px,
                         int blur_kernel,
                         int max_points_per_side);

// 方向控制平滑主入口。
// 在默认前瞻点附近做多行加权，降低舵机角噪声。
// data_path: 路径结构体。
// image_width: 图像宽度。
// default_forward: 默认前瞻行。
// max_row/min_row: 前瞻行限幅。
// 输出会更新 Data_Path::findrow / ServoAngle / ServoDir。
void compute_smoothed_servo_control(Data_Path* data_path,
                                    int image_width,
                                    int default_forward,
                                    int max_row,
                                    int min_row);

#endif
