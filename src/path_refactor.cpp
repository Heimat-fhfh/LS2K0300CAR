#include "path_refactor.h"

#include <algorithm>
#include <cmath>

namespace {
// 本地整数限幅，避免数组越界。
int clip_int(int value, int low, int high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

float degree_to_rad(float degree) {
    return degree * static_cast<float>(CV_PI / 180.0);
}
} // namespace

void triangle_blur_points(const std::vector<cv::Point2f>& in_points,
                          std::vector<cv::Point2f>& out_points,
                          int kernel) {
    out_points.clear();
    if (in_points.empty()) {
        return;
    }

    if (kernel < 1) {
        kernel = 1;
    }
    if (kernel % 2 == 0) {
        kernel += 1;
    }

    const int half = kernel / 2;
    const float sum_weight = static_cast<float>((2 * half + 2) * (half + 1) / 2);

    out_points.resize(in_points.size());
    for (size_t i = 0; i < in_points.size(); ++i) {
        cv::Point2f accum(0.0F, 0.0F);
        for (int j = -half; j <= half; ++j) {
            const int idx = clip_int(static_cast<int>(i) + j, 0, static_cast<int>(in_points.size()) - 1);
            const float w = static_cast<float>(half + 1 - std::abs(j));
            accum += in_points[idx] * w;
        }
        out_points[i] = accum / sum_weight;
    }
}

void resample_points_equal_distance(const std::vector<cv::Point2f>& in_points,
                                    std::vector<cv::Point2f>& out_points,
                                    float distance,
                                    size_t max_points) {
    out_points.clear();
    if (in_points.size() < 2 || distance <= 0.0F || max_points == 0) {
        return;
    }

    // remain 表示距离下一个采样点还需沿折线前进的弧长。
    float remain = 0.0F;
    for (size_t i = 0; i + 1 < in_points.size() && out_points.size() < max_points; ++i) {
        float x0 = in_points[i].x;
        float y0 = in_points[i].y;
        float dx = in_points[i + 1].x - x0;
        float dy = in_points[i + 1].y - y0;
        float dn = std::sqrt(dx * dx + dy * dy);
        if (dn < 1e-3F) {
            continue;
        }

        dx /= dn;
        dy /= dn;

        while (remain < dn && out_points.size() < max_points) {
            x0 += dx * remain;
            y0 += dy * remain;
            out_points.emplace_back(x0, y0);
            dn -= remain;
            remain = distance;
        }
        remain -= dn;
    }
}

void local_angle_points(const std::vector<cv::Point2f>& points,
                        std::vector<float>& angle_out,
                        int dist_step) {
    angle_out.assign(points.size(), 0.0F);
    if (points.size() < 3) {
        return;
    }

    if (dist_step < 1) {
        dist_step = 1;
    }

    const int n = static_cast<int>(points.size());
    for (int i = 1; i < n - 1; ++i) {
        const int i0 = clip_int(i - dist_step, 0, n - 1);
        const int i1 = clip_int(i + dist_step, 0, n - 1);

        const float dx1 = points[i].x - points[i0].x;
        const float dy1 = points[i].y - points[i0].y;
        const float dx2 = points[i1].x - points[i].x;
        const float dy2 = points[i1].y - points[i].y;

        const float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        const float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (dn1 < 1e-3F || dn2 < 1e-3F) {
            angle_out[i] = 0.0F;
            continue;
        }

        // 通过两段切向量计算局部转角，输出范围 (-pi, pi]。
        const float c1 = dx1 / dn1;
        const float s1 = dy1 / dn1;
        const float c2 = dx2 / dn2;
        const float s2 = dy2 / dn2;
        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

void nms_angle_absmax(const std::vector<float>& angle_in,
                      std::vector<float>& angle_out,
                      int kernel) {
    if (kernel < 1) {
        kernel = 1;
    }
    if (kernel % 2 == 0) {
        kernel += 1;
    }

    const int half = kernel / 2;
    angle_out = angle_in;
    const int n = static_cast<int>(angle_in.size());
    for (int i = 0; i < n; ++i) {
        for (int j = -half; j <= half; ++j) {
            const int idx = clip_int(i + j, 0, n - 1);
            if (std::fabs(angle_in[idx]) > std::fabs(angle_out[i])) {
                angle_out[i] = 0.0F;
                break;
            }
        }
    }
}

PathRefactorFeatureResult detect_feature_by_angle(const std::vector<cv::Point2f>& points,
                                                  float min_angle_deg,
                                                  float max_angle_deg,
                                                  int dist_step,
                                                  int index_gap,
                                                  int border_margin,
                                                  bool is_left_side) {
    PathRefactorFeatureResult result;
    if (points.size() < 8) {
        return result;
    }

    // 先计算角度变化率，再做 NMS，最后做阈值筛选。
    std::vector<float> angles;
    std::vector<float> angles_nms;
    local_angle_points(points, angles, dist_step);
    nms_angle_absmax(angles, angles_nms, 2 * dist_step + 1);

    const float min_angle = degree_to_rad(min_angle_deg);
    const float max_angle = degree_to_rad(max_angle_deg);
    int last_pick = -index_gap;

    for (int i = dist_step; i + dist_step < static_cast<int>(points.size()); ++i) {
        if (std::fabs(angles_nms[i]) < 1e-5F) {
            continue;
        }
        if (i - last_pick < index_gap) {
            continue;
        }

        if (is_left_side) {
            if (points[i].x <= border_margin) {
                continue;
            }
        } else {
            if ((image_w - 1) - points[i].x <= border_margin) {
                continue;
            }
        }

        const int im1 = clip_int(i - dist_step, 0, static_cast<int>(points.size()) - 1);
        const int ip1 = clip_int(i + dist_step, 0, static_cast<int>(points.size()) - 1);
        // 置信度采用“中心角强度 - 两侧角强度均值”，抑制缓弯误检。
        const float conf = std::fabs(angles[i]) - (std::fabs(angles[im1]) + std::fabs(angles[ip1])) * 0.5F;

        if (conf >= min_angle && conf <= max_angle) {
            result.indices.push_back(i);
            last_pick = i;

            if (result.vertical_direction_sign == 0) {
                const float vy = (points[im1].y - points[i].y) + (points[ip1].y - points[i].y);
                if (std::fabs(vy) > 1e-3F) {
                    result.vertical_direction_sign = (vy > 0.0F) ? 1 : -1;
                }
            }
        }
    }

    return result;
}

void optimize_edge_lines(Data_Path* data_path,
                         int image_width,
                         float sample_dist_px,
                         int blur_kernel,
                         int max_points_per_side) {
    if (data_path == nullptr || max_points_per_side <= 0) {
        return;
    }

    auto process_side = [&](bool left_side) {
        const int index_count = left_side ? data_path->NumSearch[0] : data_path->NumSearch[1];
        if (index_count < 5) {
            return;
        }

        std::vector<cv::Point2f> raw_points;
        raw_points.reserve(index_count);
        for (int i = 0; i < index_count; ++i) {
            const float x = static_cast<float>(left_side ? data_path->points_l[i][0] : data_path->points_r[i][0]);
            const float y = static_cast<float>(left_side ? data_path->points_l[i][1] : data_path->points_r[i][1]);
            raw_points.emplace_back(x, y);
        }

        // 平滑后再等距采样，能显著提升角度法识别稳定性。
        std::vector<cv::Point2f> blur_points;
        triangle_blur_points(raw_points, blur_points, blur_kernel);

        std::vector<cv::Point2f> sampled_points;
        resample_points_equal_distance(blur_points, sampled_points, sample_dist_px, static_cast<size_t>(max_points_per_side));

        if (sampled_points.size() < 5) {
            return;
        }

        const int write_count = std::min(static_cast<int>(sampled_points.size()), max_points_per_side);
        if (left_side) {
            data_path->NumSearch[0] = write_count;
            for (int i = 0; i < write_count; ++i) {
                data_path->points_l[i][0] = static_cast<uint16>(clip_int(static_cast<int>(std::lround(sampled_points[i].x)), border_min, image_width - 1));
                data_path->points_l[i][1] = static_cast<uint16>(clip_int(static_cast<int>(std::lround(sampled_points[i].y)), 0, image_h - 1));
            }
        } else {
            data_path->NumSearch[1] = write_count;
            for (int i = 0; i < write_count; ++i) {
                data_path->points_r[i][0] = static_cast<uint16>(clip_int(static_cast<int>(std::lround(sampled_points[i].x)), 0, border_max));
                data_path->points_r[i][1] = static_cast<uint16>(clip_int(static_cast<int>(std::lround(sampled_points[i].y)), 0, image_h - 1));
            }
        }
    };

    process_side(true);
    process_side(false);
}

void compute_smoothed_servo_control(Data_Path* data_path,
                                    int image_width,
                                    int default_forward,
                                    int max_row,
                                    int min_row) {
    if (data_path == nullptr) {
        return;
    }

    // 修复：添加限幅保护，防止 hightest + 10 超出图像范围
    // 原代码 find_row = data_path->hightest + 10 可能超出 max_row，
    // 导致后续 center_line[find_row] 访问越界
    int find_row = default_forward;
    if (find_row < data_path->hightest) {
        find_row = data_path->hightest + 10;
    }
    find_row = clip_int(find_row, min_row, max_row);
    
    // 额外保护：确保 find_row 在 center_line 有效范围内
    if (find_row < 0 || find_row >= image_h) {
        find_row = clip_int(default_forward, min_row, max_row);
    }

    // 采用 1-2-3-2-1 的多行权重，兼顾响应速度和抗抖能力。
    const int row_offsets[5] = {-8, -4, 0, 4, 8};
    const int row_weights[5] = {1, 2, 3, 2, 1};
    int weighted_center = 0;
    int weight_sum = 0;

    for (int i = 0; i < 5; ++i) {
        const int row = clip_int(find_row + row_offsets[i], min_row, max_row);
        weighted_center += static_cast<int>(data_path->center_line[row]) * row_weights[i];
        weight_sum += row_weights[i];
    }

    int servo_error = (weight_sum > 0) ? (weighted_center / weight_sum - image_width / 2) : (static_cast<int>(data_path->center_line[find_row]) - image_width / 2);

    data_path->findrow = find_row;
    data_path->ServoAngle = std::abs(servo_error);
    data_path->ServoDir = (servo_error < 0) ? 1 : -1;

    //打印赛道中线与图像中线的像素偏差
    // printf("像素偏差: %d px (赛道中线: %d, 图像中心: %d)\n", 
    //        servo_error, 
    //        (weight_sum > 0) ? (weighted_center / weight_sum) : static_cast<int>(data_path->center_line[find_row]), 
    //        image_width / 2);
    
		
	
}
