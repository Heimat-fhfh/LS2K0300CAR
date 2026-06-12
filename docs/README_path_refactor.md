# 边线算法重构使用文档

## 1. 目标与范围

本文档说明本次从 `Reference` 目录迁移的算法能力，以及在当前工程中的使用方式。

本次已落地能力：

1. 边线处理优化：三角滤波 + 等距采样。
2. 迷宫法边线提取后处理接入：在原有 `imgSearch_l_r` 后执行统一后处理。
3. 元素识别重构：拐点/弯点统一为角度变化率 + 非极大值抑制（NMS）流程。
4. 方向控制优化：单点前瞻升级为多行加权前瞻。

## 2. 代码位置

核心模块文件：

1. `include/path_refactor.h`
2. `src/path_refactor.cpp`

接入点文件：

1. `src/path_side_search.cpp`
2. `src/libdata_process.cpp`

## 3. 算法流程

### 3.1 边线后处理流程

输入：`Data_Path.points_l/points_r`（由迷宫法/八邻域输出）

处理步骤：

1. 三角滤波 `triangle_blur_points`。
2. 等距采样 `resample_points_equal_distance`。
3. 回写 `points_l/points_r` 与 `NumSearch`。

输出：

1. 更平滑、更均匀的边线点列。
2. 为后续角度识别提供稳定输入。

### 3.2 元素识别流程

输入：`Data_Path.SideCoordinate_Eight`。

处理步骤：

1. 计算局部角度变化率 `local_angle_points`。
2. 角度非极大值抑制 `nms_angle_absmax`。
3. 使用 `detect_feature_by_angle` 做阈值筛选和边界过滤。
4. 写回：
   - 拐点：`InflectionPointCoordinate` + `InflectionPointNum`
   - 弯点：`BendPointCoordinate` + `BendPointNum`
   - 趋势方向：`Vector_Add_Unit_Dir`

### 3.3 方向控制流程

输入：`center_line` 与默认前瞻行。

处理步骤：

1. 前瞻行限幅。
2. 取 5 行加权中心（权重 1-2-3-2-1）。
3. 转为 `ServoAngle` 与 `ServoDir`。

输出：控制更平滑，抖动降低。

## 4. API 使用说明

### 4.1 `optimize_edge_lines`

文件：`include/path_refactor.h`

作用：边线后处理总入口。

参数建议：

1. `sample_dist_px`：建议 `1.5` 到 `3.0`。
2. `blur_kernel`：建议 `3` 或 `5`，必须为奇数（函数内部会自动修正）。
3. `max_points_per_side`：传 `USE_num`。

调用时机：

1. 在边线巡线完成后立即调用。
2. 在 `get_left/get_right` 之前调用（当前已按此顺序接入）。

### 4.2 `detect_feature_by_angle`

作用：统一拐点/弯点识别。

关键参数：

1. `min_angle_deg/max_angle_deg`：来源于 JSON 角度阈值。
2. `dist_step`：与 `InflectionPointVectorDistance` 或 `BendPointVectorDistance` 对应。
3. `index_gap`：抑制连续重复点，当前推荐 `10`。
4. `border_margin`：边框过滤，当前推荐 `30`。

使用建议：

1. 先保证输入点列已平滑和均匀采样。
2. 若误检多，先增大 `index_gap` 或提高 `min_angle_deg`。
3. 若漏检多，降低 `min_angle_deg` 或减小 `dist_step`。

### 4.3 `compute_smoothed_servo_control`

作用：方向控制平滑入口。

参数建议：

1. `default_forward`：读取 JSON 的 `Forward`。
2. `max_row/min_row`：通常传 `[0, image_h-1]`。

使用建议：

1. 若响应偏慢，可减小行偏移范围（例如从 `{-8,-4,0,4,8}` 改为 `{-6,-3,0,3,6}`）。
2. 若抖动偏大，可提升中心行权重或略增偏移范围。

## 5. 参数调优指南

### 5.1 边线抖动明显

优先调整：

1. `blur_kernel` 从 `5` 增至 `7`。
2. `sample_dist_px` 从 `2.0` 增至 `2.5` 或 `3.0`。

### 5.2 拐点误判多

优先调整：

1. `MIN_INFLECTION_POINT_ANGLE` 上调。
2. `POINT_DISTANCE` 上调（角度计算跨度更大，抑制小噪声拐点）。
3. `index_gap` 上调。

### 5.3 弯点漏检

优先调整：

1. `MIN_BEND_POINT_ANGLE` 下调。
2. `POINT_DISTANCE` 下调。
3. `sample_dist_px` 适当减小到 `1.5~2.0`。

### 5.4 舵机方向抖动

优先调整：

1. 多行加权偏移范围。
2. `Forward` 行。
3. 先确认边线后处理参数是否过激导致细节损失。

## 6. 典型调用链（当前工程）

1. `imgSearch_l_r` 巡线得到原始边线。
2. `optimize_edge_lines` 统一后处理边线。
3. `get_left/get_right` 回填边界数组。
4. `Judge::InflectionPointSearch` / `Judge::BendPointSearch` 做元素识别。
5. `Judge::ServoDirAngle_Judge` 计算平滑方向控制。

## 7. 验证建议

每次改参数后建议至少执行以下检查：

1. 直道：观察 `ServoAngle` 是否平滑，无明显周期震荡。
2. 中等弯道：弯点数量随弯度变化是否符合预期。
3. 十字入口：左右拐点识别是否同时稳定出现。
4. 圆环准备入环：`Vector_Add_Unit_Dir` 符号是否稳定。

## 8. 常见问题

### 8.1 为什么有时边线点数突然减少？

原因：等距采样会跳过太短线段，若原始点列断裂明显会导致输出点数下降。

处理：

1. 检查二值化质量。
2. 适当减小 `sample_dist_px`。

### 8.2 为什么拐点和弯点都变少？

原因：角度阈值与 `dist_step` 同时偏高。

处理：

1. 先降低角度阈值，再微调 `dist_step`。

### 8.3 为什么速度快时识别变差？

原因：图像运动模糊 + 边线抖动放大。

处理：

1. 增加滤波强度。
2. 适当降低前瞻行或提升方向平滑权重。
3. 分赛道类型下调速度上限。

## 9. 后续扩展建议

1. 将 `sample_dist_px`、`blur_kernel`、`index_gap`、`border_margin` 写入 JSON，实现在线可调。
2. 在调试图像上增绘 NMS 后角度峰值点，便于现场调参。
3. 为十字/圆环分别设定独立阈值组，降低全局参数冲突。
