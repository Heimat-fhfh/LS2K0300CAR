# TDD Evidence Report — 目标板识别 (红色矩形 + TFLite 分类)

## Source plan
未使用 `*.plan.md`,需求由用户在会话中直接提出并在 plan 模式确认后实施。

## User journeys
1. 作为车辆, 我希望在赛道中心识别到目标板时, 通过红色矩形色块定位 -> 模型分类确认, 以便按类别采取不同的循迹策略。
2. 作为开发者, 我希望阈值可在 `config/*.jsonc` 中调, 无需重新编译就能适应不同光照/赛道场景。
3. 作为开发者, 我希望近端色块 (y>30 原图行) 才送分类, 远端噪声不触发误判。
4. 作为开发者, 我希望连续 ≥3 帧同类别才确认, 并有超时帧到期后自动失效, 防止瞬时误识别永久改变循迹线。

## Task report

| 阶段 | 执行 | 验证命令 | 结果 |
|---|---|---|---|
| RED | 桩 `TargetBoardProcess`/`TargetBoardReset` 空实现, 编译并运行测试 | `cmake --build build_host --target test_target_board && ./build_host/test_target_board` | 28 tests, 13 passed, 15 failed (RED gate satisfied) |
| GREEN | 恢复完整实现后重新构建并运行 | 同上 | 28 tests, 28 passed (GREEN satisfied) |
| 重构 | 无; 实现已最小化 | — | — |
| 覆盖率 | `--coverage` 构建 + `gcov -f` | `gcov -f .../target_board.cpp.gcno` | 4 个项目函数全部 100% (详见下) |
| 边缘构建 | 重新生成 `build/` 并刷主目标 | `cmake --build build --target main` | 100% Built target main (TFLM + 模型头链接成功) |
| Host 整合 | 构建 image_web_test (host 桩跳过推理) | `cmake --build build_host --target image_web_test` | 100% Built target image_web_test |

## Test specification

| # | What is guaranteed | Test file or command | Test type | Result |
|---|---|---|---|---|
| 1 | 无红色块时不触发 override | `tests/vision/test_target_board.cpp::Test_NoBlob` | unit | PASS |
| 2 | 连续 confirm 帧后激活 override 且类别=WEAPON, 剩余帧数正确 | `Test_WeaponConfirm` | unit | PASS |
| 3 | 候选中间换类别, 计数器重置 | `Test_SwitchClass` | unit | PASS |
| 4 | override 倒计时过期后失效 | `Test_TimeoutExpiry` | unit | PASS |
| 5 | y<Y_GATE 不送分类 (推理回调不被调用) | `Test_YGateBelow` | unit | PASS |
| 6 | 色块中心在赛道外 -> 跳过 | `Test_OutOfTrack` | unit | PASS |
| 7 | 面积<minArea -> 跳过 | `Test_AreaBelow` | unit | PASS |
| 8 | traffic 类别确认 (保持原中线) | `Test_TrafficConfirm` | unit | PASS |
| 9 | `cfg.enable=false` 直接返回不检测 | `Test_Disable` | unit | PASS |
| 10 | `TargetBoardReset` 重置 override 状态 | `Test_Reset` | unit | PASS |

## Coverage and known gaps

`gcov -f` 对 `src/vision/target_board.cpp`:

| Function | Lines executed |
|---|---|
| `TargetBoardProcess(cv::Mat&, Data_Path*)` | 100% of 79 |
| `InTrack(int, int)` | 100% of 9 |
| `TargetBoardReset()` | 100% of 7 |
| `RunInfer(cv::Mat const&)` (host-stub 分支) | 100% | 
| `TargetBoardSetInferHook(...)` | 100% |

Host 可测代码覆盖率: **100%**。

已知 gap:
- TFLM 真实推理分支 (`#if defined(TARGET_BOARD_USE_TFLM)` 内 `EnsureTflm`+`interpreter.Invoke`): 仅在边缘切换 `TARGET_BOARD_USE_TFLM` 时编译, host 无 TLM 库不可测, 由边缘构建 + 真机集成负责验证。已通过 `cmake --build build --target main` 证明链接通过; 真机推理效果需用户在含红色矩形+目标板的实拍数据集上验证。
- 防御性异常分支 (target_board.cpp:239-240 `else { 重置 }`): `RunInfer` 协议确保只返回 -1 或 0..2, 永不可达。

## Merge evidence (squash 时保留)
- RED: `28 tests, 13 passed, 15 failed` (实现为空桩)
- GREEN: `28 tests, 28 passed, 0 failed` (恢复完整实现)
- 覆盖率: 项目函数 100% 可达分支
- 构建: host `image_web_test` 100% built; edge `main` 100% built (含 TFLM)

## 迭代2 — ROI 几何中心偏移 + 指定尺寸矩形

### 需求变更
用户明确: 分类输入应当是「以色块几何中心为锚点, 按 X/Y 各偏移指定像素, 再以新中心画**指定宽高**的矩形」, 替换原 `bbox + pad` 方案。

### 配置项变更
- 移除: `TARGET_ROI_PAD_PX`
- 新增:
  - `TARGET_ROI_OFFSET_X` (像素)
  - `TARGET_ROI_OFFSET_Y` (像素)
  - `TARGET_ROI_W` (像素)
  - `TARGET_ROI_H` (像素)
- 影响: `libdata_store.h::JSON_TargetBoardConfigData`, `libdata_process.cpp::ConfigData_SYNC`, `config_{0,1,2}.jsonc`

### 实现 (`src/vision/target_board.cpp` 步骤C)
```
nCx = bbox.x + bbox.w/2 + cfg.roiOffsetX
nCy = bbox.y + bbox.h/2 + cfg.roiOffsetY
roi  = Rect(nCx - cfg.roiW/2, nCy - cfg.roiH/2, cfg.roiW, cfg.roiH)
if roi 完整落在图像内: crop -> resize(40x40) -> RunInfer
else: 跳过推理
```

### 测试新增 (5 个)
| # | What is guaranteed | Test |
|---|---|---|
| 11 | ROI 默认无偏移, 40x40 中心=bbox中心, hook 收到包含红色像素的 40x40 图 | `Test_ROIOffsetApplied` |
| 12 | 偏移将 ROI 整体推到图像外 → 矩形不完整 → 跳过推理 | `Test_ROIOffsetMisses` |

旧测试中 ROI 隐式行为依赖 (面积/y门控/越界) 均沿用默认 roiW=roiH=40, 无需改动, 全部通过。

### RED/GREEN
- RED (修改后): 旧实现引用已删除的 `cfg.roiPadPx` 导致**编译失败** (5 个测试编译不过即视为 RED gate)
- GREEN: 替换 ROI 计算后 `33 tests, 33 passed, 0 failed`
- host `image_web_test` 100% built; edge `main` 100% built (含 TFLM)