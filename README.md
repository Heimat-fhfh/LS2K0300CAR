# LS2K0300CAR — 第二十一届全国大学生智能汽车竞赛 · 龙芯组别

基于龙芯 LS2K0300 嵌入式平台，搭载计算机视觉与级联 PID 控制算法，实现自主赛道循迹的竞速智能车。

---

## 目录

- [硬件参数](#硬件参数)
- [硬件平台](#硬件平台)
- [目录结构](#目录结构)
- [构建指南](#构建指南)
- [快速开始](#快速开始)
- [系统架构](#系统架构)
- [配置文件](#配置文件)
- [测试与调试](#测试与调试)
- [安全机制](#安全机制)
- [参考资料](#参考资料)
- [开源协议](#开源协议)

---

## 硬件参数

| 参数 | 值 |
|---|---|
| 编码器齿轮齿数 | 30 |
| 电机齿轮齿数 | 68 |
| 车轮直径 | 6.5 cm |
| 编码器读数单位 | 0.1 rps |
| 实际速度换算 | V (m/s) = 0.00901 × 编码器读数 |

## 硬件平台

### 主控芯片

**龙芯 LS2K0300** — 双核 LoongArch64 处理器，主频约 1 GHz。运行 Linux 内核 4.19，板级外设驱动由 **成都逐飞科技 (SEEKFREE)** 提供。

### 外设清单

| 外设 | 接口 | 用途 |
|---|---|---|
| USB 摄像头 (UVC) | USB | 赛道图像采集 |
| 直流减速电机 × 2 | PWM + GPIO 方向 | 差速转向驱动 |
| 霍尔编码器 × 2 | 正交编码输入 | 轮速测量 |
| IMU (ICM-42688-P) | SPI/I2C | 角速度反馈、碰撞检测 |
| 有源蜂鸣器 | GPIO | 状态告警与元素音效反馈 |
| IPS200 LCD | SPI 帧缓冲 | 实时图像与状态显示 |
| ADC 分压采集 | ADC | 电池电压监测 |
| 按键 / 波动开关 | GPIO | 模式选择、复位 |

### 设备节点

```
/dev/pwm_servo       舵机 PWM
/dev/pwm_motor1      电机 1 PWM
/dev/pwm_motor2      电机 2 PWM
/dev/gpio_motor1_dir 电机 1 方向
/dev/gpio_motor2_dir 电机 2 方向
/dev/encoder1        编码器 1
/dev/encoder2        编码器 2
/dev/gpio_beep       蜂鸣器
/dev/gpio_key_0      按键 0（启动 / 复位）
/dev/gpio_key_1      按键 1
/dev/gpio_key_2      按键 2（配置选择 1）
/dev/gpio_key_3      按键 3（配置选择 2）
/dev/fb0             IPS200 帧缓冲
```

---

## 目录结构

```
.
├── main.cpp                     # 入口：主控制循环、硬件初始化、线程管理
├── CMakeLists.txt               # CMake 构建（支持边缘 / 主机双模式）
├── upload.sh                    # 部署脚本
│
├── src/
│   ├── common/                  # 核心运行时、配置解析、工具函数
│   │   ├── main_runtime.cpp     #   init、参数解析、测试任务
│   │   ├── AAAtools.cpp         #   通用工具
│   │   ├── zf_common_font.cpp   #   IPS200 字体库 (SEEKFREE)
│   │   └── zf_common_function.cpp
│   │
│   ├── control/                 # 运动控制系统
│   │   ├── MotorControlTask.cpp #   级联 PID 控制线程（外 PD + 内 PI + 速度 PI）
│   │   ├── DualMotorController.cpp  # 双电机协调
│   │   ├── MotorController.cpp  #   单电机 PWM 控制
│   │   └── PID.cpp              #   PID 算法库
│   │
│   ├── devices/                 # 物理设备接口
│   │   ├── encoder.cpp          #   编码器速度读取
│   │   ├── buzzer.cpp           #   蜂鸣器控制（支持音效模式）
│   │   ├── display_show.cpp     #   IPS200 LCD 显示
│   │   ├── zf_device_imu_core.cpp   # IMU 传感器
│   │   └── zf_device_ips200_fb.cpp  # IPS200 帧缓冲驱动 (SEEKFREE)
│   │
│   ├── drivers/                 # 底层硬件驱动 (SEEKFREE zf_ 库)
│   │   ├── zf_driver_adc.cpp
│   │   ├── zf_driver_encoder.cpp
│   │   ├── zf_driver_gpio.cpp
│   │   ├── zf_driver_pit.cpp
│   │   ├── zf_driver_pwm.cpp
│   │   ├── zf_driver_udp.cpp
│   │   └── zf_driver_file.cpp
│   │
│   ├── network/                 # 网络通信
│   │   └── seekfree_udp.cpp     #   SEEKFREE UDP 协议（图像 + 边界上传）
│   │
│   └── vision/                  # 计算机视觉管道
│       ├── Image_Process.cpp    #   核心视觉算法：二值化、边界跟踪、元素检测
│       ├── main_impl.cpp        #   视觉管线的集成调用
│       ├── vision_rings.cpp     #   环岛 / 圆环元素状态机
│       ├── libdata_process.cpp  #   决策层：速度查表、过渡区扫描
│       ├── image_display.cpp    #   图像标注与可视化
│       └── camera_capture.cpp   #   摄像头双缓冲采集线程
│
├── include/                     # 头文件（结构与 src/ 对应）
│   └── model/                   #   TensorFlow Lite 模型
│       ├── loong_cnn_model_simple.h
│       └── loong_cnn_model_simple.tflite
│
├── config/                      # 配置文件
│   ├── config_0.json            #   保守配置（基础速度 60）
│   ├── config_1.json            #   均衡配置（基础速度 75）
│   ├── config_2.json            #   激进配置（基础速度 90）
│   └── calibration.yaml         #   摄像头标定参数
│
├── tools/
│   ├── image_web_test_main.cpp  # 离线图像处理 Web 测试工具
│   └── tflite_camera_classic_main.cpp  # TFLite 摄像头推理测试
│
├── third_party/
│   ├── cpp-httplib-master/      # C++ HTTP 服务器（头文件库）
│   ├── json-develop/            # nlohmann/json
│   ├── ncnn/                    # 腾讯 ncnn 神经网络推理框架
│   ├── tflm/                    # TensorFlow Lite Micro
│   └── seekfree_assistant/      # SEEKFREE 上位机协议库
│
├── docs/
│   ├── vision_architecture.md   # 视觉算法详细文档（806 行）
│   ├── AI提示词/                #  AI 辅助开发记录
│   ├── img/                     #  离线测试图像数据集
│   ├── LS2K0300_Library/        #  SEEKFREE LS2K0300 库参考
│   └── ...
│
├── opencv_4_10_build/           # 交叉编译的 OpenCV 4.10 库
├── ls2k0300_linux_4.19/         # LS2K0300 Linux 4.19 内核源码
└── build/ & build_host/         # 构建输出目录
```

---

## 构建指南

### 依赖

- **CMake** ≥ 3.16
- **OpenCV** 4.x（边缘端使用 `opencv_4_10_build/` 预编译库，上位机使用系统 OpenCV）
- **C++17** 编译器
- **LoongArch 交叉工具链**（仅边缘构建）：`/opt/loongarch-gnu-toolchain/`

### 构建选项

构建系统通过 `USE_EDGE_TOOLCHAIN` 控制目标平台：

| 选项 | 说明 | 构建目标 |
|---|---|---|
| `USE_EDGE_TOOLCHAIN=ON` | 边缘端交叉编译（LoongArch64） | `main` |
| `USE_EDGE_TOOLCHAIN=OFF` | 上位机本机构建 | `main`、`image_web_test` |

> 若检测到编译器为 `loongarch64`，CMake 会自动强制开启边缘模式，防止链接到主机 OpenCV 库。

### 边缘设备构建（LS2K0300）

```sh
# 在仓库根目录执行
cmake -S . -B build -DUSE_EDGE_TOOLCHAIN=ON
cmake --build build -j$(nproc) --target main
```

生成的可执行文件位于 `build/main`。

### 上位机本机构建（x86）

```sh
cmake -S . -B build_host -DUSE_EDGE_TOOLCHAIN=OFF
cmake --build build_host -j$(nproc)
```

生成 `build_host/main` 和 `build_host/image_web_test`。

---

## 快速开始

### 边缘端运行

在 LS2K0300 开发板上：

```sh
# 选择配置：0 / 1 / 2（通过 stdin 输入）
echo 0 | ./main

# 或带参数（摄像帧率 60 FPS，开启帧率显示）
./main --camera-fps 60 --show-fps
```

支持的命令行参数：

| 参数 | 说明 |
|---|---|
| `-s, --straight-speed <v>` | 覆盖直线速度 |
| `--camera-fps <fps>` | 设置摄像头帧率 |
| `--show-fps` | 显示帧率 |
| `--speed-test` | 速度测试模式 |
| `--dead-zone-calib` | 电机死区校准 |
| `--test-buzzer` | 蜂鸣器测试 |
| `--test-imu` | IMU 测试 |
| `--test-motor` | 电机测试 |
| `--test-encoder` | 编码器测试 |

### 上位机 Web 图像测试

该工具用于在服务器或上位机上离线测试图像处理管线，不依赖边缘硬件。

```sh
# 编译
cmake -S . -B build_host -DUSE_EDGE_TOOLCHAIN=OFF
cmake --build build_host -j 2 --target image_web_test

# 运行（指定图像数据集和配置文件）
./build_host/image_web_test --dataset docs/img/20260409_141917 --config config/config_0.jsonc --port 8090

# 浏览器访问
http://127.0.0.1:8090
```

> 远程服务器访问时，将 `127.0.0.1` 替换为服务器实际 IP。

---

## 系统架构

### 主控制循环

每帧的处理流水线：

```
摄像头采集 ──> 图像预处理 ──> 二值化 ──> 边界跟踪 ──> 元素识别 ──> 偏差计算 ──> 差速控制
    │               │           │           │            │           │            │
    └── 双缓冲区    裁剪+缩放   OTSU       8邻域搜索   十字/环岛   加权中心   外PD+内PI+
                                 自适应阈值  左右边界    过渡区/弯道  偏移量     速度增量PI
```

主循环在 `main.cpp` 中的 6 个步骤：

1. **`FrameTaskAfterRead()`** — 从双缓冲获取最新图像帧，裁剪有效区域并缩放
2. **`ImageProcess()`** — OTSU 二值化 + 边界跟踪 + 元素识别 + 偏差计算
3. **`GetDet()`** — 从左右边界提取加权中心偏移量（像素）
4. **`MotorSpeed_Judge()`** — 根据赛道类型查表确定目标速度
5. **`ApplyDifferentialControl()`** — 将像素偏差映射为控制误差，下发至控制线程
6. **状态更新** — 蜂鸣器反馈、IPS200 显示、UDP 上传

### 视觉管道

核心视觉算法（`src/vision/Image_Process.cpp`，~1400 行）包含：

- **图像预处理**：320×240 → 裁剪 160×60 → 缩放到 80×60
- **二值化**：改进的 OTSU 自适应阈值算法，带有边缘区域增强
- **边界跟踪**：8 邻域搜索法，从图像底部向顶部逐行扫描左右赛道边界
- **元素识别**：
  - **十字**：边界宽度异常收缩 + 过渡区检测
  - **环岛**：左右边界非对称变化 + 专有状态机处理（`vision_rings.cpp`）
  - **弯道**：边界曲率 + 拐点检测
  - **坡道 / 岔路 / 车库**：边界模式分类
- **偏差计算**：左右边界加权中心线与图像中线的像素偏移

详细算法文档见 `docs/vision_architecture.md`。

### 控制系统

采用**三级级联 PID** 架构，控制周期 10 ms：

```
外环 (PD)                内环 (PI)                速度环 (增量 PI)
位置偏差 ──→ 目标角速度 ──→ 目标角加速度 ──→ 目标 PWM ──→ 电机
   ↑                        ↑                        ↑
IMU 实测角速度            编码器实测速度
```

| 环 | 类型 | 输入 | 输出 | 作用 |
|---|---|---|---|---|
| 外环 | PD | 图像中心偏移（像素） | 目标角速度 | 循迹方向控制 |
| 内环 | PI | IMU 实测角速度 | 目标角加速度 | 角速度跟踪、抑制抖动 |
| 速度环 | 增量 PI | 编码器实测速度 | PWM 占空比 | 速度维持与差速实现 |

附加特性：低通滤波、斜坡限幅、死区补偿、曲率自适应减速、碰撞保护（IMU 冲击 + 电机堵转检测）。

### 通信

- **UDP 上行**（边缘 → 上位机）：通过 SEEKFREE 自定义协议发送灰度图像 + 左/中/右三线边界数据
- **上位机协议库**：`third_party/seekfree_assistant/`

---

## 配置文件

### JSON 配置

系统使用 JSON 配置文件管理所有可调参数，位于 `config/` 目录。主要参数分类：

#### PID 参数

| 参数 | 说明 |
|---|---|
| `DIFF_OUTER_PD_KP / KD` | 外环 PD 比例/微分系数 |
| `DIFF_OUTER_PD_LIMIT_*` | 外环输出限幅 |
| `DIFF_INNER_PI_KP / KI` | 内环 PI 比例/积分系数 |
| `DIFF_INNER_PI_LIMIT_*` | 内环输出限幅 |
| `SPEED_INCR_PI_KP / KI` | 速度增量 PI 系数 |

#### 速度参数

| 参数 | 说明 |
|---|---|
| `STRIGHT_TRACK_MOTOR_SPEED` | 直线段目标速度 |
| `CURVATURE_SPEED_GAIN` | 弯道减速增益 |
| `MOTOR_MAX_DUTY` | PWM 最大占空比 (%) |
| `CONTROL_PERIOD` | 控制周期 (s) |

#### 特征开关

| 参数 | 说明 |
|---|---|
| `CAMERA_EN` | 摄像头使能 |
| `IMAGE_SAVE_EN` | 图像保存 |
| `ACROSS_IDENTIFY_EN` | 十字识别 |
| `CIRCLE_IDENTIFY_EN` | 环岛识别 |
| `IPS200_SHOW_EN` | IPS200 显示 |
| `UDP_IMAGE_UPLOAD_EN` | UDP 图像上传 |

### 预置配置

| 文件 | 直线速度 | 外环 PD | 内环 PI | 路宽 | 定位 |
|---|---|---|---|---|---|
| `config_0.json` | 60 | Kp=4.5, Kd=0.6 | Kp=2.0, Ki=4.0 | 225 | 保守调校 |
| `config_1.json` | 75 | Kp=10.0, Kd=2.0 | Kp=12.0, Ki=0.6 | 216 | 均衡调校 |
| `config_2.json` | 90 | Kp=12.0, Kd=2.5 | Kp=15.0, Ki=0.8 | 216 | 激进调校 |

通过拨码开关或 stdin 在运行时选择配置文件。

### 摄像头标定

摄像头标定参数（`config/calibration.yaml`）：

- 分辨率：320 × 240
- 内参矩阵：fx≈150, fy≈150, cx≈161, cy≈114
- 畸变系数：k1≈0.203, k2≈-0.181, p1≈0.003
- 重投影误差：0.382 像素

---

## 测试与调试

### 1. Web 图像测试工具

上位机离线测试视觉管线。加载数据集图像，运行完整视觉流水线，通过浏览器查看二值化结果、边界覆盖和元素识别标注。

```sh
./build_host/image_web_test --dataset <图像目录> --config <配置文件.jsonc> --port <端口>
```

### 2. IPS200 LCD 实时显示

车模运行时，IPS200 屏幕显示：
- 80×60 二值化图像
- 左右赛道边界覆盖
- 当前赛道类型识别结果
- 目标速度与实时速度

### 3. SEEKFREE UDP 上位机

通过 UDP 将灰度图像与三线边界数据实时上传至 SEEKFREE 地面站上位机，支持在电脑端监控摄像头视野与边界跟踪效果。

### 4. 在线参数调试

运行中通过 GPIO 按键或命令行参数切换配置、调节关键参数。

---

## 安全机制

| 机制 | 实现方式 | 触发行为 |
|---|---|---|
| 碰撞检测 | IMU 冲击阈值 (>3.0g) + 电机堵转检测 | 紧急停车，蜂鸣器告警 |
| 出界保护 | 边界完全丢失判定 | 紧急停车 + 蜂鸣器双音告警，等待 KEY_0 手动复位 |
| 电池低电压 | ADC 读数 < 10.0V 阈值 | 蜂鸣器周期性告警 |
| 斜坡限幅 | 输出变化率限制 | 防止电机/控制量突变 |
| PWM 死区补偿 | 左右电机独立死区消除 | 消除零速附近的非线性区 |

---

## 参考资料

| 文档 | 说明 |
|---|---|
| `docs/vision_architecture.md` | 视觉算法详细说明（二值化、边界跟踪、元素识别、状态机） |
| `docs/AI提示词/` | AI 辅助开发过程中的思路记录与策略讨论 |
| `docs/LS2K0300_Library/` | SEEKFREE LS2K0300 底层库参考文档 |
| `docs/opencv-4.10.0/` | OpenCV 4.10 交叉编译说明 |
| `include/control/PID.hpp` | PID 算法库文档（标准、增量、死区、前馈） |

### 三方库

| 库 | 用途 | 协议 |
|---|---|---|
| [cpp-httplib](https://github.com/yhirose/cpp-httplib) | HTTP 服务器 | MIT |
| [nlohmann/json](https://github.com/nlohmann/json) | JSON 解析 | MIT |
| [ncnn](https://github.com/Tencent/ncnn) | 神经网络推理框架 | BSD-3 |
| TensorFlow Lite Micro | 边缘端模型推理 | Apache 2.0 |
| SEEKFREE zf_ 驱动库 | LS2K0300 板级外设驱动 | GPL 3.0 |

---

## 开源协议

本项目基于 [GNU General Public License v3.0](LICENSE) 发布。

项目包含 SEEKFREE（成都逐飞科技）的 LS2K0300 底层驱动库及 SDK，同样遵循 GPL v3.0 协议。
