/**
 * @file max_speed_calib.cpp
 * @brief 电机最大速度标定程序
 *
 * 让电机以 100% 占空比满转，同时读取编码器速度值，
 * 从而换算出电机的最大实际速度（m/s），用于设置 MAX_SPEED_MPS。
 *
 * 使用方法：
 *   1. 将小车架起（轮子悬空），确保安全
 *   2. 编译并运行本程序
 *   3. 观察输出，取稳定后的速度平均值作为 MAX_SPEED_MPS
 *
 * 编译（在边缘设备上）：
 *   cd /home/fhfh/Work/LS2K0300CAR
 *   /opt/loongarch-gnu-toolchain/bin/loongarch64-linux-gnu-g++ \
 *       -std=c++17 \
 *       -I include \
 *       test/max_speed_calib.cpp \
 *       src/MotorController.cpp src/DualMotorController.cpp \
 *       src/encoder.cpp \
 *       -o max_speed_calib \
 *       -lpthread
 *
 * 运行：
 *   ./max_speed_calib
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <memory>

#include "MotorController.h"
#include "DualMotorController.h"
#include "encoder.hpp"

// 全局变量，用于信号处理
std::atomic<bool> g_running{true};
std::unique_ptr<DualMotorController> motors;
Encoder* encoder_left = nullptr;
Encoder* encoder_right = nullptr;

void sigint_handler(int signum) {
    (void)signum;
    printf("\n\n收到退出信号，正在停止电机...\n");
    g_running.store(false);
    if (motors) {
        motors->stopAll();
    }
}

void print_usage(const char* prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  -d, --duration SEC    测试持续时间（秒），默认 10\n");
    printf("  -l, --left-only       仅测试左轮\n");
    printf("  -r, --right-only      仅测试右轮\n");
    printf("  -b, --both            同时测试左右轮（默认）\n");
    printf("  -h, --help            显示帮助\n");
    printf("\n");
    printf("说明:\n");
    printf("  让电机以 100%% 占空比满转，持续读取编码器速度值。\n");
    printf("  取稳定后的速度平均值作为 MAX_SPEED_MPS。\n");
    printf("  注意：请务必将小车架起，轮子悬空后再运行！\n");
}

int main(int argc, char* argv[]) {
    // 解析命令行参数
    int test_duration = 10;
    enum { TEST_BOTH, TEST_LEFT, TEST_RIGHT } test_mode = TEST_BOTH;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-d" || arg == "--duration") {
            if (i + 1 < argc) {
                test_duration = std::atoi(argv[++i]);
                if (test_duration <= 0) test_duration = 10;
            }
        } else if (arg == "-l" || arg == "--left-only") {
            test_mode = TEST_LEFT;
        } else if (arg == "-r" || arg == "--right-only") {
            test_mode = TEST_RIGHT;
        } else if (arg == "-b" || arg == "--both") {
            test_mode = TEST_BOTH;
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    // 注册信号处理
    signal(SIGINT, sigint_handler);

    printf("========================================\n");
    printf("   电机最大速度标定程序\n");
    printf("========================================\n");
    printf("\n");
    printf("⚠ 警告：请务必将小车架起，轮子悬空！\n");
    printf("⚠ 按 Ctrl+C 可随时停止测试\n");
    printf("\n");
    printf("测试配置:\n");
    printf("  持续时间: %d 秒\n", test_duration);
    printf("  测试模式: %s\n",
           test_mode == TEST_BOTH ? "左右轮同时" :
           test_mode == TEST_LEFT ? "仅左轮" : "仅右轮");
    printf("\n");
    printf("按回车键开始测试...");
    getchar();

    // 初始化电机
    try {
        motors = std::make_unique<DualMotorController>();
        printf("[OK] 电机控制器初始化成功\n");
    } catch (const std::exception& e) {
        fprintf(stderr, "[FAIL] 电机控制器初始化失败: %s\n", e.what());
        return 1;
    }

    // 初始化编码器
    Encoder left_enc("/dev/zf_encoder_1");
    Encoder right_enc("/dev/zf_encoder_2", true);
    encoder_left = &left_enc;
    encoder_right = &right_enc;

    if (!left_enc.isValid()) {
        fprintf(stderr, "[WARN] 左编码器设备不可访问\n");
    } else {
        printf("[OK] 左编码器初始化成功, 转换系数: %.6f\n", left_enc.conversionFactor());
    }
    if (!right_enc.isValid()) {
        fprintf(stderr, "[WARN] 右编码器设备不可访问\n");
    } else {
        printf("[OK] 右编码器初始化成功, 转换系数: %.6f\n", right_enc.conversionFactor());
    }

    // 设置最大占空比为 100%
    motors->setMaxDutyLimits(100.0f);

    printf("\n开始测试...\n\n");

    // 启动电机满转
    if (test_mode == TEST_BOTH) {
        motors->setSpeeds(1.0f, 1.0f);
        printf("左右轮同时满转 (100%% 占空比)\n");
    } else if (test_mode == TEST_LEFT) {
        motors->setSpeeds(1.0f, 0.0f);
        printf("左轮满转 (100%% 占空比)，右轮停止\n");
    } else {
        motors->setSpeeds(0.0f, 1.0f);
        printf("右轮满转 (100%% 占空比)，左轮停止\n");
    }

    // 数据采集
    const int sample_count = test_duration * 100;  // 100Hz 采样
    double* left_samples = new double[sample_count];
    double* right_samples = new double[sample_count];
    int valid_left_count = 0;
    int valid_right_count = 0;

    printf("采样中...\n");
    printf("  时间(s) | 左轮速度(m/s) | 右轮速度(m/s)\n");
    printf("  --------+---------------+---------------\n");

    auto start_time = std::chrono::steady_clock::now();
    for (int i = 0; i < sample_count && g_running.load(); i++) {
        auto loop_start = std::chrono::steady_clock::now();

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();

        double left_speed = 0.0, right_speed = 0.0;
        bool left_valid = false, right_valid = false;

        try {
            if (left_enc.isValid()) {
                left_speed = left_enc.readSpeed();
                left_valid = true;
            }
        } catch (const std::exception& e) {
            left_speed = 0.0;
        }

        try {
            if (right_enc.isValid()) {
                right_speed = right_enc.readSpeed();
                right_valid = true;
            }
        } catch (const std::exception& e) {
            right_speed = 0.0;
        }

        if (left_valid) {
            left_samples[valid_left_count++] = left_speed;
        }
        if (right_valid) {
            right_samples[valid_right_count++] = right_speed;
        }

        // 每秒打印一次
        if (i % 100 == 0) {
            printf("  %6.1f   | %13.4f | %13.4f\n",
                   elapsed, left_speed, right_speed);
            fflush(stdout);
        }

        // 精确 10ms 周期
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            loop_end - loop_start).count();
        if (elapsed_us < 10000) {
            std::this_thread::sleep_for(std::chrono::microseconds(10000 - elapsed_us));
        }
    }

    // 停止电机
    motors->stopAll();

    printf("\n测试结束，电机已停止\n\n");

    // 计算统计结果
    printf("========================================\n");
    printf("   标定结果\n");
    printf("========================================\n");

    if (test_mode != TEST_RIGHT && valid_left_count > 0) {
        double sum = 0, max_val = -1e9, min_val = 1e9;
        for (int i = 0; i < valid_left_count; i++) {
            sum += std::abs(left_samples[i]);
            if (std::abs(left_samples[i]) > max_val) max_val = std::abs(left_samples[i]);
            if (std::abs(left_samples[i]) < min_val) min_val = std::abs(left_samples[i]);
        }
        double avg = sum / valid_left_count;

        printf("\n左轮:\n");
        printf("  采样点数: %d\n", valid_left_count);
        printf("  平均速度: %.4f m/s\n", avg);
        printf("  最大速度: %.4f m/s\n", max_val);
        printf("  最小速度: %.4f m/s\n", min_val);
        printf("  → 建议 MAX_SPEED_MPS = %.4f\n", avg);
    }

    if (test_mode != TEST_LEFT && valid_right_count > 0) {
        double sum = 0, max_val = -1e9, min_val = 1e9;
        for (int i = 0; i < valid_right_count; i++) {
            sum += std::abs(right_samples[i]);
            if (std::abs(right_samples[i]) > max_val) max_val = std::abs(right_samples[i]);
            if (std::abs(right_samples[i]) < min_val) min_val = std::abs(right_samples[i]);
        }
        double avg = sum / valid_right_count;

        printf("\n右轮:\n");
        printf("  采样点数: %d\n", valid_right_count);
        printf("  平均速度: %.4f m/s\n", avg);
        printf("  最大速度: %.4f m/s\n", max_val);
        printf("  最小速度: %.4f m/s\n", min_val);
        printf("  → 建议 MAX_SPEED_MPS = %.4f\n", avg);
    }

    if (test_mode == TEST_BOTH && valid_left_count > 0 && valid_right_count > 0) {
        double sum_l = 0, sum_r = 0;
        for (int i = 0; i < valid_left_count; i++) sum_l += std::abs(left_samples[i]);
        for (int i = 0; i < valid_right_count; i++) sum_r += std::abs(right_samples[i]);
        double avg_l = sum_l / valid_left_count;
        double avg_r = sum_r / valid_right_count;
        double overall = (avg_l + avg_r) / 2.0;
        printf("\n综合建议:\n");
        printf("  左右轮平均: %.4f m/s\n", overall);
        printf("  → 建议设置 MAX_SPEED_MPS = %.4f\n", overall);
    }

    printf("\n========================================\n");
    printf("请将 MAX_SPEED_MPS 值更新到\n");
    printf("  src/MotorControlTask.cpp 中的\n");
    printf("  constexpr double MAX_SPEED_MPS = ...;\n");
    printf("========================================\n");

    // 清理
    delete[] left_samples;
    delete[] right_samples;

    return 0;
}

