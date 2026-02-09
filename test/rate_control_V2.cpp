#include "rate_control.hpp"
#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

using namespace rate_control;

static std::atomic<bool> g_stop{false};

static void on_sigint(int) {
    g_stop.store(true);
    std::cout << "Stopping scheduler...\n";
}

void onOverrun(const std::string& name, std::chrono::nanoseconds overage, const RateStats& stats) {
    std::cerr << "OVERRUN: Task '" << name 
              << "' exceeded budget by " << overage.count() / 1000.0
              << "µs (cycles=" << stats.cycles 
              << ", overruns=" << stats.overruns << ")\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, on_sigint);

    bool use_realtime = false;
    if (argc > 1 && std::string(argv[1]) == "--rt") {
        use_realtime = true;
    }

    Scheduler sch;
    
    // 设置全局超时回调
    sch.set_global_overrun_callback(onOverrun);

    // 电机控制：最高优先级(10)，CatchUp 策略确保稳定性
    sch.add_task("motor_control_200hz", 200.0, [](){
        // 模拟 PWM 控制
        std::this_thread::sleep_for(std::chrono::microseconds(300));
    }, rate_control::MissPolicy::CatchUp, 10);
    // 设置时间预算 1ms，超出时发出警告
    sch.set_task_budget("motor_control_200hz", 
                       std::chrono::milliseconds(1), 
                       OverrunAction::Warn);

    // 传感器：优先级8，CatchUp保证数据连续性
    sch.add_task("sensor_100hz", 100.0, [](){
        // 读取IMU/传感器
        std::this_thread::sleep_for(std::chrono::microseconds(500)); 
    }, MissPolicy::CatchUp, 8);
    sch.set_task_budget("sensor_100hz", 
                       std::chrono::milliseconds(2), 
                       OverrunAction::Warn);

    // 图像读取：优先级6，Skip策略避免堵塞
    sch.add_task("image_read_30hz", 30.0, [](){
        // 模拟相机取帧
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }, MissPolicy::Skip, 6);
    sch.set_task_budget("image_read_30hz", 
                      std::chrono::milliseconds(10),
                      OverrunAction::Skip); // 超预算自动跳过下一次

    // 目标识别：优先级4，计算密集型任务
    sch.add_task("perception_15hz", 15.0, [](){
        // 模拟CNN推理
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }, MissPolicy::Skip, 4);
    sch.set_task_budget("perception_15hz", 
                      std::chrono::milliseconds(30),
                      OverrunAction::Callback); // 超时调用回调
    sch.set_overrun_callback("perception_15hz", 
                           [](const std::string& name, 
                              std::chrono::nanoseconds overage, 
                              const RateStats& stats) {
        std::cerr << "⚠️ AI推理超时: " << overage.count() / 1000000.0 
                  << "ms, 考虑降低模型复杂度\n";
    });

    // 路径检测：优先级5
    sch.add_task("path_detect_20hz", 20.0, [](){
        // 车道/路径检测
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }, MissPolicy::CatchUp, 5);
    sch.set_task_budget("path_detect_20hz", 
                      std::chrono::milliseconds(20), 
                      OverrunAction::Warn);

    // 显示：优先级2（非关键）
    sch.add_task("display_10hz", 10.0, [](){
        // UI渲染
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }, MissPolicy::Skip, 2);
    
    // 数据上传：最低优先级1
    sch.add_task("upload_1hz", 1.0, [](){
        // 模拟上传数据到服务器
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }, MissPolicy::Skip, 1);

    // 统计打印器：优先级3，每2秒打印一次
    sch.add_task("stats_printer_0.5hz", 0.5, [&](){
        sch.print_stats();
    }, MissPolicy::Skip, 3);

    // 可选：启用实时调度（需要root权限）
    if (use_realtime) {
        if (sch.set_realtime_priority(80)) {
            std::cout << "✓ Running with SCHED_FIFO realtime priority\n";
        }
    } else {
        std::cout << "Running with normal priority. Use --rt for realtime mode.\n";
    }
    
    std::cout << "Smartcar scheduler running. Press Ctrl+C to stop.\n";
    std::cout << "Tasks ordered by priority (highest first):\n";
    for (const auto& t : sch.tasks()) {
        std::cout << " - " << t.name << " (priority=" << t.priority << ")\n";
    }
    
    // 启动调度器
    sch.run([](){ return g_stop.load(); });

    std::cout << "Final statistics:\n";
    sch.print_stats();
    std::cout << "Stopped.\n";
    return 0;
}
