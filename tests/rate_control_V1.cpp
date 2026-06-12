#include "rate_control.hpp"
#include <atomic>
#include <csignal>
#include <iostream>

using namespace rate_control;

static std::atomic<bool> g_stop{false};

static void on_sigint(int) {
    g_stop.store(true);
}

int main() {
    std::signal(SIGINT, on_sigint);

    Scheduler sch;

    // 假装这些是你的业务函数（建议短小、非阻塞）
    sch.add_task("motor_control_200hz", 200.0, [](){
        // 电机PWM/闭环控制（高频）
        // TODO: write_motor_pwm();
    }, MissPolicy::CatchUp);

    sch.add_task("sensor_100hz", 100.0, [](){
        // IMU/里程计/超声/ToF等
        // TODO: read_sensors();
    }, MissPolicy::CatchUp);

    sch.add_task("image_read_30hz", 30.0, [](){
        // 摄像头取帧（尽量用非阻塞/双缓冲）
        // TODO: camera_grab();
    }, MissPolicy::Skip); // 取帧更适合 Skip：落后就丢帧，保证“新鲜度”

    sch.add_task("perception_15hz", 15.0, [](){
        // 目标识别/语义分割等
        // TODO: run_inference();
    }, MissPolicy::Skip); // 感知任务也常用 Skip，避免补课导致延迟越来越大

    sch.add_task("path_detect_20hz", 20.0, [](){
        // 车道线/路径检测
        // TODO: detect_path();
    }, MissPolicy::CatchUp);

    sch.add_task("display_10hz", 10.0, [](){
        // UI/OSD/串口屏等
        // TODO: render_display();
    }, MissPolicy::Skip);

    sch.add_task("upload_1hz", 1.0, [](){
        // 上传遥测数据/日志
        // TODO: upload_telemetry();
    }, MissPolicy::Skip);

    // 每2秒打印一次统计信息（不建议太频繁打印，影响实时性）
    Rate print_rate(0.5, MissPolicy::Skip);
    print_rate.reset();

    std::cout << "Smartcar scheduler running. Press Ctrl+C to stop.\n";

    sch.add_task("stats_printer_0.5hz", 0.5, [&](){
        std::cout << "\n--- Stats ---\n";
        for (const auto& t : sch.tasks()) {
            std::cout << t.name
                      << " cycles=" << t.stats.cycles
                      << " overruns=" << t.stats.overruns
                      << " max_late_ns=" << t.stats.max_late_ns
                      << "\n";
        }
    }, MissPolicy::Skip);

    sch.run([](){ return g_stop.load(); });

    std::cout << "Stopped.\n";
    return 0;
}
