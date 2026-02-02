#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace rate_control {

using Clock = std::chrono::steady_clock;

enum class MissPolicy : uint8_t {
    CatchUp, // 追赶：next += period，可能连续执行几次（不sleep）追到当前
    Skip     // 跳过：next = now + period，避免“补课”导致抖动/堆积
};

struct RateStats {
    uint64_t cycles = 0;        // 总周期数
    uint64_t overruns = 0;      // 超时次数（回调耗时 > period 或错过deadline）
    uint64_t max_late_ns = 0;   // 最大延迟（ns）
};

class Rate {
public:
    // hz: 频率（Hz），policy: 错过周期策略
    explicit Rate(double hz, MissPolicy policy = MissPolicy::CatchUp);

    // 设定下一次周期基准：通常在循环开始前调用一次即可
    void reset(Clock::time_point start = Clock::now());

    // 等待到下一个周期点；返回是否发生了 overrun（本周期已经落后）
    bool sleep();

    // 本周期结束时刻（下一次触发点）
    Clock::time_point next_deadline() const { return next_; }

    // 周期时长
    std::chrono::nanoseconds period() const { return period_; }

    const RateStats& stats() const { return stats_; }

private:
    std::chrono::nanoseconds period_;
    Clock::time_point next_{};
    MissPolicy policy_;
    RateStats stats_{};
};

struct Task {
    std::string name;
    std::chrono::nanoseconds period;
    MissPolicy policy = MissPolicy::CatchUp;
    std::function<void()> fn;

    // 运行时字段
    Clock::time_point next;
    RateStats stats;
};

// 将频率（Hz）转换为周期（纳秒）
std::chrono::nanoseconds hz_to_period(double hz);

class Scheduler {
public:
    Scheduler() = default;

    // hz: 频率；name: 用于调试统计
    void add_task(const std::string& name, double hz, std::function<void()> fn,
                  MissPolicy policy = MissPolicy::CatchUp);

    // 运行调度循环（单核协作式）。stop_flag 返回 true 时停止。
    // 建议：回调里不要长时间阻塞；耗时任务可拆小步或用队列/状态机。
    void run(const std::function<bool()>& stop_flag);

    const std::vector<Task>& tasks() const { return tasks_; }

private:
    std::vector<Task> tasks_;
};

} // namespace rate_control
