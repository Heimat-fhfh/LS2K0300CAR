#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rate_control {

using Clock = std::chrono::steady_clock;

enum class MissPolicy : uint8_t {
    CatchUp, // 追赶：next += period，可能连续执行几次（不sleep）追到当前
    Skip     // 跳过：next = now + period，避免"补课"导致抖动/堆积
};

enum class OverrunAction : uint8_t {
    None,    // 不做特殊处理（继续执行）
    Warn,    // 仅打印警告（继续执行）
    Skip,    // 自动跳过下一次执行
    Callback // 调用用户指定的超时回调
};

struct RateStats {
    uint64_t cycles = 0;        // 总周期数
    uint64_t overruns = 0;      // 超时次数（回调耗时 > period 或错过deadline）
    uint64_t skips = 0;         // 因超时/预算不足被跳过的次数
    uint64_t max_late_ns = 0;   // 最大延迟（ns）
    double avg_exec_us = 0.0;   // 平均执行耗时（微秒）
    uint64_t max_exec_ns = 0;   // 最大执行耗时（ns）
    uint64_t budget_exceeded = 0; // 超出时间预算次数
};

using OverrunCallback = std::function<void(const std::string&, 
                                         std::chrono::nanoseconds, 
                                         const RateStats&)>;

class Rate {
public:
    // hz: 频率（Hz），policy: 错过周期策略
    explicit Rate(double hz, MissPolicy policy = MissPolicy::CatchUp);

    // 设定下一次周期基准：通常在循环开始前调用一次即可
    void reset(Clock::time_point start = Clock::now());

    // 等待到下一个周期点；返回是否发生了 overrun（本周期已经落后）
    bool sleep();

    // 设置时间预算（超过此值会触发 action）
    void set_budget(std::chrono::nanoseconds budget, 
                  OverrunAction action = OverrunAction::Warn);

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
    std::chrono::nanoseconds budget_{};
    OverrunAction budget_action_ = OverrunAction::None;
};

struct Task {
    std::string name;
    std::chrono::nanoseconds period;
    MissPolicy policy = MissPolicy::CatchUp;
    int priority = 0;  // 优先级，越高越先执行
    std::function<void()> fn;
    std::chrono::nanoseconds budget{0}; // 时间预算（超过会触发action）
    OverrunAction budget_action = OverrunAction::None;
    OverrunCallback overrun_cb;

    // 运行时字段
    Clock::time_point next;
    RateStats stats;
    bool skip_next = false;
};

class Scheduler {
public:
    Scheduler();
    ~Scheduler();

    
    static std::chrono::nanoseconds hz_to_period(double hz);
    
    // 禁用拷贝/移动
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    Scheduler(Scheduler&&) = delete;
    Scheduler& operator=(Scheduler&&) = delete;

    // hz: 频率；name: 用于调试统计；priority: 优先级（高优先级优先执行）
    void add_task(const std::string& name, double hz, std::function<void()> fn,
                 MissPolicy policy = MissPolicy::CatchUp, int priority = 0);

    // 为任务设置时间预算
    void set_task_budget(const std::string& name, 
                        std::chrono::nanoseconds budget,
                        OverrunAction action = OverrunAction::Warn);
    
    // 设置超时回调（仅当 OverrunAction::Callback 时才调用）
    void set_overrun_callback(const std::string& name, OverrunCallback cb);
    
    // 设置系统全局超时回调
    void set_global_overrun_callback(OverrunCallback cb);
    
    // 运行调度循环（单核协作式）。stop_flag 返回 true 时停止。
    void run(const std::function<bool()>& stop_flag);

    // 可选：切换到实时调度策略和更高优先级（需要 root 权限）
    // 失败时返回 false 并打印错误信息
    bool set_realtime_priority(int rt_priority = 80);

    // 打印任务统计信息
    void print_stats() const;

    // 清空统计
    void reset_stats();

    const std::vector<Task>& tasks() const { return tasks_; }

private:
    std::vector<Task> tasks_;
    OverrunCallback global_overrun_cb_;
    struct SchedulerImpl;
    std::unique_ptr<SchedulerImpl> pimpl_;

    // 内部方法：找到指定名称的任务
    Task* find_task(const std::string& name);
    
    
    // 处理任务超时
    void handle_overrun(Task& task, std::chrono::nanoseconds late);
    
    // 处理预算超时
    void handle_budget_exceeded(Task& task, std::chrono::nanoseconds exec_time);
};

} // namespace rate_control
