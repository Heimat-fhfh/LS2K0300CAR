#include "rate_control.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <sstream>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#endif

namespace rate_control {

static inline uint64_t to_ns_u64(std::chrono::nanoseconds ns) {
    return static_cast<uint64_t>(ns.count() < 0 ? 0 : ns.count());
}

struct Scheduler::SchedulerImpl {
    // 保留用于实现隐藏的内部功能
    bool is_realtime = false;
    int rt_priority = 0;
};

Scheduler::Scheduler() : pimpl_(std::make_unique<SchedulerImpl>()) {}
Scheduler::~Scheduler() = default;

std::chrono::nanoseconds Scheduler::hz_to_period(double hz) {
    if (hz <= 0.0) {
        return std::chrono::seconds(1);
    }
    const double sec = 1.0 / hz;
    const auto ns = static_cast<int64_t>(sec * 1e9);
    return std::chrono::nanoseconds(std::max<int64_t>(1, ns));
}

Rate::Rate(double hz, MissPolicy policy)
    : period_(Scheduler::hz_to_period(hz)), policy_(policy) {
    reset();
}

void Rate::reset(Clock::time_point start) {
    next_ = start + period_;
    stats_ = RateStats{};
}

bool Rate::sleep() {
    stats_.cycles++;

    const auto now = Clock::now();
    bool overrun = false;

    if (now < next_) {
        std::this_thread::sleep_until(next_);
        next_ += period_;
        return false;
    }

    // 已经落后（错过 deadline）
    overrun = true;
    stats_.overruns++;

    const auto late = now - next_;
    stats_.max_late_ns = std::max<uint64_t>(stats_.max_late_ns, to_ns_u64(std::chrono::duration_cast<std::chrono::nanoseconds>(late)));

    if (policy_ == MissPolicy::CatchUp) {
        // 追赶到 now 之后的第一个 next_
        do {
            next_ += period_;
        } while (next_ <= now);
    } else {
        // Skip：直接跳到 now + period
        next_ = now + period_;
    }

    return overrun;
}

void Rate::set_budget(std::chrono::nanoseconds budget, OverrunAction action) {
    budget_ = budget;
    budget_action_ = action;
}

void Scheduler::add_task(const std::string& name, double hz, std::function<void()> fn,
                         MissPolicy policy, int priority) {
    Task t;
    t.name = name;
    t.period = hz_to_period(hz);
    t.policy = policy;
    t.priority = priority;
    t.fn = std::move(fn);
    t.next = Clock::now() + t.period;
    tasks_.push_back(std::move(t));
}

Task* Scheduler::find_task(const std::string& name) {
    for (auto& task : tasks_) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

void Scheduler::set_task_budget(const std::string& name, 
                              std::chrono::nanoseconds budget,
                              OverrunAction action) {
    auto task = find_task(name);
    if (task) {
        task->budget = budget;
        task->budget_action = action;
    } else {
        std::cerr << "Warning: Task '" << name << "' not found for budget setting\n";
    }
}

void Scheduler::set_overrun_callback(const std::string& name, OverrunCallback cb) {
    auto task = find_task(name);
    if (task) {
        task->overrun_cb = cb;
    } else {
        std::cerr << "Warning: Task '" << name << "' not found for callback setting\n";
    }
}

void Scheduler::set_global_overrun_callback(OverrunCallback cb) {
    global_overrun_cb_ = cb;
}

void Scheduler::handle_overrun(Task& task, std::chrono::nanoseconds late) {
    // 特定任务的回调优先于全局回调
    if (task.budget_action == OverrunAction::Callback && task.overrun_cb) {
        task.overrun_cb(task.name, late, task.stats);
    } else if (task.budget_action == OverrunAction::Callback && global_overrun_cb_) {
        global_overrun_cb_(task.name, late, task.stats);
    } else if (task.budget_action == OverrunAction::Warn) {
        std::cerr << "Warning: Task '" << task.name << "' missed deadline by " 
                  << late.count() / 1000.0 << "µs\n";
    } else if (task.budget_action == OverrunAction::Skip) {
        task.skip_next = true;
        task.stats.skips++;
    }
}

void Scheduler::handle_budget_exceeded(Task& task, std::chrono::nanoseconds exec_time) {
    task.stats.budget_exceeded++;
    
    auto over = exec_time - task.budget;
    
    if (task.budget_action == OverrunAction::Callback && task.overrun_cb) {
        task.overrun_cb(task.name, over, task.stats);
    } else if (task.budget_action == OverrunAction::Callback && global_overrun_cb_) {
        global_overrun_cb_(task.name, over, task.stats);
    } else if (task.budget_action == OverrunAction::Warn) {
        std::cerr << "Warning: Task '" << task.name << "' exceeded budget by " 
                  << over.count() / 1000.0 << "µs\n";
    } else if (task.budget_action == OverrunAction::Skip) {
        task.skip_next = true;
        task.stats.skips++;
    }
}

void Scheduler::print_stats() const {
    std::cout << "\n--- Task Statistics ---\n";
    for (const auto& t : tasks_) {
        std::cout << t.name
                  << " (priority=" << t.priority << ")"
                  << "\n  cycles=" << t.stats.cycles
                  << ", overruns=" << t.stats.overruns
                  << ", skips=" << t.stats.skips
                  << "\n  max_late=" << (t.stats.max_late_ns / 1000.0) << "µs"
                  << ", avg_exec=" << t.stats.avg_exec_us << "µs"
                  << ", max_exec=" << (t.stats.max_exec_ns / 1000.0) << "µs"
                  << "\n  budget_exceeded=" << t.stats.budget_exceeded
                  << (t.budget.count() > 0 ? 
                      ", budget=" + std::to_string(t.budget.count() / 1000.0) + "µs" : "")
                  << "\n";
    }
    std::cout << "----------------------\n";
}

void Scheduler::reset_stats() {
    for (auto& task : tasks_) {
        task.stats = RateStats{};
    }
}

bool Scheduler::set_realtime_priority(int rt_priority) {
#ifdef __linux__
    if (rt_priority < 1 || rt_priority > 99) {
        std::cerr << "Invalid RT priority: " << rt_priority 
                  << " (must be 1-99)\n";
        return false;
    }
    
    struct sched_param param;
    param.sched_priority = rt_priority;
    
    // 设置FIFO实时调度策略
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "Failed to set realtime priority. Error: " 
                  << strerror(errno) << "\n";
        std::cerr << "Hint: Run with sudo or set capabilities with:\n"
                  << "  sudo setcap cap_sys_nice=eip <executable>\n";
        return false;
    }
    
    pimpl_->is_realtime = true;
    pimpl_->rt_priority = rt_priority;
    return true;
#else
    std::cerr << "Realtime scheduling not supported on this platform\n";
    return false;
#endif
}

void Scheduler::run(const std::function<bool()>& stop_flag) {
    while (!stop_flag()) {
        if (tasks_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 找到下一个需要执行的任务（考虑优先级）
        Task* next_task = nullptr;
        Clock::time_point earliest = Clock::time_point::max();

        for (auto& task : tasks_) {
            if (task.skip_next) {
                // 这个任务被标记为跳过，重置标记并更新下次执行时间
                task.skip_next = false;
                task.next = Clock::now() + task.period;
                continue;
            }
            
            if (!next_task || 
                // 相同截止期限，优先级高的先执行
                (task.next == earliest && task.priority > next_task->priority) || 
                // 不同截止期限，截止期限早的先执行
                (task.next < earliest)) {
                next_task = &task;
                earliest = task.next;
            }
        }

        if (!next_task) {
            // 所有任务被跳过，睡眠一小段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto now = Clock::now();
        if (now < next_task->next) {
            // 如果有时间，就睡到下一次执行点
            std::this_thread::sleep_until(next_task->next);
        }

        // 可能 sleep 后时间变化
        const auto tnow = Clock::now();

        // 统计：是否超期
        next_task->stats.cycles++;

        bool missed = (tnow > next_task->next);
        if (missed) {
            next_task->stats.overruns++;
            const auto late = tnow - next_task->next;
            const auto late_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(late);
            next_task->stats.max_late_ns = std::max<uint64_t>(
                next_task->stats.max_late_ns,
                to_ns_u64(late_ns));
            
            // 处理超时（可能会跳过执行）
            if (next_task->budget_action != OverrunAction::None) {
                handle_overrun(*next_task, late_ns);
                if (next_task->skip_next) {
                    // 更新任务的下次执行时间并跳过本次
                    if (next_task->policy == MissPolicy::CatchUp) {
                        do {
                            next_task->next += next_task->period;
                        } while (next_task->next <= Clock::now());
                    } else {
                        next_task->next = Clock::now() + next_task->period;
                    }
                    next_task->skip_next = false;
                    continue;
                }
            }
        }

        // 执行任务，并测量执行时间
        const auto begin = Clock::now();
        
        // 如果有时间预算限制，使用预算保护执行
        if (next_task->budget.count() > 0) {
            try {
                next_task->fn();
            } catch (const std::exception& e) {
                std::cerr << "Exception in task '" << next_task->name 
                          << "': " << e.what() << "\n";
            } catch (...) {
                std::cerr << "Unknown exception in task '" << next_task->name << "'\n";
            }
        } else {
            // 无预算限制的普通执行
            try {
                next_task->fn();
            } catch (const std::exception& e) {
                std::cerr << "Exception in task '" << next_task->name 
                          << "': " << e.what() << "\n";
            } catch (...) {
                std::cerr << "Unknown exception in task '" << next_task->name << "'\n";
            }
        }
        
        // 计算执行时间
        const auto end = Clock::now();
        const auto exec_time = end - begin;
        const auto exec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(exec_time);
        
        // 更新统计信息
        next_task->stats.max_exec_ns = std::max<uint64_t>(
            next_task->stats.max_exec_ns, to_ns_u64(exec_ns));
            
        // 更新平均执行时间（指数移动平均）
        const double alpha = 0.1; // 平滑因子
        const double exec_us = exec_ns.count() / 1000.0;
        if (next_task->stats.cycles == 1) {
            next_task->stats.avg_exec_us = exec_us;
        } else {
            next_task->stats.avg_exec_us = 
                alpha * exec_us + (1.0 - alpha) * next_task->stats.avg_exec_us;
        }
        
        // 如果回调耗时超过预算，处理预算超出
        if (next_task->budget.count() > 0 && exec_ns > next_task->budget) {
            handle_budget_exceeded(*next_task, exec_ns);
        }
        
        // 如果回调耗时超过周期，也算一种 overrun
        if (exec_ns > next_task->period) {
            next_task->stats.overruns++;
            next_task->stats.max_late_ns = std::max<uint64_t>(
                next_task->stats.max_late_ns, 
                to_ns_u64(exec_ns - next_task->period));
        }

        // 更新任务的下次执行时间
        const auto after = Clock::now();
        if (next_task->policy == MissPolicy::CatchUp) {
            // 追赶：next += period，直到 > now
            do {
                next_task->next += next_task->period;
            } while (next_task->next <= after);
        } else {
            // Skip：丢弃积压，稳定节拍
            next_task->next = after + next_task->period;
        }
    }
}

} // namespace rate_control
