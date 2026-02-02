#include "rate_control.hpp"
#include <algorithm>
#include <cmath>
#include <thread>

namespace rate_control {

static inline uint64_t to_ns_u64(std::chrono::nanoseconds ns) {
    return static_cast<uint64_t>(ns.count() < 0 ? 0 : ns.count());
}

std::chrono::nanoseconds hz_to_period(double hz) {
    if (hz <= 0.0) {
        return std::chrono::seconds(1);
    }
    const double sec = 1.0 / hz;
    const auto ns = static_cast<int64_t>(sec * 1e9);
    return std::chrono::nanoseconds(std::max<int64_t>(1, ns));
}

Rate::Rate(double hz, MissPolicy policy)
    : period_(hz_to_period(hz)), policy_(policy) {
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

void Scheduler::add_task(const std::string& name, double hz, std::function<void()> fn,
                         MissPolicy policy) {
    Task t;
    t.name = name;
    t.period = hz_to_period(hz);
    t.policy = policy;
    t.fn = std::move(fn);
    t.next = Clock::now() + t.period;
    tasks_.push_back(std::move(t));
}

void Scheduler::run(const std::function<bool()>& stop_flag) {
    while (!stop_flag()) {
        if (tasks_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 找到最早需要执行的任务
        auto it = std::min_element(tasks_.begin(), tasks_.end(),
                                   [](const Task& a, const Task& b) { return a.next < b.next; });

        const auto now = Clock::now();
        if (now < it->next) {
            std::this_thread::sleep_until(it->next);
        }

        // 可能 sleep 后时间变化
        const auto tnow = Clock::now();

        // 执行任务（协作式：单核顺序跑）
        // 统计：是否超期
        it->stats.cycles++;

        bool missed = (tnow > it->next);
        if (missed) {
            it->stats.overruns++;
            const auto late = tnow - it->next;
            it->stats.max_late_ns = std::max<uint64_t>(
                it->stats.max_late_ns,
                to_ns_u64(std::chrono::duration_cast<std::chrono::nanoseconds>(late)));
        }

        // 执行回调，并记录执行耗时（可用于判断“回调本身超时”）
        const auto begin = Clock::now();
        it->fn();
        const auto end = Clock::now();

        // 如果回调耗时超过周期，也算一种 overrun（单核尤其重要）
        const auto exec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
        if (exec_ns > it->period) {
            it->stats.overruns++;
            it->stats.max_late_ns = std::max<uint64_t>(it->stats.max_late_ns, to_ns_u64(exec_ns - it->period));
        }

        // 更新 next
        const auto after = Clock::now();
        if (it->policy == MissPolicy::CatchUp) {
            // 追赶：next += period，直到 > now
            do {
                it->next += it->period;
            } while (it->next <= after);
        } else {
            // Skip：丢弃积压，稳定节拍
            it->next = after + it->period;
        }
    }
}

} // namespace rate_control
