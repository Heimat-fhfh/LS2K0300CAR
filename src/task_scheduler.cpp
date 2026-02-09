#include "task_scheduler.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>

using namespace std::chrono;

namespace robot {

// TaskInfo 构造函数实现
TaskScheduler::TaskInfo::TaskInfo(const std::string& name, 
                                 std::function<void()> func,
                                 int freq_hz,
                                 Priority prio,
                                 int budget_ms)
    : name(name)
    , function(func)
    , frequency_hz(freq_hz)
    , priority(prio)
    , time_budget_ms(budget_ms)
    , last_stat_update(steady_clock::now())
    , last_execution_time(steady_clock::now()) {
    
    if (frequency_hz > 0) {
        period = microseconds(1000000 / frequency_hz);
    } else {
        period = microseconds(0); // 单次任务
    }
    next_run_time = steady_clock::now();
    
    // 初始化原子变量
    is_running.store(false);
    actual_frequency.store(0);
    execution_time_us.store(0);
    missed_deadlines.store(0);
    execution_count.store(0);
    max_interval_us.store(0);
    min_interval_us.store(0);
    total_interval_us.store(0);
    interval_count.store(0);
}

// TaskScheduler 构造函数
TaskScheduler::TaskScheduler(int worker_threads)
    : worker_threads_count_(worker_threads > 0 ? worker_threads : 1) {
    // 确保至少有一个工作线程
    if (worker_threads_count_ < 1) {
        worker_threads_count_ = 1;
    }
}

// TaskScheduler 析构函数
TaskScheduler::~TaskScheduler() {
    stop();
}

// 添加任务
bool TaskScheduler::addTask(const std::string& name, 
                           std::function<void()> task_function,
                           int frequency_hz, 
                           Priority priority,
                           int time_budget_ms) {
    
    // 参数检查
    if (name.empty() || !task_function) {
        std::cerr << "TaskScheduler: 无效的任务参数" << std::endl;
        return false;
    }
    
    if (frequency_hz < 0) {
        std::cerr << "TaskScheduler: 频率不能为负数" << std::endl;
        return false;
    }
    
    if (time_budget_ms < 0) {
        std::cerr << "TaskScheduler: 时间预算不能为负数" << std::endl;
        return false;
    }
    
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    
    // 检查任务是否已存在
    for (const auto& task : tasks_) {
        if (task->name == name) {
            std::cerr << "TaskScheduler: 任务 '" << name << "' 已存在" << std::endl;
            return false;
        }
    }
    
    // 创建新任务
    auto task = std::make_shared<TaskInfo>(name, task_function, frequency_hz, priority, time_budget_ms);
    tasks_.push_back(task);
    
    // 按优先级排序（CRITICAL 优先）
    std::sort(tasks_.begin(), tasks_.end(), 
              [](const std::shared_ptr<TaskInfo>& a, const std::shared_ptr<TaskInfo>& b) {
                  return static_cast<int>(a->priority) < static_cast<int>(b->priority);
              });
    
    std::cout << "TaskScheduler: 添加任务 '" << name << "'，频率 " << frequency_hz << " Hz，优先级 " 
              << static_cast<int>(priority) << std::endl;
    
    // 通知工作线程有新任务
    condition_.notify_all();
    return true;
}

// 移除任务
bool TaskScheduler::removeTask(const std::string& name) {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    
    auto it = std::remove_if(tasks_.begin(), tasks_.end(),
                            [&name](const std::shared_ptr<TaskInfo>& task) {
                                return task->name == name;
                            });
    
    if (it != tasks_.end()) {
        tasks_.erase(it, tasks_.end());
        std::cout << "TaskScheduler: 移除任务 '" << name << "'" << std::endl;
        return true;
    }
    
    std::cerr << "TaskScheduler: 任务 '" << name << "' 不存在" << std::endl;
    return false;
}

// 启动调度器
bool TaskScheduler::start() {
    if (running_) {
        std::cerr << "TaskScheduler: 调度器已经在运行" << std::endl;
        return false;
    }
    
    running_ = true;
    stop_requested_ = false;
    
    // 启动工作线程
    for (int i = 0; i < worker_threads_count_; ++i) {
        worker_threads_.emplace_back(&TaskScheduler::workerThread, this, i);
    }
    
    // 启动统计线程
    stats_running_ = true;
    stats_thread_ = std::thread([this]() {
        while (stats_running_) {
            updateTaskStats();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    
    std::cout << "TaskScheduler: 启动调度器，工作线程数: " << worker_threads_count_ << std::endl;
    return true;
}

// 停止调度器
void TaskScheduler::stop() {
    if (!running_) {
        return;
    }
    
    std::cout << "TaskScheduler: 正在停止调度器..." << std::endl;
    
    stop_requested_ = true;
    running_ = false;
    
    // 通知所有线程
    condition_.notify_all();
    
    // 停止统计线程
    stats_running_ = false;
    if (stats_thread_.joinable()) {
        stats_thread_.join();
    }
    
    // 等待所有工作线程完成
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    worker_threads_.clear();
    
    std::cout << "TaskScheduler: 调度器已停止" << std::endl;
}

// 工作线程函数
void TaskScheduler::workerThread(int thread_id) {
    std::cout << "TaskScheduler: 工作线程 " << thread_id << " 启动" << std::endl;
    
    while (running_ && !stop_requested_) {
        std::shared_ptr<TaskInfo> task_to_execute;
        auto now = steady_clock::now();
        
        {
            std::unique_lock<std::mutex> lock(tasks_mutex_);
            
            // 如果没有任务，等待
            if (tasks_.empty()) {
                condition_.wait(lock);
                continue;
            }
            
            // 查找最早到期的就绪任务（按优先级和到期时间）
            steady_clock::time_point earliest_time = steady_clock::time_point::max();
            std::shared_ptr<TaskInfo> earliest_task = nullptr;
            
            for (auto& task : tasks_) {
                if (now >= task->next_run_time && !task->is_running.load()) {
                    // 找到就绪任务，选择最早到期的（按优先级优先）
                    if (earliest_task == nullptr || 
                        task->next_run_time < earliest_time ||
                        (task->next_run_time == earliest_time && 
                         static_cast<int>(task->priority) < static_cast<int>(earliest_task->priority))) {
                        earliest_task = task;
                        earliest_time = task->next_run_time;
                    }
                }
            }
            
            // 如果没有就绪任务，计算下一个唤醒时间
            if (earliest_task == nullptr) {
                auto next_wake_time = steady_clock::time_point::max();
                for (const auto& task : tasks_) {
                    if (task->next_run_time < next_wake_time) {
                        next_wake_time = task->next_run_time;
                    }
                }
                
                if (next_wake_time != steady_clock::time_point::max()) {
                    condition_.wait_until(lock, next_wake_time);
                } else {
                    condition_.wait(lock);
                }
                continue;
            }
            
            // 标记任务为执行中
            task_to_execute = earliest_task;
            task_to_execute->is_running.store(true);
            
            // 更新任务的下次执行时间（基于计划时间，不是当前时间）
            if (task_to_execute->frequency_hz > 0) {
                // 周期性任务：基于计划时间更新下次执行时间
                task_to_execute->next_run_time = task_to_execute->next_run_time + task_to_execute->period;
                
                // 如果任务已经严重滞后，跳过一些周期
                while (task_to_execute->next_run_time <= now && task_to_execute->frequency_hz > 0) {
                    task_to_execute->next_run_time += task_to_execute->period;
                    task_to_execute->missed_deadlines.fetch_add(1);
                }
            } else {
                // 单次任务：设置为最大时间，不再执行
                task_to_execute->next_run_time = steady_clock::time_point::max();
            }
        }
        
        // 执行单个任务
        if (task_to_execute) {
            executeTask(task_to_execute);
        }
        
        // 短暂休眠以避免忙等待，但保持响应性
        std::this_thread::sleep_for(microseconds(10));
    }
    
    std::cout << "TaskScheduler: 工作线程 " << thread_id << " 退出" << std::endl;
}

// 执行单个任务
void TaskScheduler::executeTask(std::shared_ptr<TaskInfo> task) {
    auto start_time = steady_clock::now();
    
    // 计算执行间隔（从上次执行到现在的时间）
    auto time_since_last_execution = duration_cast<microseconds>(start_time - task->last_execution_time);
    task->last_execution_time = start_time;
    
    // 更新间隔统计
    if (task->interval_count.load() > 0) {
        int interval_us = static_cast<int>(time_since_last_execution.count());
        
        // 更新最大间隔
        int current_max = task->max_interval_us.load();
        while (interval_us > current_max) {
            if (task->max_interval_us.compare_exchange_weak(current_max, interval_us)) {
                break;
            }
        }
        
        // 更新最小间隔
        int current_min = task->min_interval_us.load();
        while (interval_us < current_min || current_min == 0) {
            if (task->min_interval_us.compare_exchange_weak(current_min, interval_us)) {
                break;
            }
        }
        
        // 更新总间隔和计数
        task->total_interval_us.fetch_add(interval_us);
        task->interval_count.fetch_add(1);
    } else {
        // 第一次执行，初始化间隔统计
        task->interval_count.store(1);
    }
    
    try {
        // 执行任务函数
        task->function();
        
        // 更新执行计数
        task->execution_count.fetch_add(1);
        
    } catch (const std::exception& e) {
        std::cerr << "TaskScheduler: 任务 '" << task->name << "' 抛出异常: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "TaskScheduler: 任务 '" << task->name << "' 抛出未知异常" << std::endl;
    }
    
    auto end_time = steady_clock::now();
    auto execution_time = duration_cast<microseconds>(end_time - start_time);
    
    // 更新执行时间
    task->execution_time_us.store(execution_time.count());
    
    // 检查是否超时
    if (task->time_budget_ms > 0) {
        auto execution_time_ms = execution_time.count() / 1000;
        if (execution_time_ms > task->time_budget_ms) {
            task->missed_deadlines.fetch_add(1);
            std::cerr << "TaskScheduler: 任务 '" << task->name << "' 超时: " 
                      << execution_time_ms << "ms > " << task->time_budget_ms << "ms" << std::endl;
        }
    }
    
    // 标记任务完成
    task->is_running.store(false);
}

// 更新任务统计信息
void TaskScheduler::updateTaskStats() {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    
    auto now = steady_clock::now();
    
    for (const auto& task : tasks_) {
        std::map<std::string, int> stats;
        
        // 计算实际频率（过去1秒内的执行次数）
        auto time_since_last_update = duration_cast<seconds>(now - task->last_stat_update);
        if (time_since_last_update.count() >= 1) {
            task->actual_frequency.store(task->execution_count.load());
            task->execution_count.store(0);
            task->last_stat_update = now;
        }
        
        // 计算平均间隔
        int avg_interval_us = 0;
        if (task->interval_count.load() > 0) {
            avg_interval_us = task->total_interval_us.load() / task->interval_count.load();
        }
        
        stats["frequency_hz"] = task->frequency_hz;
        stats["actual_frequency_hz"] = task->actual_frequency.load();
        stats["execution_time_us"] = task->execution_time_us.load();
        stats["missed_deadlines"] = task->missed_deadlines.load();
        stats["priority"] = static_cast<int>(task->priority);
        stats["time_budget_ms"] = task->time_budget_ms;
        stats["max_interval_us"] = task->max_interval_us.load();
        stats["min_interval_us"] = task->min_interval_us.load();
        stats["avg_interval_us"] = avg_interval_us;
        stats["target_interval_us"] = (task->frequency_hz > 0) ? (1000000 / task->frequency_hz) : 0;
        
        tasks_stats_[task->name] = stats;
    }
}

// 获取所有任务统计信息
std::map<std::string, std::map<std::string, int>> TaskScheduler::getTasksStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return tasks_stats_;
}

} // namespace robot