#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <iostream>
#include <functional>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <map>
#include <string>
#include <memory>

namespace robot {

// 任务优先级定义
enum class Priority {
    CRITICAL = 0,  // 关键任务（如电机控制）
    HIGH = 1,      // 高优先级（如路径检测）
    MEDIUM = 2,    // 中优先级（如图像识别）
    LOW = 3,       // 低优先级（如数据上传）
    BACKGROUND = 4 // 后台任务（如显示）
};

// 任务信息结构体
struct TaskInfo {
    std::string name;                // 任务名称
    std::function<void()> function;  // 任务函数
    int frequency_hz;                // 目标频率 (Hz)
    Priority priority;               // 任务优先级
    int time_budget_ms;              // 时间预算 (毫秒)
    
    // 内部使用的字段
    std::chrono::steady_clock::time_point next_run_time;
    std::chrono::microseconds period;
    std::atomic<bool> is_running;
    std::atomic<int> actual_frequency;
    std::atomic<int> missed_deadlines;
    std::atomic<int> execution_time_us; // 最近一次执行时间
};

class TaskScheduler {
public:
    // 构造函数
    TaskScheduler(int worker_threads = 4);
    
    // 析构函数
    ~TaskScheduler();
    
    // 添加周期性任务
    bool addTask(const std::string& name, 
                 std::function<void()> task_function,
                 int frequency_hz, 
                 Priority priority = Priority::MEDIUM,
                 int time_budget_ms = 0);
    
    // 移除任务
    bool removeTask(const std::string& name);
    
    // 启动调度器
    bool start();
    
    // 停止调度器
    void stop();
    
    // 获取任务统计信息
    std::map<std::string, std::map<std::string, int>> getTasksStats();
    
private:
    // 工作线程函数
    void workerThread();
    
    // 调度线程函数
    void schedulerThread();
    
    // 比较任务优先级的函数
    struct TaskCompare {
        bool operator()(const std::shared_ptr<TaskInfo>& a, const std::shared_ptr<TaskInfo>& b) {
            // 首先按照下一次运行时间排序
            if (a->next_run_time != b->next_run_time) {
                return a->next_run_time > b->next_run_time;
            }
            // 时间相同时按照优先级排序
            return static_cast<int>(a->priority) > static_cast<int>(b->priority);
        }
    };
    
    // 任务队列
    std::priority_queue<std::shared_ptr<TaskInfo>, 
                       std::vector<std::shared_ptr<TaskInfo>>, 
                       TaskCompare> task_queue_;
    
    // 任务集合（按名称索引）
    std::map<std::string, std::shared_ptr<TaskInfo>> tasks_;
    
    // 线程相关
    std::vector<std::thread> worker_threads_;
    std::thread scheduler_thread_;
    
    // 同步原语
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    
    // 控制标志
    std::atomic<bool> running_;
    std::atomic<int> num_worker_threads_;
    
    // 正在执行的任务数量
    std::atomic<int> active_tasks_;
};

} // namespace robot

#endif // TASK_SCHEDULER_H
