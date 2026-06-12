#ifndef ROBOT_TASK_SCHEDULER_HPP
#define ROBOT_TASK_SCHEDULER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot {

/**
 * @brief 任务优先级枚举
 */
enum class Priority {
    CRITICAL = 0,   ///< 关键实时任务（电机控制、安全监控）
    HIGH = 1,       ///< 高优先级（传感器处理、决策）
    MEDIUM = 2,     ///< 中优先级（数据融合、规划）
    LOW = 3,        ///< 低优先级（日志记录）
    BACKGROUND = 4  ///< 后台任务（统计更新、UI）
};

/**
 * @brief 任务调度器类
 * 
 * 提供实时、可靠的周期性任务调度功能，支持多优先级、时间预算监控和运行时统计。
 */
class TaskScheduler {
public:
    /**
     * @brief 构造函数
     * @param worker_threads 工作线程数量（建议设置为CPU核心数-1）
     */
    explicit TaskScheduler(int worker_threads = 4);
    
    /**
     * @brief 析构函数
     */
    ~TaskScheduler();
    
    // 禁止拷贝和移动
    TaskScheduler(const TaskScheduler&) = delete;
    TaskScheduler& operator=(const TaskScheduler&) = delete;
    TaskScheduler(TaskScheduler&&) = delete;
    TaskScheduler& operator=(TaskScheduler&&) = delete;
    
    /**
     * @brief 添加周期性任务
     * @param name 任务唯一标识符
     * @param task_function 要执行的任务函数（无参数、无返回值）
     * @param frequency_hz 目标频率（Hz），0表示单次任务
     * @param priority 任务优先级（默认MEDIUM）
     * @param time_budget_ms 时间预算（ms），超时会被记录（默认0-不限制）
     * @return 添加成功返回true，失败返回false
     */
    bool addTask(const std::string& name, 
                 std::function<void()> task_function,
                 int frequency_hz, 
                 Priority priority = Priority::MEDIUM,
                 int time_budget_ms = 0);
    
    /**
     * @brief 移除任务
     * @param name 要移除的任务名称
     * @return 成功返回true，任务不存在返回false
     */
    bool removeTask(const std::string& name);
    
    /**
     * @brief 启动调度器
     * @return 启动成功返回true，已经运行中返回false
     */
    bool start();
    
    /**
     * @brief 停止调度器（阻塞直到所有任务完成）
     */
    void stop();
    
    /**
     * @brief 获取所有任务统计信息
     * @return 任务名 -> 统计信息 的映射
     */
    std::map<std::string, std::map<std::string, int>> getTasksStats() const;
    
    /**
     * @brief 检查调度器是否正在运行
     * @return 正在运行返回true，否则返回false
     */
    bool isRunning() const { return running_; }
    
    /**
     * @brief 获取工作线程数量
     * @return 工作线程数量
     */
    int getWorkerThreads() const { return worker_threads_count_; }

private:
    /**
     * @brief 任务信息结构体
     */
    struct TaskInfo {
        std::string name;
        std::function<void()> function;
        int frequency_hz;
        Priority priority;
        int time_budget_ms;
        std::chrono::microseconds period;
        std::chrono::steady_clock::time_point next_run_time;
        std::atomic<bool> is_running{false};
        std::atomic<int> actual_frequency{0};
        std::atomic<int> execution_time_us{0};
        std::atomic<int> missed_deadlines{0};
        std::atomic<int> execution_count{0};
        std::chrono::steady_clock::time_point last_stat_update;
        std::chrono::steady_clock::time_point last_execution_time;
        std::atomic<int> max_interval_us{0};
        std::atomic<int> min_interval_us{0};
        std::atomic<int> total_interval_us{0};
        std::atomic<int> interval_count{0};
        
        TaskInfo(const std::string& name, 
                 std::function<void()> func,
                 int freq_hz,
                 Priority prio,
                 int budget_ms);
    };
    
    /**
     * @brief 工作线程函数
     */
    void workerThread(int thread_id);
    
    /**
     * @brief 执行单个任务
     * @param task 任务信息
     */
    void executeTask(std::shared_ptr<TaskInfo> task);
    
    /**
     * @brief 更新任务统计信息
     */
    void updateTaskStats();
    
    // 成员变量
    int worker_threads_count_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    
    std::vector<std::thread> worker_threads_;
    std::vector<std::shared_ptr<TaskInfo>> tasks_;
    
    mutable std::mutex tasks_mutex_;
    std::condition_variable condition_;
    
    std::thread stats_thread_;
    std::atomic<bool> stats_running_{false};
    
    // 统计信息
    mutable std::mutex stats_mutex_;
    std::map<std::string, std::map<std::string, int>> tasks_stats_;
};

} // namespace robot

#endif // ROBOT_TASK_SCHEDULER_HPP