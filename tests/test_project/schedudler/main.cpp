/**
 * @file main.cpp
 * @brief 任务调度器库使用示例
 * 
 * 展示如何使用robot::TaskScheduler库来调度和管理周期性任务。
 */

#include "include/task_scheduler.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <iomanip>

using namespace std::chrono;
using namespace robot;

// 全局原子变量用于任务计数
std::atomic<int> critical_task_count{0};
std::atomic<int> high_priority_task_count{0};
std::atomic<int> medium_priority_task_count{0};
std::atomic<int> low_priority_task_count{0};
std::atomic<int> background_task_count{0};

/**
 * @brief 关键任务函数（模拟电机控制）
 * 
 * 这是一个高频率、低延迟的任务，模拟电机控制。
 * 执行时间：约100微秒
 */
void criticalTask() {
    // 模拟电机控制计算
    auto start = steady_clock::now();
    
    // 简单的计算模拟
    volatile double result = 0.0;
    for (int i = 0; i < 1000; ++i) {
        result += std::sin(i * 0.01);
    }
    
    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    
    critical_task_count++;
    
    // 每100次执行打印一次信息
    if (critical_task_count % 100 == 0) {
        std::cout << "[CRITICAL] 电机控制任务执行 " << critical_task_count 
                  << " 次，本次耗时 " << duration.count() << "μs" << std::endl;
    }
}

/**
 * @brief 高优先级任务函数（模拟传感器处理）
 * 
 * 处理传感器数据，需要较高的实时性。
 * 执行时间：约200-500微秒
 */
void highPriorityTask() {
    auto start = steady_clock::now();
    
    // 模拟传感器数据处理
    volatile double sensor_data = 0.0;
    for (int i = 0; i < 2000; ++i) {
        sensor_data += std::cos(i * 0.005);
    }
    
    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    
    high_priority_task_count++;
    
    if (high_priority_task_count % 50 == 0) {
        std::cout << "[HIGH] 传感器处理任务执行 " << high_priority_task_count 
                  << " 次，本次耗时 " << duration.count() << "μs" << std::endl;
    }
}

/**
 * @brief 中优先级任务函数（模拟数据融合）
 * 
 * 数据融合和路径规划，需要一定的计算时间。
 * 执行时间：约1-2毫秒
 */
void mediumPriorityTask() {
    auto start = steady_clock::now();
    
    // 模拟数据融合计算
    volatile double fusion_result = 0.0;
    for (int i = 0; i < 5000; ++i) {
        fusion_result += std::sin(i * 0.002) * std::cos(i * 0.001);
    }
    
    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    
    medium_priority_task_count++;
    
    if (medium_priority_task_count % 20 == 0) {
        std::cout << "[MEDIUM] 数据融合任务执行 " << medium_priority_task_count 
                  << " 次，本次耗时 " << duration.count() << "μs" << std::endl;
    }
}

/**
 * @brief 低优先级任务函数（模拟日志记录）
 * 
 * 记录系统状态和日志，可以容忍一定的延迟。
 * 执行时间：约5-10毫秒
 */
void lowPriorityTask() {
    auto start = steady_clock::now();
    
    // 模拟日志记录操作
    std::this_thread::sleep_for(microseconds(5000)); // 模拟I/O延迟
    
    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    
    low_priority_task_count++;
    
    if (low_priority_task_count % 10 == 0) {
        std::cout << "[LOW] 日志记录任务执行 " << low_priority_task_count 
                  << " 次，本次耗时 " << duration.count() << "μs" << std::endl;
    }
}

/**
 * @brief 后台任务函数（模拟统计信息更新）
 * 
 * 更新UI或统计信息，最低优先级。
 * 执行时间：约10-20毫秒
 */
void backgroundTask() {
    auto start = steady_clock::now();
    
    // 模拟统计信息计算
    volatile double stats = 0.0;
    for (int i = 0; i < 10000; ++i) {
        stats += std::log(1.0 + i * 0.0001);
    }
    
    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    
    background_task_count++;
    
    if (background_task_count % 5 == 0) {
        std::cout << "[BACKGROUND] 统计更新任务执行 " << background_task_count 
                  << " 次，本次耗时 " << duration.count() << "μs" << std::endl;
    }
}

/**
 * @brief 单次任务函数（模拟系统初始化）
 * 
 * 只执行一次的任务，用于系统初始化。
 */
void oneTimeTask() {
    std::cout << "========================================" << std::endl;
    std::cout << "系统初始化任务执行" << std::endl;
    std::cout << "任务调度器示例程序启动" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 模拟初始化操作
    std::this_thread::sleep_for(milliseconds(100));
    
    std::cout << "系统初始化完成" << std::endl;
}

/**
 * @brief 显示任务统计信息
 * 
 * 定期显示所有任务的统计信息。
 */
void displayStats(TaskScheduler& scheduler) {
    while (true) {
        std::this_thread::sleep_for(seconds(5));
        
        auto stats = scheduler.getTasksStats();
        
        std::cout << "\n========== 任务统计信息 ==========" << std::endl;
        std::cout << "时间: " << duration_cast<seconds>(steady_clock::now().time_since_epoch()).count() 
                  << "秒" << std::endl;
        
        for (const auto& [task_name, task_stats] : stats) {
            std::cout << "\n任务: " << task_name << std::endl;
            std::cout << "  目标频率: " << task_stats.at("frequency_hz") << " Hz" << std::endl;
            std::cout << "  实际频率: " << task_stats.at("actual_frequency_hz") << " Hz" << std::endl;
            std::cout << "  执行时间: " << task_stats.at("execution_time_us") << " μs" << std::endl;
            std::cout << "  超时次数: " << task_stats.at("missed_deadlines") << std::endl;
            std::cout << "  优先级: " << task_stats.at("priority") << std::endl;
            std::cout << "  时间预算: " << task_stats.at("time_budget_ms") << " ms" << std::endl;
            
            // 显示间隔统计信息
            if (task_stats.at("frequency_hz") > 0) {
                int target_interval = task_stats.at("target_interval_us");
                int avg_interval = task_stats.at("avg_interval_us");
                int max_interval = task_stats.at("max_interval_us");
                int min_interval = task_stats.at("min_interval_us");
                
                std::cout << "  目标间隔: " << target_interval << " μs" << std::endl;
                std::cout << "  平均间隔: " << avg_interval << " μs" << std::endl;
                std::cout << "  最大间隔: " << max_interval << " μs" << std::endl;
                std::cout << "  最小间隔: " << min_interval << " μs" << std::endl;
                
                // 计算间隔偏差百分比
                if (target_interval > 0) {
                    float deviation = (static_cast<float>(avg_interval - target_interval) / target_interval) * 100.0f;
                    std::cout << "  间隔偏差: " << std::fixed << std::setprecision(2) << deviation << "%" << std::endl;
                    
                    // 计算间隔均匀性（最大最小间隔差）
                    int interval_range = max_interval - min_interval;
                    std::cout << "  间隔范围: " << interval_range << " μs" << std::endl;
                    
                    if (interval_range > target_interval * 0.5) {
                        std::cout << "  ⚠️  警告: 间隔不均匀！" << std::endl;
                    } else if (interval_range > target_interval * 0.2) {
                        std::cout << "  ⚠️  注意: 间隔略有波动" << std::endl;
                    } else {
                        std::cout << "  ✅ 良好: 间隔均匀" << std::endl;
                    }
                }
            }
        }
        
        std::cout << "\n任务执行计数:" << std::endl;
        std::cout << "  关键任务: " << critical_task_count.load() << std::endl;
        std::cout << "  高优先级: " << high_priority_task_count.load() << std::endl;
        std::cout << "  中优先级: " << medium_priority_task_count.load() << std::endl;
        std::cout << "  低优先级: " << low_priority_task_count.load() << std::endl;
        std::cout << "  后台任务: " << background_task_count.load() << std::endl;
        std::cout << "====================================\n" << std::endl;
    }
}

/**
 * @brief 主函数
 * 
 * 演示任务调度器的完整使用流程：
 * 1. 创建调度器
 * 2. 添加各种优先级的任务
 * 3. 启动调度器
 * 4. 运行一段时间
 * 5. 动态添加/移除任务
 * 6. 停止调度器
 */
int main() {
    std::cout << "开始任务调度器示例程序...2.0" << std::endl;
    
    try {
        // 1. 创建任务调度器（使用4个工作线程）
        TaskScheduler scheduler(7);
        std::cout << "创建任务调度器，工作线程数: " << scheduler.getWorkerThreads() << std::endl;
        
        // 2. 添加各种优先级的任务
        // 单次任务（系统初始化）
        scheduler.addTask("system_init", oneTimeTask, 0, Priority::CRITICAL, 200);
        
        // 关键任务：电机控制（200Hz，时间预算1ms）
        scheduler.addTask("motor_control", criticalTask, 200, Priority::CRITICAL, 1);
        
        // 高优先级任务：传感器处理（100Hz，时间预算5ms）
        scheduler.addTask("sensor_processing", highPriorityTask, 100, Priority::HIGH, 5);
        
        // 中优先级任务：数据融合（50Hz，时间预算10ms）
        scheduler.addTask("data_fusion", mediumPriorityTask, 50, Priority::MEDIUM, 10);
        
        // 低优先级任务：日志记录（10Hz，时间预算50ms）
        scheduler.addTask("log_writing", lowPriorityTask, 10, Priority::LOW, 50);
        
        // 后台任务：统计更新（5Hz，时间预算100ms）
        scheduler.addTask("stats_update", backgroundTask, 5, Priority::BACKGROUND, 100);
        
        // 3. 启动调度器
        if (!scheduler.start()) {
            std::cerr << "无法启动调度器" << std::endl;
            return 1;
        }
        
        std::cout << "调度器启动成功，运行10秒..." << std::endl;
        
        // 启动统计显示线程
        std::thread stats_thread(displayStats, std::ref(scheduler));
        
        // 4. 运行一段时间
        std::this_thread::sleep_for(seconds(10));
        
        // 5. 动态添加新任务
        std::cout << "\n动态添加新任务：图像处理（30Hz）" << std::endl;
        scheduler.addTask("image_processing", 
                         []() {
                             std::this_thread::sleep_for(milliseconds(5));
                             static int count = 0;
                             if (++count % 30 == 0) {
                                 std::cout << "[NEW] 图像处理任务执行 " << count << " 次" << std::endl;
                             }
                         },
                         30, Priority::MEDIUM, 20);
        
        // 再运行5秒
        std::this_thread::sleep_for(seconds(5));
        
        // 6. 动态移除任务
        std::cout << "\n动态移除任务：日志记录" << std::endl;
        scheduler.removeTask("log_writing");
        
        // 再运行5秒
        std::this_thread::sleep_for(seconds(5));
        
        // 7. 停止调度器
        std::cout << "\n准备停止调度器..." << std::endl;
        scheduler.stop();
        
        // 停止统计线程
        stats_thread.detach(); // 简单起见，直接分离
        
        // 8. 显示最终统计
        std::cout << "\n========== 最终统计 ==========" << std::endl;
        auto final_stats = scheduler.getTasksStats();
        
        for (const auto& [task_name, task_stats] : final_stats) {
            std::cout << "任务 '" << task_name << "': " 
                      << "实际频率 " << task_stats.at("actual_frequency_hz") << " Hz, "
                      << "超时 " << task_stats.at("missed_deadlines") << " 次" << std::endl;
        }
        
        std::cout << "\n所有任务执行完成！" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}