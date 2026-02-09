# 任务调度器库函数详细文档

## 1. 概述

### 1.1 设计目标
本任务调度器库旨在为机器人系统提供实时、可靠的周期性任务调度功能。支持多优先级、时间预算监控和运行时统计。

### 1.2 核心特性
- ✅ 多线程并行执行
- ✅ 周期性任务调度
- ✅ 5级优先级（CRITICAL 到 BACKGROUND）
- ✅ 执行时间统计和监控
- ✅ 超时检测和报告
- ✅ 线程安全的API
- ✅ 单核心,0.37GB内存,基于Linux的loongOS系统

### 1.3 性能指标
- 最高支持任务频率：1000Hz（1ms周期）
- 调度精度：±100μs
- 最大并发任务数：无硬性限制
- 最小任务周期：1ms

## 2. API 参考

### 2.1 核心类

#### `class robot::TaskScheduler`

**构造函数**
```cpp
// 创建一个任务调度器
// @param worker_threads: 工作线程数量（建议设置为CPU核心数-1）
explicit TaskScheduler(int worker_threads = 4);
```

**关键方法**
```cpp
// 添加周期性任务
// @param name: 任务唯一标识符
// @param task_function: 要执行的任务函数（无参数、无返回值）
// @param frequency_hz: 目标频率（Hz），0表示单次任务
// @param priority: 任务优先级（默认MEDIUM）
// @param time_budget_ms: 时间预算（ms），超时会被记录（默认0-不限制）
// @return: 添加成功返回true，失败返回false
bool addTask(const std::string& name, 
             std::function<void()> task_function,
             int frequency_hz, 
             Priority priority = Priority::MEDIUM,
             int time_budget_ms = 0);

// 移除任务
// @param name: 要移除的任务名称
// @return: 成功返回true，任务不存在返回false
bool removeTask(const std::string& name);

// 启动调度器
// @return: 启动成功返回true，已经运行中返回false
bool start();

// 停止调度器（阻塞直到所有任务完成）
void stop();

// 获取所有任务统计信息
// @return: 任务名 -> 统计信息 的映射
std::map<std::string, std::map<std::string, int>> getTasksStats();
```

### 2.2 数据类型

#### `enum class Priority`
```cpp
enum class Priority {
    CRITICAL = 0,  // 关键实时任务（电机控制、安全监控）
    HIGH = 1,      // 高优先级（传感器处理、决策）
    MEDIUM = 2,    // 中优先级（数据融合、规划）
    LOW = 3,       // 低优先级（日志记录）
    BACKGROUND = 4 // 后台任务（统计更新、UI）
};
```

#### `struct TaskInfo`
内部使用，但需要了解其字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `std::string` | 任务名称 |
| `function` | `std::function<void()>` | 任务函数 |
| `frequency_hz` | `int` | 目标频率（Hz） |
| `priority` | `Priority` | 任务优先级 |
| `time_budget_ms` | `int` | 时间预算（ms） |
| `period` | `std::chrono::microseconds` | 执行周期（内部计算） |
| `next_run_time` | `steady_clock::time_point` | 下一次执行时间 |
| `is_running` | `std::atomic<bool>` | 任务是否正在执行 |
| `actual_frequency` | `std::atomic<int>` | 实际运行频率 |
| `execution_time_us` | `std::atomic<int>` | 最近执行时间（μs） |
| `missed_deadlines` | `std::atomic<int>` | 超时次数 |

## 3. 实现细节文档

### 3.1 调度算法

#### 核心调度循环
```cpp
// 工作线程主循环伪代码
void workerThread() {
    while (running_) {
        1. 获取当前时间 now
        2. 锁定互斥锁
        3. 遍历所有任务，找出满足条件的：
           - now >= next_run_time
           - !is_running
        4. 更新 next_run_time = now + period
        5. 标记 is_running = true
        6. 解锁互斥锁
        
        7. 并行执行所有就绪任务
        8. 等待所有任务完成
        9. 标记 is_running = false
        
        10. 计算下一个唤醒时间
        11. sleep_until(下一个任务时间或最小间隔)
    }
}
```

#### 时间计算规则
```cpp
// 周期计算
period = 1,000,000μs / frequency_hz  // 例如：100Hz → 10,000μs

// 下一次执行时间计算
next_run_time = current_time + period

// 频率统计更新（每秒）
actual_frequency = 过去1秒内的执行次数
```

### 3.2 线程模型

```
主线程
    │
    ├── 启动调度器
    │
    └── 工作线程池 (worker_threads个)
        ├── 工作线程1: 执行任务A、B、C...
        ├── 工作线程2: 执行任务D、E、F...
        ├── 工作线程3: 执行任务...
        └── 工作线程4: 执行任务...
```

**重要约束**：
- 每个任务函数必须保证线程安全
- 任务函数执行时间应小于其周期
- 避免在任务函数中长时间阻塞

### 3.3 同步机制

```cpp
// 主要同步对象
std::mutex queue_mutex_;          // 保护任务队列和集合
std::condition_variable condition_; // 线程间通知

// 使用模式：
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // 访问共享数据
}

// 或
{
    std::unique_lock<std::mutex> lock(queue_mutex_);
    condition_.wait(lock, predicate);
}
```

## 4. 正确使用指南

### 4.1 任务设计原则

#### ✅ 正确示例
```cpp
// 良好的任务函数：短时间、非阻塞
void goodTask() {
    // 快速计算
    auto data = sensor.read();
    auto result = process(data);
    controller.set(result);
    // 执行时间 < 1ms
}
```

#### ❌ 错误示例
```cpp
// 不良的任务函数
void badTask() {
    std::this_thread::sleep_for(10ms);  // 长时间阻塞
    // 或
    while(!condition) {}  // 忙等待
    // 或
    std::mutex mtx; mtx.lock();  // 可能死锁
}
```

### 4.2 配置建议

| 任务类型 | 建议频率 | 建议优先级 | 时间预算 |
|----------|----------|------------|----------|
| 电机控制 | 100-500Hz | CRITICAL | 1-2ms |
| 传感器读取 | 50-100Hz | HIGH | 1-5ms |
| 图像处理 | 10-30Hz | MEDIUM | 10-30ms |
| 数据上传 | 1-10Hz | LOW | 50-100ms |
| 状态显示 | 5-10Hz | BACKGROUND | 10-20ms |

```

**重要提醒**：在使用本调度器时，务必遵循以下规则：
1. **任务函数必须异常安全** - 确保不会抛出未捕获的异常
2. **避免在任务中创建新线程** - 使用调度器提供的并发机制
3. **定期监控统计信息** - 及早发现性能问题
4. **合理设置优先级** - 避免低优先级任务饥饿
