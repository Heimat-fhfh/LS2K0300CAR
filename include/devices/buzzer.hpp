#pragma once

#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>

/**
 * @brief 有源蜂鸣器控制类
 *
 * 默认设备路径：/dev/zf_driver_gpio_beep
 * 默认假设低电平有效（0 为响，1 为不响）。
 */
class Buzzer {
public:
    static constexpr uint32_t kDefaultShortDurationMs = 100;
    static constexpr uint32_t kDefaultLongDurationMs = 500;
    static constexpr uint32_t kDefaultIntervalDurationMs = 200;

    explicit Buzzer(const char* device_path = "/dev/zf_driver_gpio_beep");
    ~Buzzer();

    void shortBeep(uint32_t times = 1);
    void patternDoubleShort();
    void patternLongShort();
    void patternContinuous();
    void stop();

    void customPattern(const std::vector<uint32_t>& on_times,
                       const std::vector<uint32_t>& off_times,
                       uint32_t repeat_times = 1);

    Buzzer& setShortDuration(uint32_t ms);
    Buzzer& setLongDuration(uint32_t ms);
    Buzzer& setIntervalDuration(uint32_t ms);

    Buzzer(const Buzzer&) = delete;
    Buzzer& operator=(const Buzzer&) = delete;

private:
    void ensureDeviceAccessible() const;
    void setLevel(bool active);
    void startPattern(const std::vector<uint32_t>& on_times,
                      const std::vector<uint32_t>& off_times,
                      uint32_t repeat_times,
                      bool continuous_mode = false);
    bool waitOrStop(uint32_t duration_ms);
    void stopLocked(std::unique_lock<std::mutex>& lock);

private:
    std::string device_path_;
    uint32_t short_duration_ms_;
    uint32_t long_duration_ms_;
    uint32_t interval_duration_ms_;

    std::thread worker_thread_;
    std::mutex control_mutex_;
    std::mutex wait_mutex_;
    std::condition_variable stop_cv_;
    std::atomic<bool> stop_requested_;
};
