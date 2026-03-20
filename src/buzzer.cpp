#include "buzzer.hpp"

#include "zf_driver_gpio.h"

#include <stdexcept>
#include <sstream>

#include <unistd.h>

namespace {
constexpr uint8_t kBuzzerOnLevel = 0x0;
constexpr uint8_t kBuzzerOffLevel = 0x1;
}

Buzzer::Buzzer(const char* device_path)
    : device_path_(device_path == nullptr ? "" : device_path),
      short_duration_ms_(kDefaultShortDurationMs),
      long_duration_ms_(kDefaultLongDurationMs),
      interval_duration_ms_(kDefaultIntervalDurationMs),
      stop_requested_(false) {
    if (device_path_.empty()) {
        throw std::invalid_argument("Buzzer device path must not be empty");
    }

    ensureDeviceAccessible();
    off();
}

Buzzer::~Buzzer() {
    try {
        stop();
    } catch (...) {
        // 析构函数中避免异常传播
    }
}

void Buzzer::on() {
    std::unique_lock<std::mutex> lock(control_mutex_);
    stopLocked(lock);
    setLevel(true);
}

void Buzzer::off() {
    std::unique_lock<std::mutex> lock(control_mutex_);
    stopLocked(lock);
    setLevel(false);
}

void Buzzer::beep(uint32_t duration_ms) {
    if (duration_ms == 0) {
        throw std::invalid_argument("beep duration must be greater than 0");
    }
    startPattern({duration_ms}, {0}, 1, false);
}

void Buzzer::shortBeep(uint32_t times) {
    if (times == 0) {
        throw std::invalid_argument("shortBeep times must be greater than 0");
    }
    startPattern({short_duration_ms_}, {interval_duration_ms_}, times, false);
}

void Buzzer::longBeep(uint32_t times) {
    if (times == 0) {
        throw std::invalid_argument("longBeep times must be greater than 0");
    }
    startPattern({long_duration_ms_}, {interval_duration_ms_}, times, false);
}

void Buzzer::patternDoubleShort() {
    startPattern({short_duration_ms_, short_duration_ms_},
                 {interval_duration_ms_, interval_duration_ms_},
                 1,
                 false);
}

void Buzzer::patternTripleShort() {
    startPattern({short_duration_ms_, short_duration_ms_, short_duration_ms_},
                 {interval_duration_ms_, interval_duration_ms_, interval_duration_ms_},
                 1,
                 false);
}

void Buzzer::patternLongShort() {
    startPattern({long_duration_ms_, short_duration_ms_},
                 {interval_duration_ms_, interval_duration_ms_},
                 1,
                 false);
}

void Buzzer::patternContinuous() {
    startPattern({0}, {0}, 1, true);
}

void Buzzer::stop() {
    std::unique_lock<std::mutex> lock(control_mutex_);
    stopLocked(lock);
    setLevel(false);
}

void Buzzer::customPattern(const std::vector<uint32_t>& on_times,
                           const std::vector<uint32_t>& off_times,
                           uint32_t repeat_times) {
    if (on_times.empty()) {
        throw std::invalid_argument("customPattern on_times must not be empty");
    }
    if (repeat_times == 0) {
        throw std::invalid_argument("customPattern repeat_times must be greater than 0");
    }

    for (uint32_t duration : on_times) {
        if (duration == 0) {
            throw std::invalid_argument("customPattern on_times contains 0 duration");
        }
    }

    if (!off_times.empty() && off_times.size() != 1 && off_times.size() != on_times.size()) {
        throw std::invalid_argument("customPattern off_times size must be 0, 1, or equal to on_times size");
    }

    startPattern(on_times, off_times, repeat_times, false);
}

Buzzer& Buzzer::setShortDuration(uint32_t ms) {
    if (ms == 0) {
        throw std::invalid_argument("short duration must be greater than 0");
    }
    short_duration_ms_ = ms;
    return *this;
}

Buzzer& Buzzer::setLongDuration(uint32_t ms) {
    if (ms == 0) {
        throw std::invalid_argument("long duration must be greater than 0");
    }
    long_duration_ms_ = ms;
    return *this;
}

Buzzer& Buzzer::setIntervalDuration(uint32_t ms) {
    interval_duration_ms_ = ms;
    return *this;
}

void Buzzer::ensureDeviceAccessible() const {
    if (access(device_path_.c_str(), W_OK) == 0) {
        return;
    }

    std::ostringstream oss;
    oss << "Buzzer device path is not writable: " << device_path_;
    throw std::runtime_error(oss.str());
}

void Buzzer::setLevel(bool active) {
    ensureDeviceAccessible();
    gpio_set_level(device_path_.c_str(), active ? kBuzzerOnLevel : kBuzzerOffLevel);
}

void Buzzer::startPattern(const std::vector<uint32_t>& on_times,
                          const std::vector<uint32_t>& off_times,
                          uint32_t repeat_times,
                          bool continuous_mode) {
    std::unique_lock<std::mutex> lock(control_mutex_);
    stopLocked(lock);
    stop_requested_.store(false);

    worker_thread_ = std::thread([this, on_times, off_times, repeat_times, continuous_mode]() {
        try {
            for (uint32_t repeat = 0; repeat < repeat_times && !stop_requested_.load(); ++repeat) {
                for (size_t i = 0; i < on_times.size() && !stop_requested_.load(); ++i) {
                    setLevel(true);

                    if (continuous_mode) {
                        std::unique_lock<std::mutex> wait_lock(wait_mutex_);
                        stop_cv_.wait(wait_lock, [this]() { return stop_requested_.load(); });
                        return;
                    }

                    if (!waitOrStop(on_times[i])) {
                        return;
                    }

                    setLevel(false);

                    bool has_next_segment = (i + 1 < on_times.size()) || (repeat + 1 < repeat_times);
                    if (!has_next_segment) {
                        continue;
                    }

                    uint32_t off_duration = interval_duration_ms_;
                    if (!off_times.empty()) {
                        off_duration = (off_times.size() == 1) ? off_times[0] : off_times[i];
                    }

                    if (off_duration > 0 && !waitOrStop(off_duration)) {
                        return;
                    }
                }
            }

            setLevel(false);
        } catch (...) {
            // 异步线程中出现异常时强制关闭蜂鸣器，避免持续鸣叫。
            try {
                gpio_set_level(device_path_.c_str(), kBuzzerOffLevel);
            } catch (...) {
            }
        }
    });
}

bool Buzzer::waitOrStop(uint32_t duration_ms) {
    if (duration_ms == 0) {
        return !stop_requested_.load();
    }

    std::unique_lock<std::mutex> lock(wait_mutex_);
    bool stopped = stop_cv_.wait_for(
        lock,
        std::chrono::milliseconds(duration_ms),
        [this]() { return stop_requested_.load(); });
    return !stopped;
}

void Buzzer::stopLocked(std::unique_lock<std::mutex>& lock) {
    stop_requested_.store(true);
    stop_cv_.notify_all();

    if (!worker_thread_.joinable()) {
        return;
    }

    lock.unlock();
    worker_thread_.join();
    lock.lock();
}