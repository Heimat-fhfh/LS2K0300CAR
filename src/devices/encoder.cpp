// encoder.cpp
#include "devices/encoder.hpp"
#include <fstream>
#include <sstream>
#include <system_error>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

// 构造函数现在接受取反标志和机械参数
Encoder::Encoder(std::string device_path,
                 bool invert,
                 int encoder_teeth,
                 int motor_teeth,
                 double wheel_diameter_cm,
                 double unit_rps)
    : device_path_(std::move(device_path))
    , invert_(invert)  // 新增取反标志初始化
    , conversion_factor_(calculateConversionFactor(encoder_teeth, motor_teeth,
                                                   wheel_diameter_cm, unit_rps)) {
    
    // 使用 POSIX 方式验证文件可读性
    int fd = open(device_path_.c_str(), O_RDONLY);
    if (fd == -1) {
        device_path_.clear();  // 标记为无效
    } else {
        close(fd);
    }
}

// 计算转换系数
double Encoder::calculateConversionFactor(int encoder_teeth, int motor_teeth,
                                          double wheel_diameter_cm, double unit_rps) {
    // 正确的传动比 = 编码器齿轮齿数 / 电机齿轮齿数（轮胎齿轮）
    double gear_ratio = static_cast<double>(encoder_teeth) / motor_teeth;
    
    // 车轮周长(m) = π * 直径(m) = π * (直径cm / 100)
    double wheel_circumference_m = M_PI * (wheel_diameter_cm / 100.0);
    
    // 每rps对应的速度(m/s) = 传动比 * 车轮周长
    double speed_per_rps = gear_ratio * wheel_circumference_m;
    
    // 根据编码器读数单位计算实际转换系数
    // 速度(m/s) = 编码器读数 * (单位rps) * speed_per_rps
    // 所以转换系数 = unit_rps * speed_per_rps
    return unit_rps * speed_per_rps;
}

// 读取速度的新方法
double Encoder::readSpeed() const {
    if (!isValid()) {
        throw EncoderException(device_path_, "invalid");
    }
    
    // 读取原始计数值
    std::int16_t count = readFromDevice(device_path_);
    
    // 如果需要取反
    if (invert_) {
        count = -count;
    }
    
    // 直接乘以预计算的转换系数得到速度(m/s)
    return static_cast<double>(count) * conversion_factor_;
}

// 获取转换系数
double Encoder::conversionFactor() const noexcept {
    return conversion_factor_;
}

// 读取当前计数值（已取反）
std::int16_t Encoder::readCount() const {
    if (!isValid()) {
        throw EncoderException(device_path_, "invalid");
    }
    
    std::int16_t count = readFromDevice(device_path_);
    
    // 如果需要取反
    if (invert_) {
        count = -count;
    }
    
    return count;
}

std::int16_t Encoder::readFromDevice(const std::string& path) {
    int fd = open(path.c_str(), O_RDONLY);
    if (fd == -1) {
        throw EncoderException(path, "open");
    }
    
    std::int16_t value = 0;
    ssize_t result = read(fd, &value, sizeof(value));
    
    if (result == -1) {
        close(fd);
        throw EncoderException(path, "read");
    }
    
    if (close(fd) == -1) {
        throw EncoderException(path, "close");
    }
    
    return value;
}

const std::string& Encoder::devicePath() const noexcept {
    return device_path_;
}

bool Encoder::isValid() const noexcept {
    return !device_path_.empty();
}

EncoderException::EncoderException(const std::string& device, const std::string& operation)
    : std::system_error(errno, std::generic_category(),
                       "Encoder device '" + device + "' " + operation + " failed") {}