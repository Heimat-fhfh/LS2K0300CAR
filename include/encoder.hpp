// encoder.hpp
#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cstdint>
#include <string>
#include <system_error>

class Encoder {
public:
    // 显式构造函数，接受设备文件路径
    explicit Encoder(std::string device_path);
    
    // 禁止拷贝，允许移动
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    Encoder(Encoder&&) noexcept = default;
    Encoder& operator=(Encoder&&) noexcept = default;
    
    // 虚析构函数，支持继承
    virtual ~Encoder() noexcept = default;
    
    // 读取当前计数值
    [[nodiscard]] std::int16_t readCount() const;
    
    // 获取设备路径
    [[nodiscard]] const std::string& devicePath() const noexcept;
    
    // 检查设备是否可读
    [[nodiscard]] bool isValid() const noexcept;

private:
    std::string device_path_;
    
    // 实际执行文件读取的静态方法
    static std::int16_t readFromDevice(const std::string& path);
};

// 异常类
class EncoderException : public std::system_error {
public:
    EncoderException(const std::string& device, const std::string& operation);
};

#endif // ENCODER_HPP