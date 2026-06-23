// encoder.hpp
#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cstdint>
#include <string>
#include <system_error>

class Encoder {
public:
    // 显式构造函数，接受设备文件路径、取反标志和机械参数
    explicit Encoder(std::string device_path,
                     bool invert = false,              // 新增取反参数，默认不取反
                     int encoder_teeth = 30,           // 编码器齿轮齿数
                     int motor_teeth = 68,             // 电机齿轮齿数
                     double wheel_diameter_cm = 6.5,   // 车轮直径(cm)
                     double unit_rps = 0.1);            // 编码器读数单位(rps)
    
    // 禁止拷贝，允许移动
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    Encoder(Encoder&&) noexcept = default;
    Encoder& operator=(Encoder&&) noexcept = default;
    
    // 虚析构函数，支持继承
    virtual ~Encoder() noexcept = default;
    
    // 读取当前速度值(m/s)（已取反）
    [[nodiscard]] double readSpeed() const;
    
    // 获取设备路径
    [[nodiscard]] const std::string& devicePath() const noexcept;
    
    // 检查设备是否可读
    [[nodiscard]] bool isValid() const noexcept;
    
    // 获取转换系数
    [[nodiscard]] double conversionFactor() const noexcept;

private:
    std::string device_path_;
    bool invert_;                // 新增取反标志
    double conversion_factor_;    // 预计算的转换系数
    
    // 计算转换系数
    static double calculateConversionFactor(int encoder_teeth, int motor_teeth,
                                           double wheel_diameter_cm, double unit_rps);
    
    // 实际执行文件读取的静态方法
    static std::int16_t readFromDevice(const std::string& path);
};

// 异常类
class EncoderException : public std::system_error {
public:
    EncoderException(const std::string& device, const std::string& operation);
};

#endif // ENCODER_HPP