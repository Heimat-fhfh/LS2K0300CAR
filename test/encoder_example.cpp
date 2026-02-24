// usage_example.cpp
#include "encoder.hpp"
#include <iostream>
#include <memory>
#include <vector>

class WheelEncoder {
public:
    explicit WheelEncoder(std::unique_ptr<Encoder> left, std::unique_ptr<Encoder> right)
        : left_encoder_(std::move(left))
        , right_encoder_(std::move(right)) {
        
        if (!left_encoder_ || !right_encoder_) {
            throw std::invalid_argument("Encoder pointers cannot be null");
        }
    }
    
    // 读取左右编码器值
    std::pair<std::int16_t, std::int16_t> readBoth() {
        return { left_encoder_->readCount(), right_encoder_->readCount() };
    }
    
    // 单独读取左编码器
    std::int16_t readLeft() const {
        return left_encoder_->readCount();
    }
    
    // 单独读取右编码器
    std::int16_t readRight() const {
        return right_encoder_->readCount();
    }

private:
    std::unique_ptr<Encoder> left_encoder_;
    std::unique_ptr<Encoder> right_encoder_;
};

// 简单使用示例
int main() {
    try {
        // 创建编码器对象
        Encoder left_encoder("/dev/zf_encoder_1");
        Encoder right_encoder("/dev/zf_encoder_2");
        
        // 验证设备可用性
        if (!left_encoder.isValid() || !right_encoder.isValid()) {
            std::cerr << "Warning: One or more encoders may be invalid\n";
        }
        
        // 主循环
        while (true) {
            try {
                auto left_value = left_encoder.readCount();
                auto right_value = right_encoder.readCount();
                
                std::cout << "Left encoder: " << left_value 
                         << ", Right encoder: " << right_value << std::endl;
                
            } catch (const EncoderException& e) {
                std::cerr << "Read error: " << e.what() << std::endl;
                // 短暂延迟后重试
            }
            
            // 简单的延时替代，实际应用中应使用系统定时器
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}