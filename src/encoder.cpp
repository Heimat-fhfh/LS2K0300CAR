// encoder.cpp
#include "encoder.hpp"
#include <fstream>
#include <sstream>
#include <system_error>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

Encoder::Encoder(std::string device_path) 
    : device_path_(std::move(device_path)) {
    
        // 使用 POSIX 方式验证文件可读性
    int fd = open(device_path_.c_str(), O_RDONLY);
    if (fd == -1) {
        device_path_.clear();  // 标记为无效
    } else {
        close(fd);
    }

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

std::int16_t Encoder::readCount() const {
    if (!isValid()) {
        throw EncoderException(device_path_, "invalid");
    }
    return readFromDevice(device_path_);
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