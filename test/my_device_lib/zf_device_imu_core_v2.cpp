// 1.2 zf_device_imu_core_v2.cpp - C++面向对象设计

#include "zf_device_imu_core_v2.hpp"
#include "zf_driver_file.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace zf {

//--------------------------------------------------------------------
// ImuSensorData 方法实现
//--------------------------------------------------------------------
std::string ImuSensorData::toString() const
{
    std::stringstream ss;
    ss << "Acc: [" << acc_x << ", " << acc_y << ", " << acc_z << "] "
       << "Gyro: [" << gyro_x << ", " << gyro_y << ", " << gyro_z << "] "
       << "Mag: [" << mag_x << ", " << mag_y << ", " << mag_z << "]";
    return ss.str();
}

//--------------------------------------------------------------------
// ImuConfig 默认构造函数实现
//--------------------------------------------------------------------
ImuConfig::ImuConfig() :
    event_path("/sys/bus/iio/devices/iio:device1/events/in_voltage_change_en"),
    name_path("/sys/bus/iio/devices/iio:device1/name"),
    acc_x_path("/sys/bus/iio/devices/iio:device1/in_accel_x_raw"),
    acc_y_path("/sys/bus/iio/devices/iio:device1/in_accel_y_raw"),
    acc_z_path("/sys/bus/iio/devices/iio:device1/in_accel_z_raw"),
    gyro_x_path("/sys/bus/iio/devices/iio:device1/in_anglvel_x_raw"),
    gyro_y_path("/sys/bus/iio/devices/iio:device1/in_anglvel_y_raw"),
    gyro_z_path("/sys/bus/iio/devices/iio:device1/in_anglvel_z_raw"),
    mag_x_path("/sys/bus/iio/devices/iio:device1/in_magn_x_raw"),
    mag_y_path("/sys/bus/iio/devices/iio:device1/in_magn_y_raw"),
    mag_z_path("/sys/bus/iio/devices/iio:device1/in_magn_z_raw")
{
}

//--------------------------------------------------------------------
// 静态辅助方法
//--------------------------------------------------------------------
std::string ImuDeviceCore::deviceTypeToString(ImuDeviceType type)
{
    switch(type)
    {
        case ImuDeviceType::IMU660RA: return "IMU660RA";
        case ImuDeviceType::IMU660RB: return "IMU660RB";
        case ImuDeviceType::IMU963RA: return "IMU963RA";
        case ImuDeviceType::NO_DEVICE: return "NO_DEVICE";
        default: return "UNKNOWN";
    }
}

//--------------------------------------------------------------------
// ImuDeviceCore 构造函数
//--------------------------------------------------------------------
ImuDeviceCore::ImuDeviceCore() :
    device_type_(ImuDeviceType::NO_DEVICE),
    config_(),
    raw_data_(),
    initialized_(false)
{
}

ImuDeviceCore::ImuDeviceCore(const ImuConfig& config) :
    device_type_(ImuDeviceType::NO_DEVICE),
    config_(config),
    raw_data_(),
    initialized_(false)
{
}

//--------------------------------------------------------------------
// 私有方法：读取原始值
//--------------------------------------------------------------------
int16_t ImuDeviceCore::readRawValue(const std::string& path) const
{
    char str[20] = {0};
    if(file_read_string(path.c_str(), str) < 0)
    {
        return 0;
    }
    return static_cast<int16_t>(atoi(str));
}

//--------------------------------------------------------------------
// 私有方法：通过事件文件初始化
//--------------------------------------------------------------------
bool ImuDeviceCore::initViaEventFile()
{
    char read_buf[20] = {0};
    int read_val = static_cast<int>(ImuDeviceType::NO_DEVICE);
    
    FILE* fp = fopen(config_.event_path.c_str(), "r+");
    if(fp == nullptr)
    {
        std::cerr << "Failed to open event file: " << config_.event_path << std::endl;
        return false;
    }
    
    bool success = false;
    
    // 写入字符'1'完成初始化
    if(fwrite("1", 1, 1, fp) == 1)
    {
        fflush(fp);
        rewind(fp);  // 回到文件开头
        
        // 尝试读取设备类型值
        if(fgets(read_buf, sizeof(read_buf), fp) != nullptr)
        {
            read_val = atoi(read_buf);
            
            // 验证读取的值是否有效
            if(read_val == static_cast<int>(ImuDeviceType::IMU660RA) ||
               read_val == static_cast<int>(ImuDeviceType::IMU660RB) ||
               read_val == static_cast<int>(ImuDeviceType::IMU963RA))
            {
                device_type_ = static_cast<ImuDeviceType>(read_val);
                success = true;
                std::cout << "IMU initialized via event file, type: " 
                          << deviceTypeToString(device_type_) << std::endl;
            }
        }
    }
    
    fclose(fp);
    return success;
}

//--------------------------------------------------------------------
// 私有方法：通过名称文件初始化
//--------------------------------------------------------------------
bool ImuDeviceCore::initViaNameFile()
{
    char str[20] = {0};
    
    if(file_read_string(config_.name_path.c_str(), str) < 0)
    {
        std::cerr << "Failed to read IMU name file" << std::endl;
        return false;
    }
    
    // 清理字符串中的换行符
    str[strcspn(str, "\r\n")] = 0;
    
    if(strcmp(str, "IMU660RA") == 0)
    {
        device_type_ = ImuDeviceType::IMU660RA;
        std::cout << "IMU detected via name file: IMU660RA" << std::endl;
        return true;
    }
    else if(strcmp(str, "IMU660RB") == 0)
    {
        device_type_ = ImuDeviceType::IMU660RB;
        std::cout << "IMU detected via name file: IMU660RB" << std::endl;
        return true;
    }
    else if(strcmp(str, "IMU963RA") == 0)
    {
        device_type_ = ImuDeviceType::IMU963RA;
        std::cout << "IMU detected via name file: IMU963RA" << std::endl;
        return true;
    }
    
    std::cerr << "Unknown IMU device: " << str << std::endl;
    return false;
}

//--------------------------------------------------------------------
// 公共方法：初始化设备
//--------------------------------------------------------------------
bool ImuDeviceCore::initialize()
{
    // 如果已经初始化，直接返回成功
    if(initialized_)
    {
        return true;
    }
    
    // 重置设备状态
    device_type_ = ImuDeviceType::NO_DEVICE;
    raw_data_ = RawImuData();
    
    std::cout << "Initializing IMU device..." << std::endl;
    
    // 第一步：尝试通过事件文件初始化
    if(initViaEventFile())
    {
        initialized_ = true;
        return true;
    }
    
    // 第二步：如果事件文件方式失败，回退到name文件方式
    std::cout << "Event file initialization failed, trying name file..." << std::endl;
    
    if(initViaNameFile())
    {
        initialized_ = true;
        return true;
    }
    
    // 两种方式都失败
    std::cerr << "IMU initialization failed with both methods" << std::endl;
    initialized_ = false;
    return false;
}

//--------------------------------------------------------------------
// 公共方法：读取所有传感器数据
//--------------------------------------------------------------------
bool ImuDeviceCore::readAllSensors()
{
    if(!initialized_)
    {
        std::cerr << "IMU not initialized. Call initialize() first." << std::endl;
        return false;
    }
    
    bool success = true;
    
    // 读取加速度计数据
    raw_data_.accelerometer.x = readRawValue(config_.acc_x_path);
    raw_data_.accelerometer.y = readRawValue(config_.acc_y_path);
    raw_data_.accelerometer.z = readRawValue(config_.acc_z_path);
    
    // 读取陀螺仪数据
    raw_data_.gyroscope.x = readRawValue(config_.gyro_x_path);
    raw_data_.gyroscope.y = readRawValue(config_.gyro_y_path);
    raw_data_.gyroscope.z = readRawValue(config_.gyro_z_path);
    
    // 只有IMU963RA型号才读取磁力计数据
    if(device_type_ == ImuDeviceType::IMU963RA)
    {
        raw_data_.magnetometer.x = readRawValue(config_.mag_x_path);
        raw_data_.magnetometer.y = readRawValue(config_.mag_y_path);
        raw_data_.magnetometer.z = readRawValue(config_.mag_z_path);
    }
    else
    {
        raw_data_.magnetometer = Vector3D();
    }
    
    return success;
}

//--------------------------------------------------------------------
// 公共方法：读取加速度计数据
//--------------------------------------------------------------------
bool ImuDeviceCore::readAccelerometer()
{
    if(!initialized_)
    {
        std::cerr << "IMU not initialized. Call initialize() first." << std::endl;
        return false;
    }
    
    raw_data_.accelerometer.x = readRawValue(config_.acc_x_path);
    raw_data_.accelerometer.y = readRawValue(config_.acc_y_path);
    raw_data_.accelerometer.z = readRawValue(config_.acc_z_path);
    
    return true;
}

//--------------------------------------------------------------------
// 公共方法：读取陀螺仪数据
//--------------------------------------------------------------------
bool ImuDeviceCore::readGyroscope()
{
    if(!initialized_)
    {
        std::cerr << "IMU not initialized. Call initialize() first." << std::endl;
        return false;
    }
    
    raw_data_.gyroscope.x = readRawValue(config_.gyro_x_path);
    raw_data_.gyroscope.y = readRawValue(config_.gyro_y_path);
    raw_data_.gyroscope.z = readRawValue(config_.gyro_z_path);
    
    return true;
}

//--------------------------------------------------------------------
// 公共方法：读取磁力计数据
//--------------------------------------------------------------------
bool ImuDeviceCore::readMagnetometer()
{
    if(!initialized_)
    {
        std::cerr << "IMU not initialized. Call initialize() first." << std::endl;
        return false;
    }
    
    if(device_type_ != ImuDeviceType::IMU963RA)
    {
        std::cerr << "Magnetometer not available for this device type" << std::endl;
        return false;
    }
    
    raw_data_.magnetometer.x = readRawValue(config_.mag_x_path);
    raw_data_.magnetometer.y = readRawValue(config_.mag_y_path);
    raw_data_.magnetometer.z = readRawValue(config_.mag_z_path);
    
    return true;
}

//--------------------------------------------------------------------
// 公共方法：获取设备类型字符串
//--------------------------------------------------------------------
std::string ImuDeviceCore::getDeviceTypeString() const
{
    return deviceTypeToString(device_type_);
}

//--------------------------------------------------------------------
// 调试方法：打印状态
//--------------------------------------------------------------------
void ImuDeviceCore::printStatus() const
{
    std::cout << "=== IMU Device Status ===" << std::endl;
    std::cout << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
    std::cout << "Device Type: " << getDeviceTypeString() << std::endl;
    std::cout << "========================" << std::endl;
}

//--------------------------------------------------------------------
// 调试方法：打印原始数据
//--------------------------------------------------------------------
void ImuDeviceCore::printRawData() const
{
    std::cout << "=== IMU Raw Data ===" << std::endl;
    std::cout << "Accelerometer: [" 
              << raw_data_.accelerometer.x << ", "
              << raw_data_.accelerometer.y << ", "
              << raw_data_.accelerometer.z << "]" << std::endl;
    
    std::cout << "Gyroscope:     [" 
              << raw_data_.gyroscope.x << ", "
              << raw_data_.gyroscope.y << ", "
              << raw_data_.gyroscope.z << "]" << std::endl;
    
    if(device_type_ == ImuDeviceType::IMU963RA)
    {
        std::cout << "Magnetometer:  [" 
                  << raw_data_.magnetometer.x << ", "
                  << raw_data_.magnetometer.y << ", "
                  << raw_data_.magnetometer.z << "]" << std::endl;
    }
    std::cout << "==================" << std::endl;
}

} // namespace zf