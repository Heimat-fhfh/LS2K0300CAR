// 1.2 zf_device_imu_core_v2.hpp - C++面向对象设计

#ifndef _ZF_DEVICE_IMU_CORE_V2_HPP_
#define _ZF_DEVICE_IMU_CORE_V2_HPP_

#include <cstdint>
#include <string>

namespace zf {

// IMU设备类型枚举
enum class ImuDeviceType : uint8_t
{
    NO_DEVICE  = 0,    // 未识别到设备
    IMU660RA   = 1,    // IMU660RA型号
    IMU660RB   = 2,    // IMU660RB型号
    IMU963RA   = 4     // IMU963RA型号(带磁力计)
};

// 传感器数据结构体
struct ImuSensorData
{
    int16_t acc_x;    // 加速度X轴
    int16_t acc_y;    // 加速度Y轴
    int16_t acc_z;    // 加速度Z轴
    int16_t gyro_x;   // 陀螺仪X轴
    int16_t gyro_y;   // 陀螺仪Y轴
    int16_t gyro_z;   // 陀螺仪Z轴
    int16_t mag_x;    // 磁力计X轴
    int16_t mag_y;    // 磁力计Y轴
    int16_t mag_z;    // 磁力计Z轴
    
    // 构造函数
    ImuSensorData() : 
        acc_x(0), acc_y(0), acc_z(0),
        gyro_x(0), gyro_y(0), gyro_z(0),
        mag_x(0), mag_y(0), mag_z(0) {}
    
    // 重置所有数据为0
    void reset() {
        acc_x = acc_y = acc_z = 0;
        gyro_x = gyro_y = gyro_z = 0;
        mag_x = mag_y = mag_z = 0;
    }
    
    // 转换为字符串用于调试
    std::string toString() const;
};

// 三轴向量数据
struct Vector3D
{
    int16_t x;
    int16_t y;
    int16_t z;
    
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(int16_t x_val, int16_t y_val, int16_t z_val) : 
        x(x_val), y(y_val), z(z_val) {}
};

// 传感器原始数据集合
struct RawImuData
{
    Vector3D accelerometer;  // 加速度计数据
    Vector3D gyroscope;      // 陀螺仪数据
    Vector3D magnetometer;   // 磁力计数据
    
    RawImuData() = default;
};

// IMU设备配置结构体
struct ImuConfig
{
    std::string event_path;      // 事件文件路径
    std::string name_path;       // 名称文件路径
    std::string acc_x_path;      // 加速度X轴路径
    std::string acc_y_path;      // 加速度Y轴路径
    std::string acc_z_path;      // 加速度Z轴路径
    std::string gyro_x_path;     // 陀螺仪X轴路径
    std::string gyro_y_path;     // 陀螺仪Y轴路径
    std::string gyro_z_path;     // 陀螺仪Z轴路径
    std::string mag_x_path;      // 磁力计X轴路径
    std::string mag_y_path;      // 磁力计Y轴路径
    std::string mag_z_path;      // 磁力计Z轴路径
    
    // 默认构造函数使用默认路径
    ImuConfig();
};

//--------------------------------------------------------------------
// 类：IMU设备核心类
//--------------------------------------------------------------------
class ImuDeviceCore
{
private:
    // 私有成员变量
    ImuDeviceType device_type_;   // 设备类型
    ImuConfig config_;           // 配置信息
    RawImuData raw_data_;        // 原始传感器数据
    bool initialized_;           // 初始化标志
    
    // 私有方法
    bool initViaEventFile();     // 通过事件文件初始化
    bool initViaNameFile();      // 通过名称文件初始化
    int16_t readRawValue(const std::string& path) const; // 读取原始值
    
    // 禁用拷贝构造和赋值
    ImuDeviceCore(const ImuDeviceCore&) = delete;
    ImuDeviceCore& operator=(const ImuDeviceCore&) = delete;

public:
    // 构造函数和析构函数
    ImuDeviceCore();
    explicit ImuDeviceCore(const ImuConfig& config);
    ~ImuDeviceCore() = default;
    
    // 初始化方法
    bool initialize();
    
    // 状态查询
    bool isInitialized() const { return initialized_; }
    ImuDeviceType getDeviceType() const { return device_type_; }
    std::string getDeviceTypeString() const;
    
    // 数据读取方法 - 批量读取
    bool readAllSensors();
    bool readAccelerometer();
    bool readGyroscope();
    bool readMagnetometer();
    
    // 数据获取方法
    const RawImuData& getRawData() const { return raw_data_; }
    Vector3D getAccelerometerData() const { return raw_data_.accelerometer; }
    Vector3D getGyroscopeData() const { return raw_data_.gyroscope; }
    Vector3D getMagnetometerData() const { return raw_data_.magnetometer; }
    
    // 单个轴数据获取
    int16_t getAccX() const { return raw_data_.accelerometer.x; }
    int16_t getAccY() const { return raw_data_.accelerometer.y; }
    int16_t getAccZ() const { return raw_data_.accelerometer.z; }
    
    int16_t getGyroX() const { return raw_data_.gyroscope.x; }
    int16_t getGyroY() const { return raw_data_.gyroscope.y; }
    int16_t getGyroZ() const { return raw_data_.gyroscope.z; }
    
    int16_t getMagX() const { return raw_data_.magnetometer.x; }
    int16_t getMagY() const { return raw_data_.magnetometer.y; }
    int16_t getMagZ() const { return raw_data_.magnetometer.z; }
    
    // 配置相关
    void setConfig(const ImuConfig& config) { config_ = config; }
    const ImuConfig& getConfig() const { return config_; }
    
    // 静态辅助方法
    static std::string deviceTypeToString(ImuDeviceType type);
    
    // 调试方法
    void printStatus() const;
    void printRawData() const;
};

} // namespace zf

#endif // _ZF_DEVICE_IMU_CORE_V2_HPP_