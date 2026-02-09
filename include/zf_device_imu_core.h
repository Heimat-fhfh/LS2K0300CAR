// 1.0 zf_device_imu_core.h - 优化版本

#ifndef _ZF_DEVICE_IMU_CORE_H_
#define _ZF_DEVICE_IMU_CORE_H_

#include "zf_common_typedef.h"
#include <stdint.h>

//===================================================================================================================
// IMU设备类型枚举
//===================================================================================================================
typedef enum
{
    IMU_DEV_NO_FIND    = 0,    // 未识别到设备
    IMU_DEV_IMU660RA   = 1,    // IMU660RA型号
    IMU_DEV_IMU660RB   = 2,    // IMU660RB型号
    IMU_DEV_IMU963RA   = 3     // IMU963RA型号(带磁力计)
} imu_device_type_t;

//===================================================================================================================
// IMU传感器数据结构体
//===================================================================================================================
typedef struct
{
    int16_t acc_x;      // 加速度X轴
    int16_t acc_y;      // 加速度Y轴
    int16_t acc_z;      // 加速度Z轴
    int16_t gyro_x;     // 陀螺仪X轴
    int16_t gyro_y;     // 陀螺仪Y轴
    int16_t gyro_z;     // 陀螺仪Z轴
    int16_t mag_x;      // 磁力计X轴
    int16_t mag_y;      // 磁力计Y轴
    int16_t mag_z;      // 磁力计Z轴
} imu_raw_data_t;

//===================================================================================================================
// IMU设备类
//===================================================================================================================
class IMUDevice
{
private:
    // 私有成员变量
    imu_device_type_t device_type_;     // 设备类型
    imu_raw_data_t raw_data_;          // 原始数据
    bool is_initialized_;              // 初始化标志
    
    // 文件句柄（使用数组简化管理）
    int sensor_fds_[9];                // 9个传感器文件句柄
    
    // 路径常量（基于1.0内核）
    static constexpr const char* DEVICE_NAME_PATH = "/sys/bus/iio/devices/iio:device1/name";
    
    static constexpr const char* SENSOR_PATHS[9] = {
        "/sys/bus/iio/devices/iio:device1/in_accel_x_raw",
        "/sys/bus/iio/devices/iio:device1/in_accel_y_raw",
        "/sys/bus/iio/devices/iio:device1/in_accel_z_raw",
        "/sys/bus/iio/devices/iio:device1/in_anglvel_x_raw",
        "/sys/bus/iio/devices/iio:device1/in_anglvel_y_raw",
        "/sys/bus/iio/devices/iio:device1/in_anglvel_z_raw",
        "/sys/bus/iio/devices/iio:device1/in_magn_x_raw",
        "/sys/bus/iio/devices/iio:device1/in_magn_y_raw",
        "/sys/bus/iio/devices/iio:device1/in_magn_z_raw"
    };
    
    // 传感器索引枚举
    typedef enum {
        SENSOR_ACC_X = 0,
        SENSOR_ACC_Y,
        SENSOR_ACC_Z,
        SENSOR_GYRO_X,
        SENSOR_GYRO_Y,
        SENSOR_GYRO_Z,
        SENSOR_MAG_X,
        SENSOR_MAG_Y,
        SENSOR_MAG_Z
    } sensor_index_t;
    
    // 私有方法
    bool open_sensor_files();
    void close_sensor_files();
    int16_t read_sensor_data(sensor_index_t index);
    bool check_device_type(const char* device_name);
    
    // 禁用拷贝构造和赋值
    IMUDevice(const IMUDevice&) = delete;
    IMUDevice& operator=(const IMUDevice&) = delete;
    
public:
    // 构造函数和析构函数
    IMUDevice();
    ~IMUDevice();
    
    // 公共接口
    bool initialize();                          // 初始化设备
    imu_device_type_t get_device_type() const;  // 获取设备类型
    bool is_initialized() const;                // 检查是否初始化
    
    // 数据读取接口
    bool update_all_data();                     // 更新所有数据
    const imu_raw_data_t& get_raw_data() const; // 获取原始数据
    
    // 单个数据读取（可选）
    int16_t get_acc_x() const;
    int16_t get_acc_y() const;
    int16_t get_acc_z() const;
    int16_t get_gyro_x() const;
    int16_t get_gyro_y() const;
    int16_t get_gyro_z() const;
    int16_t get_mag_x() const;
    int16_t get_mag_y() const;
    int16_t get_mag_z() const;
};

#endif // _ZF_DEVICE_IMU_CORE_H_