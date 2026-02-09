// 1.0 zf_device_imu_core.cpp - 优化版本

#include "zf_device_imu_core.h"
#include "zf_driver_file.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

//===================================================================================================================
// IMUDevice 类实现
//===================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 构造函数
//-------------------------------------------------------------------------------------------------------------------
IMUDevice::IMUDevice() 
    : device_type_(IMU_DEV_NO_FIND), 
      is_initialized_(false)
{
    // 初始化传感器数据结构
    memset(&raw_data_, 0, sizeof(raw_data_));
    
    // 初始化文件句柄数组为无效值
    for (int i = 0; i < 9; ++i) {
        sensor_fds_[i] = -1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 析构函数
//-------------------------------------------------------------------------------------------------------------------
IMUDevice::~IMUDevice()
{
    close_sensor_files();
}

//-------------------------------------------------------------------------------------------------------------------
// 关闭所有传感器文件
//-------------------------------------------------------------------------------------------------------------------
void IMUDevice::close_sensor_files()
{
    for (int i = 0; i < 9; ++i) {
        if (sensor_fds_[i] > 0) {
            close(sensor_fds_[i]);
            sensor_fds_[i] = -1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 打开传感器文件
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::open_sensor_files()
{
    bool success = true;
    
    // 根据设备类型决定打开哪些文件
    int sensors_to_open = 6; // 默认打开6个传感器（加速度+陀螺仪）
    
    if (device_type_ == IMU_DEV_IMU963RA) {
        sensors_to_open = 9; // 963RA打开所有9个传感器（包括磁力计）
    }
    
    // 打开需要的传感器文件
    for (int i = 0; i < sensors_to_open; ++i) {
        sensor_fds_[i] = open(SENSOR_PATHS[i], O_RDONLY);
        if (sensor_fds_[i] < 0) {
            printf("Failed to open sensor file %s: errno=%d\n", SENSOR_PATHS[i], errno);
            success = false;
            break;
        }
    }
    
    // 如果打开失败，关闭所有已打开的文件
    if (!success) {
        close_sensor_files();
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
// 读取单个传感器数据
//-------------------------------------------------------------------------------------------------------------------
int16_t IMUDevice::read_sensor_data(sensor_index_t index)
{
    if (!is_initialized_ || sensor_fds_[index] < 0) {
        return 0;
    }
    
    char buffer[20] = {0};
    
    // 重置文件指针到开头
    lseek(sensor_fds_[index], 0, SEEK_SET);
    
    // 读取数据
    ssize_t bytes_read = read(sensor_fds_[index], buffer, sizeof(buffer) - 1);
    if (bytes_read <= 0) {
        return 0;
    }
    
    // 转换为整数
    return static_cast<int16_t>(atoi(buffer));
}

//-------------------------------------------------------------------------------------------------------------------
// 检查设备类型
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::check_device_type(const char* device_name)
{
    if (strcmp(device_name, "IMU660RA") == 0) {
        device_type_ = IMU_DEV_IMU660RA;
        return true;
    }
    else if (strcmp(device_name, "IMU660RB") == 0) {
        device_type_ = IMU_DEV_IMU660RB;
        return true;
    }
    else if (strcmp(device_name, "IMU963RA") == 0) {
        device_type_ = IMU_DEV_IMU963RA;
        return true;
    }
    
    device_type_ = IMU_DEV_NO_FIND;
    return false;
}

//-------------------------------------------------------------------------------------------------------------------
// 初始化设备（基于1.0内核，使用name文件识别设备）
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::initialize()
{
    // 如果已经初始化，直接返回成功
    if (is_initialized_) {
        return true;
    }
    
    char device_name[32] = {0};
    
    // 步骤1：读取设备名称文件
    if (file_read_string(DEVICE_NAME_PATH, device_name) < 0) {
        printf("Failed to read IMU device name from %s\n", DEVICE_NAME_PATH);
        device_type_ = IMU_DEV_NO_FIND;
        return false;
    }
    
    // 步骤2：根据设备名称确定设备类型
    if (!check_device_type(device_name)) {
        printf("Unknown IMU device: %s\n", device_name);
        return false;
    }
    
    printf("Detected IMU device: %s (type=%d)\n", device_name, device_type_);
    
    // 步骤3：打开传感器文件
    if (!open_sensor_files()) {
        printf("Failed to open sensor files for IMU device\n");
        device_type_ = IMU_DEV_NO_FIND;
        return false;
    }
    
    // 步骤4：尝试读取一次数据以验证设备正常工作
    is_initialized_ = true;
    if (!update_all_data()) {
        is_initialized_ = false;
        printf("Failed to read initial data from IMU device\n");
        close_sensor_files();
        device_type_ = IMU_DEV_NO_FIND;
        return false;
    }
    
    is_initialized_ = true;
    printf("IMU device initialized successfully\n");
    
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 更新所有传感器数据
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::update_all_data()
{
    if (!is_initialized_) {
        return false;
    }
    
    // 读取加速度数据
    raw_data_.acc_x = read_sensor_data(SENSOR_ACC_X);
    raw_data_.acc_y = read_sensor_data(SENSOR_ACC_Y);
    raw_data_.acc_z = read_sensor_data(SENSOR_ACC_Z);
    
    // 读取陀螺仪数据
    raw_data_.gyro_x = read_sensor_data(SENSOR_GYRO_X);
    raw_data_.gyro_y = read_sensor_data(SENSOR_GYRO_Y);
    raw_data_.gyro_z = read_sensor_data(SENSOR_GYRO_Z);
    
    
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 获取设备类型
//-------------------------------------------------------------------------------------------------------------------
imu_device_type_t IMUDevice::get_device_type() const
{
    return device_type_;
}

//-------------------------------------------------------------------------------------------------------------------
// 检查是否已初始化
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::is_initialized() const
{
    return is_initialized_;
}

//-------------------------------------------------------------------------------------------------------------------
// 获取原始数据（const引用，避免拷贝）
//-------------------------------------------------------------------------------------------------------------------
const imu_raw_data_t& IMUDevice::get_raw_data() const
{
    return raw_data_;
}

//-------------------------------------------------------------------------------------------------------------------
// 单个传感器数据获取方法
//-------------------------------------------------------------------------------------------------------------------
int16_t IMUDevice::get_acc_x() const { return raw_data_.acc_x; }
int16_t IMUDevice::get_acc_y() const { return raw_data_.acc_y; }
int16_t IMUDevice::get_acc_z() const { return raw_data_.acc_z; }
int16_t IMUDevice::get_gyro_x() const { return raw_data_.gyro_x; }
int16_t IMUDevice::get_gyro_y() const { return raw_data_.gyro_y; }
int16_t IMUDevice::get_gyro_z() const { return raw_data_.gyro_z; }
int16_t IMUDevice::get_mag_x() const { return raw_data_.mag_x; }
int16_t IMUDevice::get_mag_y() const { return raw_data_.mag_y; }
int16_t IMUDevice::get_mag_z() const { return raw_data_.mag_z; }