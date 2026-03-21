// 1.0 zf_device_imu_core.cpp - 优化版本

#include "zf_device_imu_core.h"
#include "zf_driver_file.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
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
      is_initialized_(false),
      zero_drift_bias_x_(0.0f),
      zero_drift_bias_y_(0.0f),
      zero_drift_bias_z_(0.0f)
{
    // 初始化传感器数据结构
    memset(&raw_data_, 0, sizeof(raw_data_));
    memset(&compensated_unit_data_, 0, sizeof(compensated_unit_data_));
    
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
    unit_data_.acc_x = raw_data_.acc_x / 4096.0f;
    raw_data_.acc_y = read_sensor_data(SENSOR_ACC_Y);
    unit_data_.acc_y = raw_data_.acc_y / 4096.0f;
    raw_data_.acc_z = read_sensor_data(SENSOR_ACC_Z);
    unit_data_.acc_z = raw_data_.acc_z / 4096.0f;
    
    // 读取陀螺仪数据
    raw_data_.gyro_x = read_sensor_data(SENSOR_GYRO_X);
    unit_data_.gyro_x = raw_data_.gyro_x / 16.4f;
    raw_data_.gyro_y = read_sensor_data(SENSOR_GYRO_Y);
    unit_data_.gyro_y = raw_data_.gyro_y / 16.4f;
    raw_data_.gyro_z = read_sensor_data(SENSOR_GYRO_Z);
    unit_data_.gyro_z = raw_data_.gyro_z / 16.4f;

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
// 获取带单位的数据
// 加速度=原始数据/4096，单位为g
// 陀螺仪=原始数据/16.4，单位为°/s
//-------------------------------------------------------------------------------------------------------------------
const imu_unit_data_t& IMUDevice::get_unit_data() const
{
    return unit_data_;
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

//-------------------------------------------------------------------------------------------------------------------
// 零漂测量函数
//-------------------------------------------------------------------------------------------------------------------
bool IMUDevice::measure_zero_drift()
{
    float bias_x, bias_y, bias_z;
    if (!is_initialized_) {
        printf("IMU device not initialized!\n");
        return false;
    }

    const int sample_count = 200 * 3;   // 3秒
    int valid_count = 0;

    double sum_x = 0, sum_y = 0, sum_z = 0;
    double sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;

    printf("Zero drift calibration start...\n");
    printf("Please keep the robot completely STILL!\n");

    for(int i = 0; i < sample_count; i++)
    {
        if(!update_all_data())
            continue;

        const imu_unit_data_t& unit = get_unit_data();

        // 判断是否接近静止（简单判断法）
        if (fabs(unit.gyro_x) > 5 ||
            fabs(unit.gyro_y) > 5 ||
            fabs(unit.gyro_z) > 5)
        {
            printf("Motion detected! Calibration failed.\n");
            return false;
        }

        sum_x += unit.gyro_x;
        sum_y += unit.gyro_y;
        sum_z += unit.gyro_z;

        sum_sq_x += unit.gyro_x * unit.gyro_x;
        sum_sq_y += unit.gyro_y * unit.gyro_y;
        sum_sq_z += unit.gyro_z * unit.gyro_z;

        valid_count++;

        usleep(5000);  // 200Hz
    }

    if(valid_count < sample_count * 0.9)
    {
        printf("Not enough valid samples!\n");
        return false;
    }

    bias_x = sum_x / valid_count;
    bias_y = sum_y / valid_count;
    bias_z = sum_z / valid_count;

    double var_z = sum_sq_z / valid_count - bias_z * bias_z;

    printf("\n===== Zero Drift Result =====\n");
    printf("Bias X: %.6f °/s\n", bias_x);
    printf("Bias Y: %.6f °/s\n", bias_y);
    printf("Bias Z: %.6f °/s\n", bias_z);
    printf("Z variance: %.6f\n", var_z);

    if(var_z > 0.5)
    {
        printf("Warning: IMU noise too large!\n");
        return false;
    }

    // 设置零漂偏置
    set_zero_drift_bias(bias_x, bias_y, bias_z);

    printf("Calibration success.\n\n");
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 设置零漂偏置
//-------------------------------------------------------------------------------------------------------------------
void IMUDevice::set_zero_drift_bias(float bias_x, float bias_y, float bias_z)
{
    zero_drift_bias_x_ = bias_x;
    zero_drift_bias_y_ = bias_y;
    zero_drift_bias_z_ = bias_z;
    
    printf("Zero drift bias set: X=%.6f, Y=%.6f, Z=%.6f °/s\n", 
           zero_drift_bias_x_, zero_drift_bias_y_, zero_drift_bias_z_);
}

//-------------------------------------------------------------------------------------------------------------------
// 应用零漂补偿
//-------------------------------------------------------------------------------------------------------------------
void IMUDevice::apply_zero_drift_compensation()
{
    // 复制原始单位数据
    compensated_unit_data_ = unit_data_;
    
    // 应用零漂补偿（只补偿陀螺仪数据）
    compensated_unit_data_.gyro_x = unit_data_.gyro_x - zero_drift_bias_x_;
    compensated_unit_data_.gyro_y = unit_data_.gyro_y - zero_drift_bias_y_;
    compensated_unit_data_.gyro_z = unit_data_.gyro_z - zero_drift_bias_z_;
    
    // 加速度和磁力计数据保持不变
    compensated_unit_data_.acc_x = unit_data_.acc_x;
    compensated_unit_data_.acc_y = unit_data_.acc_y;
    compensated_unit_data_.acc_z = unit_data_.acc_z;
    compensated_unit_data_.mag_x = unit_data_.mag_x;
    compensated_unit_data_.mag_y = unit_data_.mag_y;
    compensated_unit_data_.mag_z = unit_data_.mag_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 获取补偿后的数据
//-------------------------------------------------------------------------------------------------------------------
const imu_unit_data_t& IMUDevice::get_compensated_unit_data() const
{
    return compensated_unit_data_;
}
