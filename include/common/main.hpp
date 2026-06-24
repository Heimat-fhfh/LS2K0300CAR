#pragma once



#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <cmath>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <math.h>		
#include <stdlib.h>
#include <thread>

#include "common/base.h"
#include "common/common_system.h"
#include "common/common_program.h"
#include "stdio.h"

#include "vision/libimage_process.h"
#include "vision/libdata_process.h"
#include "vision/libdata_store.h"

#if defined(MAKE_MAIN_CPP)

	#include "drivers/zf_common_headfile.h"
	#include "drivers/zf_driver_udp.hpp"

	#include "control/PID.hpp"
	#include "vision/AAAdefine.h"

	#include "devices/display_show.h"

	#include "devices/encoder.hpp"
	#include "control/DualMotorController.h"
	#include "control/MotorControlTask.hpp"
	#include "devices/buzzer.hpp"

	#define SERVO_MOTOR1_PWM        "/dev/zf_device_pwm_servo"

	#define MOTOR1_DIR              "/dev/zf_driver_gpio_motor_1"
	#define MOTOR1_PWM              "/dev/zf_device_pwm_motor_1"

	#define MOTOR2_DIR              "/dev/zf_driver_gpio_motor_2"
	#define MOTOR2_PWM              "/dev/zf_device_pwm_motor_2"

	#define ENCODER_1               "/dev/zf_encoder_1"
	#define ENCODER_2               "/dev/zf_encoder_2"


	#define ADC_REG_PATH   "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"
	#define ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

	#define KEY_0       "/dev/zf_driver_gpio_key_0"
	#define KEY_1       "/dev/zf_driver_gpio_key_1"
	#define KEY_2       "/dev/zf_driver_gpio_key_2"
	#define KEY_3       "/dev/zf_driver_gpio_key_3"
	#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
	#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

	// 另外一端的IP地址
	#define SERVER_IP "172.25.80.183"
	// 端口号
	#define PORT 8086

	// 定义主板上舵机频率  请务必注意范围 50-300
	// 如果要修改，需要直接修改设备树。
	#define SERVO_MOTOR_FREQ            (servo_pwm_info.freq)                       

	// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
	#define PWM_DUTY_MAX                (servo_pwm_info.duty_max)                         

	#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))


	struct MainTestConfig {
		bool buzzer_test = false;
		bool imu_test = false;
		bool motor_test = false;
		bool angular_velocity_test = false;
		bool encoder_test = false;
	};

	
	extern IMUDevice imu;
	extern std::unique_ptr<DualMotorController> motors;
	extern Encoder encoder_left;
	extern Encoder encoder_right;
	extern Control::PID::Parameters diffOuterParams;
	extern Control::PID::Parameters diffInnerParams;
	extern Control::IncrementalPID::Parameters speedIncrParams;
	extern std::unique_ptr<MotorControlTask> motorTask;
	extern MainTestConfig test_config;
	extern bool g_runtime_config_ok;
	extern CameraKind g_camera_kind;
	extern int g_camera_fps;
	extern bool g_simple_tracking_enabled;
	extern JSON_DifferentialPDConfigData JSON_DifferentialPDConfigData_s;
	extern JSON_AngularVelocityPIDConfigData JSON_AngularVelocityPIDConfigData_s;
	extern JSON_SpeedIncrementalPIConfigData JSON_SpeedIncrementalPIConfigData_s;
	extern JSON_VehicleConfigData JSON_VehicleConfigData_s;
	extern Function_EN Function_EN_s;
	extern Data_Path Data_Path_s;
	extern ImgProcess imgProcess;
	extern Judge judge;
	extern SYNC Sync;

#endif // MAKE_MAIN_CPP

extern std::atomic<bool> g_running;

void argument_config(void);
void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p);
bool BatteryVoltageCheck(double threshold_mv);
void FrameTaskAfterRead(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p, ImgProcess *imgProcess_p, Judge *judge_p);
