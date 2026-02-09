#pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <math.h>		
#include <stdlib.h>
#include <thread>
#include <atomic>
#include "base.h"
#include "common_system.h"
#include "common_program.h"
#include "stdio.h"
#include "zf_common_headfile.h"

#include "libimage_process.h"
#include "libdata_process.h"
#include "libdata_store.h"

#include "PID.h"
#include "AAAdefine.h"

#include "myacross.h"
#include "display_show.h"


#define SERVO_MOTOR1_PWM        "/dev/zf_device_pwm_servo"

#define MOTOR1_DIR              "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM              "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR              "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM              "/dev/zf_device_pwm_motor_2"

#define ENCODER_1               "/dev/zf_encoder_1"
#define ENCODER_2               "/dev/zf_encoder_2"


#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"


// 定义主板上舵机频率  请务必注意范围 50-300
// 如果要修改，需要直接修改设备树。
#define SERVO_MOTOR_FREQ            (servo_pwm_info.freq)                       

// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX                (servo_pwm_info.duty_max)                         

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

void pit_callback(void);
void sigint_handler(int signum);
void cleanup();
int main_init_task();
