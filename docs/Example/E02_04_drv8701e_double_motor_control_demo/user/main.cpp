
#include "zf_common_headfile.h"

#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"


struct pwm_info servo_pwm_info;


int8 duty = 0;
bool dir = true;

#define PWM_DUTY_MAX    (10000)         // 在设备树中，设置的10000。如果要修改，需要与设备树对应。

#define MAX_DUTY        (30 )           // 最大 MAX_DUTY% 占空比

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    pwm_set_duty(MOTOR1_PWM, 0);   
    pwm_set_duty(MOTOR2_PWM, 0);    
}

int main(int, char**) 
{

    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    while(1)
    {
        if(duty >= 0)                                                   // 正转
        {
            gpio_set_level(MOTOR1_DIR, 1);                              // DIR输出高电平
            pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));      // 计算占空比

            gpio_set_level(MOTOR2_DIR, 1);                              // DIR输出高电平
            pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));      // 计算占空比
        }
        else
        {
            gpio_set_level(MOTOR1_DIR, 0);                              // DIR输出低电平
            pwm_set_duty(MOTOR1_PWM, -duty * (PWM_DUTY_MAX / 100));     // 计算占空比

            gpio_set_level(MOTOR2_DIR, 0);                              // DIR输出低电平
            pwm_set_duty(MOTOR2_PWM, -duty * (PWM_DUTY_MAX / 100));     // 计算占空比

        }

        if(dir)                                                         // 根据方向判断计数方向 本例程仅作参考
        {
            duty ++;                                                    // 正向计数
            if(duty >= MAX_DUTY)                                        // 达到最大值
            dir = false;                                                // 变更计数方向
        }
        else
        {
            duty --;                                                    // 反向计数
            if(duty <= -MAX_DUTY)                                       // 达到最小值
            dir = true;                                                 // 变更计数方向
        }

        system_delay_ms(50);
    }
}