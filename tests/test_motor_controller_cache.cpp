#include "MotorController.h"

#include <cstdlib>
#include <iostream>

namespace
{
int g_gpioWrites = 0;
int g_pwmWrites = 0;
uint8 g_lastGpioLevel = 0;
uint16 g_lastPwmDuty = 0;

void expectEqual(const char* name, int actual, int expected)
{
    if (actual != expected)
    {
        std::cerr << name << ": expected " << expected << ", got " << actual << std::endl;
        std::exit(EXIT_FAILURE);
    }
}
}

void gpio_set_level(const char*, uint8 dat)
{
    ++g_gpioWrites;
    g_lastGpioLevel = dat;
}

uint8 gpio_get_level(const char*)
{
    return g_lastGpioLevel;
}

void pwm_set_duty(const char*, uint16 duty)
{
    ++g_pwmWrites;
    g_lastPwmDuty = duty;
}

int main()
{
    auto* motor = new MotorController("dir", "pwm");

    motor->setSpeed(0.5f);
    expectEqual("first gpio write count", g_gpioWrites, 1);
    expectEqual("first pwm write count", g_pwmWrites, 1);
    if (g_lastPwmDuty == 0)
    {
        std::cerr << "expected non-zero PWM duty after setSpeed(0.5)" << std::endl;
        return EXIT_FAILURE;
    }

    motor->setSpeed(0.5f);
    expectEqual("repeated speed gpio write count", g_gpioWrites, 1);
    expectEqual("repeated speed pwm write count", g_pwmWrites, 1);

    motor->setSpeed(0.0f);
    expectEqual("stop pwm write count", g_pwmWrites, 2);
    expectEqual("stop pwm duty", static_cast<int>(g_lastPwmDuty), 0);

    motor->setSpeed(0.0f);
    expectEqual("repeated stop pwm write count", g_pwmWrites, 2);

    return EXIT_SUCCESS;
}
