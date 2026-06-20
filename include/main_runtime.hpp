#pragma once

#include "main.hpp"

class Buzzer;
Buzzer& GetBuzzer();

bool ParseCameraFpsArgument(int argc, char** argv);
bool IsVideoSpeedTestMode(int argc, char** argv);
bool IsMotorDeadMode(int argc, char** argv);

int RunVideoSpeedTest();
int RunMotorDeadZoneMode();
bool StartMotorControlTask();

void argument_config(void);
void sigint_handler(int signum);
void cleanup();
int main_init_task();
int main_test_task(const MainTestConfig& test_config);
