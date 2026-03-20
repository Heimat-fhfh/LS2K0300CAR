#include "buzzer.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main() {
    try {
        Buzzer buzzer;

        buzzer.setShortDuration(100)
              .setLongDuration(500)
              .setIntervalDuration(200);

        std::cout << "短鸣 1 次" << std::endl;
        buzzer.shortBeep();
        std::this_thread::sleep_for(std::chrono::milliseconds(600));

        std::cout << "双短鸣" << std::endl;
        buzzer.patternDoubleShort();
        std::this_thread::sleep_for(std::chrono::milliseconds(900));

        std::cout << "一长一短" << std::endl;
        buzzer.patternLongShort();
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));

        std::cout << "自定义模式（300ms 响 / 100ms 停，重复 3 次）" << std::endl;
        buzzer.customPattern({300}, {100}, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        std::cout << "连续鸣叫 2 秒" << std::endl;
        buzzer.patternContinuous();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        buzzer.stop();

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Buzzer example failed: " << e.what() << std::endl;
        return 1;
    }
}