#include <iostream>
#include <string>
#include "AAAtools.h"


void Display::update(const std::string& data) {
    // 模拟显示更新
    std::cout << "\033[2J\033[1;1H"; // 清屏
    std::cout << "======== 智能小车状态 ========" << std::endl;
    std::cout << data << std::endl;
    std::cout << "=============================" << std::endl;
}

