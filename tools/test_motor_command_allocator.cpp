#include "MotorCommandAllocator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEqual(double a, double b) {
    return std::abs(a - b) < 1e-9;
}

void expectWheelCommands(const char* name,
                         const MotorCommandAllocator::WheelCommands& actual,
                         double expectedLeft,
                         double expectedRight) {
    if (!nearlyEqual(actual.left, expectedLeft) || !nearlyEqual(actual.right, expectedRight)) {
        std::cerr << name << " failed: expected left=" << expectedLeft
                  << " right=" << expectedRight
                  << ", got left=" << actual.left
                  << " right=" << actual.right << std::endl;
        std::exit(1);
    }
}

} // namespace

int main() {
    expectWheelCommands(
        "moves base to preserve right-turn differential",
        MotorCommandAllocator::allocateDifferentialPriority(0.25, 0.50),
        0.0,
        1.0);

    expectWheelCommands(
        "moves base to preserve left-turn differential",
        MotorCommandAllocator::allocateDifferentialPriority(0.25, -0.50),
        1.0,
        0.0);

    expectWheelCommands(
        "clamps unachievable differential",
        MotorCommandAllocator::allocateDifferentialPriority(0.25, 0.70),
        0.0,
        1.0);

    expectWheelCommands(
        "keeps achievable commands unchanged",
        MotorCommandAllocator::allocateDifferentialPriority(0.70, 0.20),
        0.50,
        0.90);

    return 0;
}
