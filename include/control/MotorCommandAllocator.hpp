#pragma once

#include <algorithm>
#include <cmath>

namespace MotorCommandAllocator {

struct WheelCommands {
    double left;
    double right;
    double base;
    double diff;
};

inline WheelCommands allocateDifferentialPriority(double speedOutput, double diffOutput) {
    constexpr double motorMinCmd = 0.0;
    constexpr double motorMaxCmd = 1.0;
    constexpr double maxDiffCmd = (motorMaxCmd - motorMinCmd) * 0.5;

    const double diffCmd = std::clamp(diffOutput, -maxDiffCmd, maxDiffCmd);
    const double absDiff = std::abs(diffCmd);
    const double baseCmd = std::clamp(speedOutput, motorMinCmd + absDiff, motorMaxCmd - absDiff);

    return {
        baseCmd - diffCmd,
        baseCmd + diffCmd,
        baseCmd,
        diffCmd
    };
}

} // namespace MotorCommandAllocator
