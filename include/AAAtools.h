#pragma once

#include <iostream>
#include <cstdint>
#include <chrono>
#include <vector>
#include <string>
#include <termios.h>

namespace cv {
class Mat;
}

class Display {
public:
    void update(const std::string& data);
};

class PerfWindowRecorder {
public:
    explicit PerfWindowRecorder(uint64_t windowSize, bool enablePerFrameLog);
    void record(const std::chrono::steady_clock::duration& frameCost,
                const std::chrono::steady_clock::duration& captureCost,
                const std::chrono::steady_clock::duration& undistortCost,
                bool undistortExecuted);
    void flush();

private:
    uint64_t windowSize_ = 30;
    bool enablePerFrameLog_ = false;
    uint64_t frameIndex_ = 0;
    uint64_t windowStartFrame_ = 0;
    uint64_t undistortExecutedCount_ = 0;
    std::vector<long long> totalSamples_;
    std::vector<long long> captureSamples_;
    std::vector<long long> undistortSamples_;
};

class TempCaptureSession {
public:
    explicit TempCaptureSession(bool enableTempCapture);
    ~TempCaptureSession();
    bool handleKeyEvent();
    void saveFrameIfNeeded(const cv::Mat& frame);
    void printSummary() const;

private:
    void ensureOutputDirReady();
    bool enableTerminalRawMode();
    bool pollTerminalKeyPress();

    bool enableTempCapture_ = false;
    bool keyCaptureReady_ = false;
    bool saveEnabled_ = false;
    bool saveSessionStarted_ = false;
    std::string outputDir_;
    uint64_t outputIndex_ = 0;
    std::vector<cv::Mat> cachedFrames_;
    bool terminalRawModeEnabled_ = false;
    termios oldTermios_{};
};
