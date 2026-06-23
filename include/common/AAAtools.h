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
