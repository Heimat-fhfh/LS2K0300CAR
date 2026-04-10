#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <termios.h>
#include <sys/select.h>

#include "AAAtools.h"
#include "main.hpp"

using namespace std;
namespace fs = std::filesystem;

void Display::update(const std::string& data) {
    std::cout << "\033[2J\033[1;1H";
    std::cout << "======== 智能小车状态 ========" << std::endl;
    std::cout << data << std::endl;
    std::cout << "=============================" << std::endl;
}

namespace
{
std::string BuildTimestampString(const std::chrono::system_clock::time_point &tp)
{
    const std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm localTm{};
    localtime_r(&tt, &localTm);

    std::ostringstream oss;
    oss << std::put_time(&localTm, "%Y%m%d_%H%M%S");
    return oss.str();
}

long long DurationToMicroseconds(const std::chrono::steady_clock::duration &d)
{
    return std::chrono::duration_cast<std::chrono::microseconds>(d).count();
}

void LogFrameTiming(uint64_t frameIndex,
                    const std::chrono::steady_clock::duration &frameCost,
                    const std::chrono::steady_clock::duration &captureCost,
                    const std::chrono::steady_clock::duration &undistortCost,
                    bool undistortExecuted)
{
    std::cout << "[Perf] frame=" << frameIndex
              << " total_us=" << DurationToMicroseconds(frameCost)
              << " capture_us=" << DurationToMicroseconds(captureCost)
              << " undistort_us=" << DurationToMicroseconds(undistortCost);
    if (!undistortExecuted)
    {
        std::cout << " (skip)";
    }
    std::cout << std::endl;
}

struct PerfSummary
{
    long long avgUs = 0;
    long long p95Us = 0;
    long long maxUs = 0;
};

PerfSummary BuildPerfSummary(std::vector<long long> samples)
{
    PerfSummary summary{};
    if (samples.empty())
    {
        return summary;
    }

    long long sumUs = 0;
    for (const long long value : samples)
    {
        sumUs += value;
    }
    summary.avgUs = sumUs / static_cast<long long>(samples.size());

    std::sort(samples.begin(), samples.end());
    summary.maxUs = samples.back();

    size_t rank = (95 * samples.size() + 99) / 100;
    if (rank == 0)
    {
        rank = 1;
    }
    summary.p95Us = samples[rank - 1];
    return summary;
}

void LogPerfWindowTiming(uint64_t startFrame,
                         uint64_t endFrame,
                         const std::vector<long long> &totalSamples,
                         const std::vector<long long> &captureSamples,
                         const std::vector<long long> &undistortSamples,
                         uint64_t undistortExecutedCount)
{
    const PerfSummary totalSummary = BuildPerfSummary(totalSamples);
    const PerfSummary captureSummary = BuildPerfSummary(captureSamples);

    std::cout << "[PerfWindow] frame=" << startFrame << "-" << endFrame
              << " count=" << totalSamples.size()
              << " total(avg/p95/max)_us=" << totalSummary.avgUs << "/" << totalSummary.p95Us << "/" << totalSummary.maxUs
              << " capture(avg/p95/max)_us=" << captureSummary.avgUs << "/" << captureSummary.p95Us << "/" << captureSummary.maxUs;

    if (!undistortSamples.empty())
    {
        const PerfSummary undistortSummary = BuildPerfSummary(undistortSamples);
        std::cout << " undistort(avg/p95/max)_us=" << undistortSummary.avgUs << "/" << undistortSummary.p95Us << "/" << undistortSummary.maxUs
                  << " undistort_exec=" << undistortExecutedCount;
    }
    else
    {
        std::cout << " undistort=skip";
    }

    std::cout << std::endl;
}

}

PerfWindowRecorder::PerfWindowRecorder(uint64_t windowSize, bool enablePerFrameLog)
    : windowSize_(windowSize), enablePerFrameLog_(enablePerFrameLog)
{
    totalSamples_.reserve(windowSize_);
    captureSamples_.reserve(windowSize_);
    undistortSamples_.reserve(windowSize_);
}

void PerfWindowRecorder::record(const std::chrono::steady_clock::duration &frameCost,
                                const std::chrono::steady_clock::duration &captureCost,
                                const std::chrono::steady_clock::duration &undistortCost,
                                bool undistortExecuted)
{
    const uint64_t currentFrame = frameIndex_++;
    if (totalSamples_.empty())
    {
        windowStartFrame_ = currentFrame;
    }

    totalSamples_.push_back(DurationToMicroseconds(frameCost));
    captureSamples_.push_back(DurationToMicroseconds(captureCost));

    if (undistortExecuted)
    {
        undistortSamples_.push_back(DurationToMicroseconds(undistortCost));
        ++undistortExecutedCount_;
    }

    if (enablePerFrameLog_)
    {
        LogFrameTiming(currentFrame, frameCost, captureCost, undistortCost, undistortExecuted);
    }

    if (totalSamples_.size() >= windowSize_)
    {
        flush();
    }
}

void PerfWindowRecorder::flush()
{
    if (totalSamples_.empty())
    {
        return;
    }

    const uint64_t windowEndFrame = windowStartFrame_ + totalSamples_.size() - 1;
    LogPerfWindowTiming(
        windowStartFrame_,
        windowEndFrame,
        totalSamples_,
        captureSamples_,
        undistortSamples_,
        undistortExecutedCount_);

    totalSamples_.clear();
    captureSamples_.clear();
    undistortSamples_.clear();
    undistortExecutedCount_ = 0;
}

TempCaptureSession::TempCaptureSession(bool enableTempCapture)
    : enableTempCapture_(enableTempCapture)
{
    keyCaptureReady_ = enableTempCapture_ && enableTerminalRawMode();
    cachedFrames_.reserve(1024);

    if (keyCaptureReady_)
    {
        cout << "按任意键开始保存图像，再按任意键停止并退出..." << endl;
    }
    else
    {
        cout << "终端按键捕获不可用，临时采图功能将不会触发。" << endl;
    }
}

TempCaptureSession::~TempCaptureSession()
{
    if (terminalRawModeEnabled_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios_);
        terminalRawModeEnabled_ = false;
    }
}

bool TempCaptureSession::handleKeyEvent()
{
    if (!keyCaptureReady_ || !pollTerminalKeyPress())
    {
        return true;
    }

    if (!saveEnabled_)
    {
        saveEnabled_ = true;
        saveSessionStarted_ = false;
        outputDir_.clear();
        outputIndex_ = 0;
        cachedFrames_.clear();
        cout << "检测到按键，开始保存图像。" << endl;
        return true;
    }

    saveEnabled_ = false;
    g_running.store(false);
    cout << "检测到按键，停止保存图像。" << endl;
    return false;
}

bool TempCaptureSession::enableTerminalRawMode()
{
    if (!isatty(STDIN_FILENO))
    {
        return false;
    }
    if (tcgetattr(STDIN_FILENO, &oldTermios_) != 0)
    {
        return false;
    }

    termios raw = oldTermios_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    {
        return false;
    }

    terminalRawModeEnabled_ = true;
    return true;
}

bool TempCaptureSession::pollTerminalKeyPress()
{
    if (!terminalRawModeEnabled_)
    {
        return false;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeval tv{0, 0};
    const int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &readfds))
    {
        return false;
    }

    char ch = '\0';
    const ssize_t bytes = read(STDIN_FILENO, &ch, 1);
    return bytes == 1;
}

void TempCaptureSession::saveFrameIfNeeded(const cv::Mat &frame)
{
    if (!saveEnabled_)
    {
        return;
    }

    ensureOutputDirReady();
    if (!saveEnabled_ || !saveSessionStarted_)
    {
        return;
    }

    cachedFrames_.emplace_back(frame.clone());
    const std::string filePath = outputDir_ + "/" + std::to_string(outputIndex_) + ".jpg";
    if (!imwrite(filePath, cachedFrames_.back()))
    {
        std::cerr << "保存图像失败: " << filePath << std::endl;
    }
    ++outputIndex_;
}

void TempCaptureSession::printSummary() const
{
    if (!outputDir_.empty())
    {
        cout << "保存结束，目录: " << outputDir_ << "，总帧数: " << outputIndex_
             << "，内存缓存帧数: " << cachedFrames_.size() << endl;
    }
}

void TempCaptureSession::ensureOutputDirReady()
{
    if (saveSessionStarted_)
    {
        return;
    }

    outputDir_ = "img/" + BuildTimestampString(std::chrono::system_clock::now());
    std::error_code ec;
    fs::create_directories(outputDir_, ec);
    if (ec)
    {
        std::cerr << "创建输出目录失败: " << outputDir_ << " error=" << ec.message() << std::endl;
        saveEnabled_ = false;
        return;
    }

    saveSessionStarted_ = true;
    cout << "输出目录: " << outputDir_ << endl;
}
