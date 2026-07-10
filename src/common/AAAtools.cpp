#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <termios.h>
#include <sys/select.h>

#include "common/AAAtools.h"
#include "common/main.hpp"

using namespace std;
namespace fs = std::filesystem;



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
