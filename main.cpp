#include <httplib.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <random>
#include <csignal>
#include <fstream>
#include <streambuf>

using namespace httplib;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::stringstream;
using std::thread;
using std::this_thread::sleep_for;
using std::chrono::seconds;
using std::atomic;
using std::random_device;
using std::mt19937;
using std::uniform_int_distribution;
using std::time;
using std::signal;

// 原子变量用于控制服务器运行状态
atomic<bool> running(false);

// 全局服务器指针用于信号处理
Server* g_server = nullptr;

// 信号处理函数
void signal_handler(int signal) {
    cout << "\n收到终止信号 " << signal << ", 正在关闭服务器..." << endl;
    running = false;
    if (g_server) {
        g_server->stop();
    }
}

// SSE流处理函数
void handle_sse_stream(const Request& req, Response& res) {
    // 设置SSE相关的HTTP头
    res.set_header("Content-Type", "text/event-stream");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Connection", "keep-alive");
    res.set_header("Access-Control-Allow-Origin", "*");
    
    // 设置块传输编码（支持持续流式响应）
    res.set_chunked_content_provider("text/event-stream", 
        [&](size_t offset, DataSink& sink) {
            // 初始化随机数生成器
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> temp_dist(20, 35);
            uniform_int_distribution<> humid_dist(30, 80);
            
            int count = 0;
            while (running && count < 100) { // 限制发送100条消息，避免无限循环
                count++;
                
                // 生成随机数据
                int temperature = temp_dist(gen);
                int humidity = humid_dist(gen);
                
                // 构建SSE格式的消息
                stringstream ss;
                
                // 事件类型（可选）
                ss << "event: sensor-data\n";
                
                // 数据行
                ss << "data: {\"temperature\": " << temperature 
                   << ", \"humidity\": " << humidity 
                   << ", \"timestamp\": " << time(nullptr) 
                   << ", \"message\": \"Data #" << count << "\"}\n";
                
                // 注释行（可选，可用于保持连接）
                ss << ": heartbeat\n";
                
                // 空行表示消息结束
                ss << "\n";
                
                string message = ss.str();
                
                // 发送数据块
                if (!sink.write(message.c_str(), message.size())) {
                    cout << "客户端断开连接" << endl;
                    break;
                }
                
                // 移除不必要的flush调用
                
                // 等待1秒
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // 发送结束消息
            string end_msg = "event: end\ndata: {\"status\": \"finished\"}\n\n";
            sink.write(end_msg.c_str(), end_msg.size());
            sink.done();
            
            return true;
        }
    );
}

// 读取文件内容的辅助函数
string read_file(const string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return "";
    }
    std::string content((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
    return content;
}

// 处理根路径请求，返回静态HTML文件
void handle_root(const Request& req, Response& res) {
    string html = read_file("web/index.html");
    if (html.empty()) {
        res.status = 404;
        res.set_content("File not found", "text/plain");
        return;
    }
    res.set_content(html, "text/html");
}

int main() {
    Server svr;
    
    // 设置全局服务器指针用于信号处理
    g_server = &svr;
    
    // 注册信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 设置日志处理
    svr.set_logger([](const Request& req, const Response& res) {
        cout << req.method << " " << req.path << " -> " << res.status << endl;
    });
    
    // 注册路由
    svr.Get("/", handle_root);
    svr.Get("/events", handle_sse_stream);
    
    // 静态文件服务 - 提供CSS文件
    svr.Get("/css/(.*)", [&](const Request& req, Response& res) {
        string filename = "web/css/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        res.set_content(content, "text/css");
    });
    
    // 静态文件服务 - 提供JS文件
    svr.Get("/js/(.*)", [&](const Request& req, Response& res) {
        string filename = "web/js/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        res.set_content(content, "application/javascript");
    });
    
    // 静态文件服务 - 提供图片文件（如果需要）
    svr.Get("/image/(.*)", [&](const Request& req, Response& res) {
        string filename = "web/image/" + req.matches[1].str();
        string content = read_file(filename);
        if (content.empty()) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }
        // 根据文件扩展名设置正确的MIME类型
        if (filename.find(".png") != string::npos) {
            res.set_content(content, "image/png");
        } else if (filename.find(".jpg") != string::npos || filename.find(".jpeg") != string::npos) {
            res.set_content(content, "image/jpeg");
        } else if (filename.find(".gif") != string::npos) {
            res.set_content(content, "image/gif");
        } else if (filename.find(".svg") != string::npos) {
            res.set_content(content, "image/svg+xml");
        } else {
            res.set_content(content, "application/octet-stream");
        }
    });
    
    // 停止服务器的接口
    svr.Get("/stop", [&](const Request& req, Response& res) {
        running = false;
        res.set_content("服务器正在停止...", "text/plain");
    });
    
    // 健康检查接口
    svr.Get("/health", [](const Request& req, Response& res) {
        res.set_content("{\"status\": \"ok\"}", "application/json");
    });
    
    cout << "服务器启动在 http://0.0.0.0:8080" << endl;
    cout << "访问 http://0.0.0.0:8080 查看SSE演示" << endl;
    cout << "SSE流地址: http://0.0.0.0:8080/events" << endl;
    cout << "按Ctrl+C停止服务器" << endl;
    
    running = true;
    
    // 启动服务器
    if (!svr.listen("0.0.0.0", 8080)) {
        cerr << "启动服务器失败" << endl;
        return 1;
    }
    
    return 0;
}
