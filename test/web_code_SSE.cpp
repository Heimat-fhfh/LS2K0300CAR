#include <httplib.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <random>
#include <csignal>

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

// 处理普通HTTP请求的示例
void handle_root(const Request& req, Response& res) {
    string html = R"raw(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>SSE Demo</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        #data { 
            border: 1px solid #ccc; 
            padding: 20px; 
            margin: 20px 0; 
            min-height: 200px;
            background-color: #f9f9f9;
        }
        button { 
            padding: 10px 20px; 
            font-size: 16px; 
            margin: 5px; 
        }
        .event { color: blue; font-weight: bold; }
        .data { color: green; }
        .error { color: red; }
    </style>
</head>
<body>
    <h1>Server-Sent Events (SSE) 演示</h1>
    
    <div>
        <button onclick="startSSE()">开始接收数据</button>
        <button onclick="stopSSE()">停止接收</button>
    </div>
    
    <div id="data">等待数据...</div>
    
    <script>
        let eventSource = null;
        
        function startSSE() {
            if (eventSource) {
                return;
            }
            
            document.getElementById('data').innerHTML = '正在连接...';
            
            // 创建EventSource连接
            eventSource = new EventSource('/events');
            
            // 监听默认消息事件
            eventSource.onmessage = function(event) {
                displayData('message', event.data);
            };
            
            // 监听自定义事件
            eventSource.addEventListener('sensor-data', function(event) {
                displayData('sensor-data', event.data);
            });
            
            eventSource.addEventListener('end', function(event) {
                displayData('end', event.data);
                eventSource.close();
                eventSource = null;
            });
            
            // 错误处理
            eventSource.onerror = function(event) {
                displayData('error', '连接错误或已关闭');
                if (eventSource) {
                    eventSource.close();
                    eventSource = null;
                }
            };
        }
        
        function stopSSE() {
            if (eventSource) {
                eventSource.close();
                eventSource = null;
                displayData('info', '已停止接收数据');
            }
        }
        
        function displayData(type, data) {
            const div = document.getElementById('data');
            const time = new Date().toLocaleTimeString();
            
            let parsedData = data;
            try {
                parsedData = JSON.parse(data);
                parsedData = JSON.stringify(parsedData, null, 2);
            } catch (e) {
                // 如果不是JSON，保持原样
            }
            
            div.innerHTML = `
                <div>
                    <span class="event">[${time}] ${type}:</span>
                    <pre class="data">${parsedData}</pre>
                </div>
            ` + div.innerHTML;
        }
    </script>
</body>
</html>
)raw";
    
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
