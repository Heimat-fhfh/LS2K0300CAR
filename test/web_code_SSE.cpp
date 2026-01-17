#include "web_server.h"

// 测试专用的handle_root函数，使用内嵌HTML
void test_handle_root(const httplib::Request& req, httplib::Response& res) {
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

// 测试专用的web服务器启动函数
int test_start_web_server() {
    httplib::Server svr;
    
    // 设置全局服务器指针用于信号处理
    g_server = &svr;
    
    // 注册信号处理器
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 设置日志处理
    svr.set_logger([](const httplib::Request& req, const httplib::Response& res) {
        cout << req.method << " " << req.path << " -> " << res.status << endl;
    });
    
    // 注册路由
    svr.Get("/", test_handle_root);
    svr.Get("/events", handle_sse_stream);
    
    // 停止服务器的接口
    svr.Get("/stop", [&](const httplib::Request& req, httplib::Response& res) {
        running = false;
        res.set_content("服务器正在停止...", "text/plain");
    });
    
    // 健康检查接口
    svr.Get("/health", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("{\"status\": \"ok\"}", "application/json");
    });
    
    cout << "测试服务器启动在 http://0.0.0.0:8080" << endl;
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

int main() {
    return test_start_web_server();
}
