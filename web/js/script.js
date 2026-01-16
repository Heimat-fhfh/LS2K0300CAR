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
