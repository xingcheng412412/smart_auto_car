#!/usr/bin/env python3
"""RTKGPSLog 实时数据查看服务器 (ROS 2)

订阅 ROS 2 topic /rtk_log，通过 WebSocket 实时推送到网页。
局域网其他电脑可通过浏览器访问 http://<本机IP>:8080 查看实时数据。
"""

import base64
import hashlib
import http.server
import json
import os
import socket
import socketserver
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rtk_gps_driver_cpp.msg import RTKGPSLog

PORT = 8080
TOPIC = "/rtk_log"

# 全局存储最新消息
latest_data = None
data_lock = threading.Lock()
clients = []


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def msg_to_dict(msg):
    """将 RTKGPSLog 消息转为字典"""
    return {
        "header": {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
        },
        "log_level": msg.log_level,
        "connection_phase": msg.connection_phase,
        "sock_connected": msg.sock_connected,
        "network_status": msg.network_status,
        "ntrip_consecutive_failure_count": msg.ntrip_consecutive_failure_count,
        "ntrip_next_reconnect_delay": msg.ntrip_next_reconnect_delay,
        "ntrip_failure_category": msg.ntrip_failure_category,
        "ntrip_sdk_status_code": msg.ntrip_sdk_status_code,
        "fix_quality": msg.fix_quality,
        "num_satellites": msg.num_satellites,
        "hdop": msg.hdop,
        "latitude": msg.latitude,
        "longitude": msg.longitude,
        "altitude": msg.altitude,
        "chc_status": msg.chc_status,
        "differential_age": msg.differential_age,
        "warning_flags": msg.warning_flags,
        "rtcm_receive_freq": msg.rtcm_receive_freq,
        "time_since_last_rtcm": msg.time_since_last_rtcm,
        "network_latency": msg.network_latency,
        "connection_duration": msg.connection_duration,
        "non_rtk_fixed_count": msg.non_rtk_fixed_count,
        "non_rtk_fixed_duration": msg.non_rtk_fixed_duration,
        "gga_send_time_ms": msg.gga_send_time_ms,
        "error_type": msg.error_type,
        "error_description": msg.error_description,
    }


# ---------- 简易 WebSocket 处理 ----------
WebSocketGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def compute_accept(key):
    accept = key + WebSocketGuid
    h = hashlib.sha1(accept.encode()).digest()
    return base64.b64encode(h).decode()


def ws_send_frame(conn, message):
    """发送文本帧"""
    payload = message.encode("utf-8")
    header = bytearray()
    header.append(0x81)  # FIN + text frame
    mask = False
    length = len(payload)
    if length < 126:
        header.append((0x80 if mask else 0x00) | length)
    elif length < 65536:
        header.append((0x80 if mask else 0x00) | 126)
        header.extend(struct.pack(">H", length))
    else:
        header.append((0x80 if mask else 0x00) | 127)
        header.extend(struct.pack(">Q", length))
    conn.sendall(header + payload)


def ws_recv_frame(conn):
    """接收一帧，返回文本内容或 None"""
    header = conn.recv(2)
    if len(header) < 2:
        return None
    opcode = header[0] & 0x0F
    masked = (header[1] & 0x80) != 0
    length = header[1] & 0x7F
    if length == 126:
        data = conn.recv(2)
        length = struct.unpack(">H", data)[0]
    elif length == 127:
        data = conn.recv(8)
        length = struct.unpack(">Q", data)[0]
    if masked:
        mask = conn.recv(4)
    payload = b""
    while len(payload) < length:
        chunk = conn.recv(length - len(payload))
        if not chunk:
            return None
        payload += chunk
    if masked:
        payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
    if opcode == 0x8:  # close
        return None
    return payload.decode("utf-8", errors="replace")


class WSClient:
    """WebSocket 客户端封装"""

    def __init__(self, conn, addr):
        self.conn = conn
        self.addr = addr

    def send(self, message):
        ws_send_frame(self.conn, message)

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass


def handle_websocket(conn, addr):
    """处理 WebSocket 连接"""
    # 发送当前最新数据
    with data_lock:
        if latest_data:
            ws_send_frame(conn, json.dumps({"type": "data", "data": latest_data}))

    ws_client = WSClient(conn, addr)
    clients.append(ws_client)

    try:
        while True:
            msg = ws_recv_frame(conn)
            if msg is None:
                break
            # 忽略客户端消息（仅推送）
    except Exception:
        pass
    finally:
        if ws_client in clients:
            clients.remove(ws_client)
        conn.close()


# ---------- HTTP 处理 ----------
HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RTK GPS 实时监控</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
    font-family: -apple-system, "Segoe UI", "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
    background: #0f0f1a;
    color: #e0e0e0;
    padding: 16px;
}
.header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 16px;
    flex-wrap: wrap;
    gap: 12px;
}
.header h1 { font-size: 20px; color: #89b4fa; }
.status-bar {
    display: flex;
    align-items: center;
    gap: 12px;
    font-size: 13px;
}
.conn-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: #f38ba8;
    transition: background 0.3s;
}
.conn-dot.connected { background: #a6e3a1; }
.conn-text { color: #6c7086; }
.timestamp { color: #6c7086; font-family: monospace; }
.grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(340px, 1fr));
    gap: 12px;
}
.card {
    background: #1a1a2e;
    border: 1px solid #2a2a4a;
    border-radius: 10px;
    padding: 14px 16px;
}
.card-title {
    font-size: 12px;
    color: #6c7086;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin-bottom: 10px;
    border-bottom: 1px solid #2a2a4a;
    padding-bottom: 6px;
}
.field {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 4px 0;
    font-size: 14px;
}
.field-label { color: #9399b2; }
.field-value {
    color: #cdd6f4;
    font-family: "JetBrains Mono", "Fira Code", Consolas, monospace;
    font-weight: 500;
    text-align: right;
}
.badge {
    display: inline-block;
    padding: 2px 10px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: bold;
}
.badge-green { background: #1e3a1e; color: #a6e3a1; }
.badge-red { background: #3a1e1e; color: #f38ba8; }
.badge-yellow { background: #3a3a1e; color: #f9e2af; }
.badge-blue { background: #1e2a3a; color: #89b4fa; }
.badge-gray { background: #2a2a3a; color: #6c7086; }
.warn-list {
    margin-top: 6px;
    display: flex;
    flex-direction: column;
    gap: 4px;
}
.warn-item {
    font-size: 12px;
    color: #f38ba8;
    font-family: monospace;
}
</style>
</head>
<body>
<div class="header">
    <h1>RTK GPS 实时监控 - RTKGPSLog</h1>
    <div class="status-bar">
        <div class="conn-dot" id="connDot"></div>
        <span class="conn-text" id="connText">等待连接...</span>
        <span class="timestamp" id="timestamp"></span>
    </div>
</div>
<div class="grid" id="grid"></div>

<script>
const fixQualityMap = {
    0: { text: "无定位", cls: "badge-red" },
    1: { text: "GPS", cls: "badge-yellow" },
    2: { text: "DGPS", cls: "badge-yellow" },
    4: { text: "RTK固定", cls: "badge-green" },
    5: { text: "RTK浮点", cls: "badge-yellow" },
    6: { text: "Status80", cls: "badge-blue" },
};
const phaseMap = {
    "RUNNING": "badge-green",
    "DISCONNECTED": "badge-red",
    "CONNECTING": "badge-yellow",
    "AUTH_FAILED": "badge-red",
};
const netMap = {
    "NORMAL": "badge-green",
    "DELAY_WARNING": "badge-yellow",
    "ABNORMAL": "badge-red",
    "NOT_CONNECTED": "badge-red",
};
const levelMap = {
    "INFO": "badge-green",
    "WARN": "badge-yellow",
    "ERROR": "badge-red",
};

// warning_flags 解析
const warnBits = {
    0x0001: "卫星数不足",
    0x0002: "差分数据龄期超限",
    0x0004: "RTK未固定",
    0x0010: "天线异常",
    0x0020: "加速度超限",
    0x0100: "高程异常",
    0x0200: "位置异常",
};

function parseWarningFlags(flags) {
    const items = [];
    for (const [bit, desc] of Object.entries(warnBits)) {
        if (flags & parseInt(bit)) items.push(desc);
    }
    return items;
}

function badge(text, cls) {
    return `<span class="badge ${cls}">${text}</span>`;
}

function fmt(val, digits) {
    if (val === null || val === undefined) return "-";
    if (typeof val === "number") return val.toFixed(digits);
    return val;
}

function buildGrid(d) {
    const grid = document.getElementById("grid");
    const warnItems = parseWarningFlags(d.warning_flags);

    grid.innerHTML = `
    <div class="card">
        <div class="card-title">连接状态</div>
        <div class="field"><span class="field-label">日志级别</span>${badge(d.log_level || "-", levelMap[d.log_level] || "badge-gray")}</div>
        <div class="field"><span class="field-label">连接阶段</span>${badge(d.connection_phase || "-", phaseMap[d.connection_phase] || "badge-gray")}</div>
        <div class="field"><span class="field-label">Socket连接</span>${d.sock_connected ? badge("已连接", "badge-green") : badge("未连接", "badge-red")}</div>
        <div class="field"><span class="field-label">网络状态</span>${badge(d.network_status || "-", netMap[d.network_status] || "badge-gray")}</div>
        <div class="field"><span class="field-label">失败次数</span><span class="field-value">${d.ntrip_consecutive_failure_count}</span></div>
        <div class="field"><span class="field-label">重连等待(s)</span><span class="field-value">${fmt(d.ntrip_next_reconnect_delay, 1)}</span></div>
        <div class="field"><span class="field-label">失败类别</span><span class="field-value">${d.ntrip_failure_category || "-"}</span></div>
    </div>

    <div class="card">
        <div class="card-title">GPS定位质量</div>
        <div class="field"><span class="field-label">定位类型</span>${fixQualityMap[d.fix_quality] ? badge(fixQualityMap[d.fix_quality].text, fixQualityMap[d.fix_quality].cls) : badge("未知:"+d.fix_quality, "badge-gray")}</div>
        <div class="field"><span class="field-label">卫星数</span><span class="field-value">${d.num_satellites}</span></div>
        <div class="field"><span class="field-label">HDOP</span><span class="field-value">${fmt(d.hdop, 2)}</span></div>
        <div class="field"><span class="field-label">纬度(°)</span><span class="field-value">${fmt(d.latitude, 8)}</span></div>
        <div class="field"><span class="field-label">经度(°)</span><span class="field-value">${fmt(d.longitude, 8)}</span></div>
        <div class="field"><span class="field-label">高度(m)</span><span class="field-value">${fmt(d.altitude, 2)}</span></div>
    </div>

    <div class="card">
        <div class="card-title">GPCHCX 组合导航</div>
        <div class="field"><span class="field-label">CHC状态(0x)</span><span class="field-value">${d.chc_status.toString(16).toUpperCase().padStart(2,'0')}</span></div>
        <div class="field"><span class="field-label">差分延时(s)</span><span class="field-value">${fmt(d.differential_age, 1)}</span></div>
        <div class="field"><span class="field-label">告警标志(0x)</span><span class="field-value">${d.warning_flags.toString(16).toUpperCase().padStart(4,'0')}</span></div>
        ${warnItems.length ? `<div class="warn-list">${warnItems.map(w=>`<div class="warn-item">⚠ ${w}</div>`).join("")}</div>` : ""}
    </div>

    <div class="card">
        <div class="card-title">RTCM 数据统计</div>
        <div class="field"><span class="field-label">RTCM频率(包/s)</span><span class="field-value">${fmt(d.rtcm_receive_freq, 1)}</span></div>
        <div class="field"><span class="field-label">距上次RTCM(s)</span><span class="field-value">${fmt(d.time_since_last_rtcm, 1)}</span></div>
        <div class="field"><span class="field-label">网络延迟(ms)</span><span class="field-value">${fmt(d.network_latency, 0)}</span></div>
        <div class="field"><span class="field-label">连接持续时间(s)</span><span class="field-value">${fmt(d.connection_duration, 0)}</span></div>
    </div>

    <div class="card">
        <div class="card-title">RTK 固定解统计</div>
        <div class="field"><span class="field-label">非固定解次数</span><span class="field-value">${d.non_rtk_fixed_count}</span></div>
        <div class="field"><span class="field-label">非固定解时长(s)</span><span class="field-value">${fmt(d.non_rtk_fixed_duration, 1)}</span></div>
        <div class="field"><span class="field-label">GGA发送耗时(ms)</span><span class="field-value">${fmt(d.gga_send_time_ms, 2)}</span></div>
    </div>

    <div class="card">
        <div class="card-title">错误信息</div>
        <div class="field"><span class="field-label">错误类型</span><span class="field-value">${d.error_type || "-"}</span></div>
        <div class="field" style="margin-top:4px;"><span class="field-label">错误描述</span></div>
        <div style="color:#f38ba8; font-size:13px; margin-top:2px; word-break:break-all;">${d.error_description || "-"}</div>
    </div>
    `;
}

function updateTimestamp(stamp) {
    const d = new Date(stamp * 1000);
    const pad = n => String(n).padStart(2, "0");
    document.getElementById("timestamp").textContent =
        `${d.getFullYear()}-${pad(d.getMonth()+1)}-${pad(d.getDate())} ${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

// WebSocket 连接
function connectWS() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(`${proto}//${location.host}/ws`);
    const dot = document.getElementById("connDot");
    const text = document.getElementById("connText");

    ws.onopen = () => {
        dot.classList.add("connected");
        text.textContent = "已连接";
    };
    ws.onmessage = (ev) => {
        const msg = JSON.parse(ev.data);
        if (msg.type === "data" && msg.data) {
            buildGrid(msg.data);
            updateTimestamp(msg.data.header.stamp);
        }
    };
    ws.onclose = () => {
        dot.classList.remove("connected");
        text.textContent = "连接断开，5秒后重连...";
        setTimeout(connectWS, 5000);
    };
    ws.onerror = () => { ws.close(); };
}
connectWS();
</script>
</body>
</html>"""


class HTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/ws":
            self.handle_ws()
        else:
            self.handle_page()

    def handle_page(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode("utf-8"))

    def handle_ws(self):
        # WebSocket 握手
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self.send_error(400, "Missing WebSocket Key")
            return
        accept = compute_accept(key)
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        # 获取底层 socket
        conn = self.connection
        addr = self.client_address
        handle_websocket(conn, addr)

    def log_message(self, format, *args):
        pass


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class RTKGPSLogViewerNode(Node):
    """ROS 2 节点：订阅 /rtk_log 并通过 WebSocket 推送给网页。"""

    def __init__(self):
        super().__init__("rtkgpslog_web_viewer")

        # 声明参数
        self.declare_parameter("port", PORT)
        self.declare_parameter("topic", TOPIC)

        port = self.get_parameter("port").value
        topic = self.get_parameter("topic").value

        # 订阅 RTKGPSLog topic，使用可靠 QoS 保证不丢日志
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            RTKGPSLog, topic, self._topic_callback, qos
        )

        # 启动 HTTP/WebSocket 服务器线程
        local_ip = get_local_ip()
        self._server = ThreadingHTTPServer(("0.0.0.0", port), HTTPHandler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._server_thread.start()

        self.get_logger().info("=" * 55)
        self.get_logger().info("  RTKGPSLog 实时数据查看服务器")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"  ROS 2 Topic: {topic}")
        self.get_logger().info(f"  局域网访问:  http://{local_ip}:{port}")
        self.get_logger().info(f"  本机访问:    http://127.0.0.1:{port}")
        self.get_logger().info("=" * 55)

    def _topic_callback(self, msg):
        """RTKGPSLog 消息回调：更新最新数据并广播给 WebSocket 客户端。"""
        global latest_data
        data = msg_to_dict(msg)
        with data_lock:
            latest_data = data
        # 广播给所有 WebSocket 客户端
        msg_json = json.dumps({"type": "data", "data": data})
        for ws_client in list(clients):
            try:
                ws_client.send(msg_json)
            except Exception:
                if ws_client in clients:
                    clients.remove(ws_client)

    def destroy_node(self):
        if hasattr(self, "_server"):
            self._server.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTKGPSLogViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
