# K230D MJPEG browser streaming with computer-side recording.
#
# Run this script in CanMV IDE. After Wi-Fi is connected, open the URL printed
# in the serial console with a recent Chrome or Edge browser.
#
# 上游来源：
# https://gitee.com/linsen06/2026diansaikaiyuanqingdane/tree/master/H题/图传/k230/k230d版源码
# 本测试版本仅修改热点参数和日志名称，用于独立图传K230，不进入车辆控制链路。

import gc
import network
import os
import socket
import sys
import time

from media.sensor import Sensor

try:
    # Added after the v1.8 release. Prefer it when the firmware provides it.
    from media.mjpeg import MJPEGEncoder as NativeMJPEGEncoder
    HAS_NATIVE_MJPEG = True
except ImportError:
    # CanMV v1.8 still provides hardware JPEG through the lower-level VENC API.
    import media.vencoder as legacy_vencoder
    import uctypes
    HAS_NATIVE_MJPEG = False


# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------

WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD"

WIFI_DEVICE = "w0"

SERVER_PORT = 8080
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_ALIGNMENT = 12
JPEG_QUALITY = 50
STREAM_FPS = 25
RECORDING_BITRATE = 3_000_000
VENC_OUT_BUFFER_COUNT = 8
ENCODE_TIMEOUT_MS = 1000

NETWORK_TIMEOUT_S = 30
REQUEST_TIMEOUT_MS = 2000
MAX_REQUEST_BYTES = 4096
SEND_STALL_TIMEOUT_MS = 5000
SEND_CHUNK_BYTES = 16384

# Video-frame input avoids copying each camera frame through an image buffer.
USE_VIDEO_FRAME = True


class HardwareJpegEncoder:
    """Use the best hardware JPEG API available in the running firmware."""

    def __init__(self, width, height, quality, fps):
        self.native = None
        self.legacy = None
        self.legacy_channel = None
        self.legacy_created = False
        self.legacy_started = False
        self.stream_data = None

        if HAS_NATIVE_MJPEG:
            self.backend_name = "media.mjpeg.MJPEGEncoder"
            self.native = NativeMJPEGEncoder(quality=quality)
            return

        self.backend_name = "media.vencoder JPEG (CanMV 1.6 compatibility)"
        self.legacy_channel = legacy_vencoder.VENC_CHN_ID_0
        self.legacy = legacy_vencoder.Encoder()
        try:
            self.legacy.SetOutBufs(
                self.legacy_channel,
                VENC_OUT_BUFFER_COUNT,
                width,
                height,
            )

            channel_attr = legacy_vencoder.ChnAttrStr(
                self.legacy.PAYLOAD_TYPE_JPEG,
                0,
                width,
                height,
                src_frame_rate=fps,
                dst_frame_rate=fps,
                mjpeg_quality_factor=quality,
            )
            self.legacy.Create(self.legacy_channel, channel_attr)
            self.legacy_created = True
            self.legacy.Start(self.legacy_channel)
            self.legacy_started = True
            self.stream_data = legacy_vencoder.StreamData()
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            raise

    def encode(self, sensor):
        frame = sensor.snapshot(dump_frame=USE_VIDEO_FRAME)

        if self.native is not None:
            try:
                return self.native.encode(
                    frame,
                    timeout_ms=ENCODE_TIMEOUT_MS,
                )
            finally:
                del frame

        try:
            result = self.legacy.SendFrame(
                self.legacy_channel,
                frame,
                timeout=ENCODE_TIMEOUT_MS,
            )
        except BaseException:
            del frame
            raise

        if result != 0:
            del frame
            raise OSError("VENC failed to accept a camera frame: %d" % result)

        stream_acquired = False
        try:
            result = self.legacy.GetStream(
                self.legacy_channel,
                self.stream_data,
                timeout=ENCODE_TIMEOUT_MS,
            )
            if result != 0:
                raise OSError("Timed out while waiting for a JPEG frame")
            stream_acquired = True

            parts = []
            for pack_index in range(self.stream_data.pack_cnt):
                part = uctypes.bytearray_at(
                    self.stream_data.data[pack_index],
                    self.stream_data.data_size[pack_index],
                )
                parts.append(bytes(part))

            if len(parts) == 1:
                return parts[0]
            return b"".join(parts)
        finally:
            try:
                if stream_acquired:
                    self.legacy.ReleaseStream(
                        self.legacy_channel,
                        self.stream_data,
                    )
            finally:
                del frame

    def close(self):
        if self.native is not None:
            self.native.close()
            self.native = None
            return

        if self.legacy is None:
            return

        stop_error = None
        if self.legacy_started:
            try:
                self.legacy.Stop(self.legacy_channel)
            except BaseException as error:
                stop_error = error
            self.legacy_started = False

        if self.legacy_created:
            try:
                self.legacy.Destroy(self.legacy_channel)
            except BaseException as error:
                if stop_error is None:
                    stop_error = error
            self.legacy_created = False
        elif getattr(self.legacy, "private_poolid", -1) != -1:
            # v1.8 SetOutBufs creates a private VB pool before Create().
            try:
                legacy_vencoder.kd_mpi_vb_destory_pool(
                    self.legacy.private_poolid
                )
                self.legacy.private_poolid = -1
            except BaseException as error:
                if stop_error is None:
                    stop_error = error

        self.legacy = None
        self.stream_data = None
        if stop_error is not None:
            raise stop_error


INDEX_HTML = (
    """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>创源启明 K230D 图传</title>
<style>
*{box-sizing:border-box}
html,body{width:100%;min-height:100%;margin:0;background:#303030;color:#fff;
font-family:Arial,"Microsoft YaHei",sans-serif}
body{display:flex;justify-content:center;padding:24px}
.container{width:100%;max-width:1360px;text-align:center}
.title{margin-bottom:22px;padding:15px 20px;border-radius:16px;
background:rgba(255,255,255,.09);font-size:clamp(22px,3vw,40px);
font-weight:700;box-shadow:0 8px 28px rgba(0,0,0,.14)}
.controls{display:flex;flex-wrap:wrap;justify-content:center;align-items:center;
gap:12px;margin-bottom:18px}
button{height:42px;border:1px solid rgba(255,255,255,.18);border-radius:10px;
padding:0 18px;color:#fff;background:#4a4a4a;font-size:16px;cursor:pointer}
button:hover:not(:disabled){background:#5b5b5b}
button:disabled{cursor:not-allowed;opacity:.5}
#record-button{min-width:138px;background:#c63737}
#record-button:hover:not(:disabled){background:#dc4444}
#record-button.recording{background:#8d2020;
box-shadow:0 0 0 4px rgba(220,68,68,.18)}
#record-timer{min-width:72px;font-family:Consolas,monospace;font-size:18px}
#record-message{flex-basis:100%;min-height:20px;color:#d7d7d7;font-size:14px}
.video-wrap{position:relative;overflow:hidden;width:100%;aspect-ratio:16/9;
border-radius:20px;background:#111;box-shadow:0 18px 45px rgba(0,0,0,.42)}
canvas{position:relative;z-index:1;display:block;width:100%;height:100%;
object-fit:contain;background:#111}
.badge{position:absolute;bottom:16px;padding:8px 13px;border-radius:11px;
background:rgba(0,0,0,.64);font-size:15px;backdrop-filter:blur(5px)}
#resolution-badge{left:16px}
#status-badge{right:16px;display:flex;align-items:center;gap:8px}
.dot{width:9px;height:9px;border-radius:50%;background:#e34949}
.dot.live{background:#33c46a;box-shadow:0 0 0 4px rgba(51,196,106,.16)}
#fullscreen-button{position:fixed;right:26px;bottom:26px;display:none;width:56px;
height:56px;padding:0;border:0;border-radius:50%;background:rgba(255,255,255,.2);
font-size:24px;backdrop-filter:blur(8px)}
#mjpeg-source{position:absolute;z-index:0;inset:0;display:block;width:100%;height:100%;
object-fit:contain;pointer-events:none}
@media(max-width:640px){
body{padding:10px}.title{margin-bottom:14px}.controls{gap:8px}
button{height:38px;font-size:14px}.badge{bottom:9px;padding:6px 9px;font-size:12px}
#resolution-badge{left:9px}#status-badge{right:9px}
#fullscreen-button{right:16px;bottom:16px}
}
</style>
</head>
<body>
<main class="container">
<div class="title">创源启明 K230D 图传</div>
<div class="controls">
<button id="record-button" disabled>● 开始录像</button>
<span id="record-timer">00:00</span>
<div id="record-message">正在等待视频连接</div>
</div>
<div class="video-wrap" id="video-wrap">
<img id="mjpeg-source" alt="">
<canvas id="video-canvas" width="__FRAME_WIDTH__" height="__FRAME_HEIGHT__"></canvas>
<div class="badge" id="resolution-badge">__FRAME_WIDTH__ × __FRAME_HEIGHT__ · MJPEG</div>
<div class="badge" id="status-badge">
<span class="dot" id="status-dot"></span><span id="status-text">正在连接</span>
</div>
</div>
</main>
<button id="fullscreen-button" title="全屏">⛶</button>
<script>
"use strict";

const FRAME_WIDTH = __FRAME_WIDTH__;
const FRAME_HEIGHT = __FRAME_HEIGHT__;
const STREAM_FPS = __STREAM_FPS__;
const RECORDING_BITRATE = __RECORDING_BITRATE__;

const source = document.getElementById("mjpeg-source");
const canvas = document.getElementById("video-canvas");
const context = canvas.getContext("2d", {alpha: false});
const videoWrap = document.getElementById("video-wrap");
const recordButton = document.getElementById("record-button");
const recordTimer = document.getElementById("record-timer");
const recordMessage = document.getElementById("record-message");
const statusText = document.getElementById("status-text");
const statusDot = document.getElementById("status-dot");
const fullscreenButton = document.getElementById("fullscreen-button");

let streamReady = false;
let reconnectTimer = null;
let mediaRecorder = null;
let recordingStream = null;
let recordedChunks = [];
let recordingStartedAt = 0;
let recordingTimer = null;
let recordingStopReason = "";

context.fillStyle = "#111";
context.fillRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);

function setStatus(text, live) {
    statusText.textContent = text;
    statusDot.classList.toggle("live", Boolean(live));
}

function elapsedText(milliseconds) {
    const total = Math.floor(milliseconds / 1000);
    const hours = Math.floor(total / 3600);
    const minutes = Math.floor((total % 3600) / 60);
    const seconds = total % 60;
    const pad = (value) => String(value).padStart(2, "0");
    return hours > 0
        ? `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`
        : `${pad(minutes)}:${pad(seconds)}`;
}

function timestampName() {
    const now = new Date();
    const pad = (value) => String(value).padStart(2, "0");
    return `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}_` +
           `${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
}

function recordingSupported() {
    return "MediaRecorder" in window &&
           typeof canvas.captureStream === "function";
}

function restoreRecordButton() {
    recordButton.disabled = !streamReady || !recordingSupported() ||
                            mediaRecorder !== null;
    recordButton.classList.remove("recording");
    recordButton.textContent = "● 开始录像";
}

function markStreamReady() {
    if (streamReady) {
        return;
    }
    streamReady = true;
    setStatus("已连接", true);
    fullscreenButton.style.display = "block";
    if (recordingSupported()) {
        recordMessage.textContent = "可开始电脑端录像";
    } else {
        recordMessage.textContent = "当前浏览器不支持画布录像，请使用最新版 Chrome 或 Edge";
    }
    restoreRecordButton();
}

function scheduleReconnect() {
    if (reconnectTimer !== null) {
        return;
    }
    reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectStream();
    }, 1000);
}

function streamFailed() {
    streamReady = false;
    setStatus("连接断开", false);
    fullscreenButton.style.display = "none";
    recordButton.disabled = true;
    if (mediaRecorder && mediaRecorder.state === "recording") {
        recordingStopReason = "图传中断，已保存当前录像";
        stopRecording();
    } else if (mediaRecorder === null) {
        recordMessage.textContent = "正在尝试重新连接";
    }
    source.removeAttribute("src");
    scheduleReconnect();
}

function connectStream() {
    streamReady = false;
    recordButton.disabled = true;
    setStatus("正在连接", false);
    source.src = `/stream?_=${Date.now()}`;
}

source.onerror = streamFailed;
source.onload = markStreamReady;

// Draw the current MJPEG frame into a canvas. The canvas is both the visible
// preview and the source used by MediaRecorder.
setInterval(() => {
    if (source.naturalWidth <= 0 || source.naturalHeight <= 0) {
        return;
    }
    try {
        context.drawImage(source, 0, 0, FRAME_WIDTH, FRAME_HEIGHT);
        markStreamReady();
    } catch (error) {
        console.warn("Frame draw failed", error);
    }
}, Math.max(10, Math.floor(1000 / STREAM_FPS)));

function createRecorder(stream) {
    const candidates = [
        "video/mp4;codecs=avc1.42E01E",
        "video/webm;codecs=vp9",
        "video/webm;codecs=vp8",
        "video/webm"
    ];

    for (const mimeType of candidates) {
        if (MediaRecorder.isTypeSupported &&
            !MediaRecorder.isTypeSupported(mimeType)) {
            continue;
        }
        try {
            return new MediaRecorder(stream, {
                mimeType,
                videoBitsPerSecond: RECORDING_BITRATE
            });
        } catch (error) {
            console.warn(`Recorder format unavailable: ${mimeType}`, error);
        }
    }

    return new MediaRecorder(stream, {
        videoBitsPerSecond: RECORDING_BITRATE
    });
}

function recorderExtension(mimeType) {
    return mimeType && mimeType.toLowerCase().includes("mp4") ? "mp4" : "webm";
}

function downloadRecording(blob, filename) {
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 30000);
}

function finishRecording() {
    clearInterval(recordingTimer);
    recordingTimer = null;

    const mimeType = mediaRecorder.mimeType ||
        (recordedChunks[0] && recordedChunks[0].type) ||
        "video/webm";
    const extension = recorderExtension(mimeType);
    const filename = `${timestampName()}.${extension}`;
    const blob = new Blob(recordedChunks, {type: mimeType});
    recordedChunks = [];

    if (recordingStream) {
        recordingStream.getTracks().forEach((track) => track.stop());
        recordingStream = null;
    }

    mediaRecorder = null;
    if (blob.size > 0) {
        downloadRecording(blob, filename);
        recordMessage.textContent = recordingStopReason ||
            `录像已下载到电脑：${filename}`;
    } else {
        recordMessage.textContent = "录像失败：没有产生视频数据";
    }
    recordingStopReason = "";
    restoreRecordButton();
}

function startRecording() {
    if (!streamReady || !recordingSupported() || mediaRecorder !== null) {
        recordMessage.textContent = "视频尚未连接或浏览器不支持录像";
        return;
    }

    try {
        recordingStream = canvas.captureStream(STREAM_FPS);
        mediaRecorder = createRecorder(recordingStream);
        recordedChunks = [];
        recordingStopReason = "";

        mediaRecorder.ondataavailable = (event) => {
            if (event.data && event.data.size > 0) {
                recordedChunks.push(event.data);
            }
        };
        mediaRecorder.onerror = (event) => {
            const detail = event.error && event.error.message
                ? event.error.message : "未知错误";
            recordMessage.textContent = `录像错误：${detail}`;
        };
        mediaRecorder.onstop = finishRecording;
        mediaRecorder.start(1000);

        recordingStartedAt = performance.now();
        recordTimer.textContent = "00:00";
        recordingTimer = setInterval(() => {
            recordTimer.textContent =
                elapsedText(performance.now() - recordingStartedAt);
        }, 500);
        recordButton.disabled = false;
        recordButton.classList.add("recording");
        recordButton.textContent = "■ 停止录像";
        recordMessage.textContent = "正在录像，停止后自动下载到电脑";
    } catch (error) {
        console.error(error);
        if (recordingStream) {
            recordingStream.getTracks().forEach((track) => track.stop());
            recordingStream = null;
        }
        mediaRecorder = null;
        recordMessage.textContent = `无法开始录像：${error.message}`;
        restoreRecordButton();
    }
}

function stopRecording() {
    if (mediaRecorder && mediaRecorder.state === "recording") {
        recordButton.disabled = true;
        recordButton.textContent = "正在保存...";
        mediaRecorder.stop();
    }
}

recordButton.onclick = () => {
    if (mediaRecorder && mediaRecorder.state === "recording") {
        stopRecording();
    } else {
        startRecording();
    }
};

fullscreenButton.onclick = () => {
    if (!document.fullscreenElement) {
        videoWrap.requestFullscreen?.();
    } else {
        document.exitFullscreen?.();
    }
};

window.addEventListener("beforeunload", (event) => {
    if (mediaRecorder && mediaRecorder.state === "recording") {
        event.preventDefault();
        event.returnValue = "";
    }
});

if (!recordingSupported()) {
    recordMessage.textContent = "当前浏览器不支持录像，请使用最新版 Chrome 或 Edge";
}
connectStream();
</script>
</body>
</html>
"""
    .replace("__FRAME_WIDTH__", str(FRAME_WIDTH))
    .replace("__FRAME_HEIGHT__", str(FRAME_HEIGHT))
    .replace("__STREAM_FPS__", str(STREAM_FPS))
    .replace("__RECORDING_BITRATE__", str(RECORDING_BITRATE))
    .encode("utf-8")
)


def wait_for_ip(netif, timeout_s=NETWORK_TIMEOUT_S):
    start = time.time()
    last_report = start - 1
    while time.time() - start < timeout_s:
        config = netif.ifconfig()
        if netif.isconnected() and config and config[0] != "0.0.0.0":
            return config[0]
        now = time.time()
        if now - last_report >= 1:
            last_report = now
            print(
                "Wi-Fi waiting: status=%s ip=%s" % (
                    netif.status(),
                    config[0] if config else "unknown",
                )
            )
        os.exitpoint()
        time.sleep_ms(100)
    raise RuntimeError("Wi-Fi connection timed out before obtaining an IP address")


def connect_wifi():
    if not hasattr(network, "WLAN"):
        raise RuntimeError("This firmware does not provide WLAN support")

    if hasattr(network, "get_dev_list"):
        devices = network.get_dev_list()
        if devices is not None and WIFI_DEVICE not in devices:
            raise RuntimeError(
                "Wi-Fi device '%s' is unavailable; found: %s"
                % (WIFI_DEVICE, devices)
            )

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected() and wlan.ifconfig()[0] != "0.0.0.0":
        print("Wi-Fi already connected")
    else:
        print("Connecting to Wi-Fi:", WIFI_SSID)
        # CanMV 1.6官方例程不判断connect()返回值，而是轮询连接状态。
        connect_result = wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        print("Wi-Fi connect request result:", connect_result)

    ip = wait_for_ip(wlan)

    if hasattr(network, "set_default_dev"):
        default_result = network.set_default_dev(WIFI_DEVICE)
        print("Default network device result:", default_result)

    print("Wi-Fi network information:", wlan.ifconfig())
    return wlan, ip


def capture_jpeg(sensor, encoder):
    return encoder.encode(sensor)


# RT-Smart caps a blocking socket's send timeout at 500 ms. Send incrementally
# in nonblocking mode so a large JPEG can wait for TCP backpressure safely.
def send_all(client, data):
    view = memoryview(data)
    offset = 0
    deadline = time.ticks_add(time.ticks_ms(), SEND_STALL_TIMEOUT_MS)

    while offset < len(view):
        try:
            end = min(offset + SEND_CHUNK_BYTES, len(view))
            sent = client.send(view[offset:end])
            if sent:
                offset += sent
                deadline = time.ticks_add(
                    time.ticks_ms(), SEND_STALL_TIMEOUT_MS
                )
                continue
        except OSError as error:
            if error.errno not in (11, 110):
                raise

        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            raise OSError(110)
        os.exitpoint()
        time.sleep_ms(1)


def send_response(client, status, content_type, body):
    header = (
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    ) % (status, content_type, len(body))
    send_all(client, header.encode())
    if body:
        send_all(client, body)


def read_path(client):
    request = bytearray()
    deadline = time.ticks_add(time.ticks_ms(), REQUEST_TIMEOUT_MS)
    client.setblocking(False)

    while len(request) < MAX_REQUEST_BYTES:
        chunk = None
        try:
            chunk = client.recv(min(256, MAX_REQUEST_BYTES - len(request)))
        except OSError as error:
            if error.errno not in (11, 110):
                raise

        if chunk:
            request.extend(chunk)
            if request.find(b"\r\n\r\n") >= 0 or request.find(b"\n\n") >= 0:
                break
        elif time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            raise OSError(110)
        else:
            os.exitpoint()
            time.sleep_ms(10)

    request_line = bytes(request).split(b"\r\n", 1)[0].split()
    if len(request_line) < 2:
        return "/"
    return request_line[1].decode().split("?", 1)[0]


def stream_mjpeg(client, sensor, encoder):
    send_all(
        client,
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        b"Cache-Control: no-store, no-cache, must-revalidate\r\n"
        b"Pragma: no-cache\r\n"
        b"Connection: close\r\n\r\n",
    )

    frame_interval = 1000 // STREAM_FPS
    frame_count = 0
    report_start = time.ticks_ms()

    while True:
        started = time.ticks_ms()
        jpeg = capture_jpeg(sensor, encoder)
        part_header = (
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n\r\n"
        ) % len(jpeg)
        send_all(client, part_header.encode())
        send_all(client, jpeg)
        send_all(client, b"\r\n")
        del jpeg

        frame_count += 1
        if frame_count % 10 == 0:
            gc.collect()

        now = time.ticks_ms()
        report_elapsed = time.ticks_diff(now, report_start)
        if report_elapsed >= 5000:
            actual_fps = frame_count * 1000.0 / report_elapsed
            print("MJPEG stream: %.2f fps" % actual_fps)
            frame_count = 0
            report_start = now

        delay = frame_interval - time.ticks_diff(time.ticks_ms(), started)
        if delay > 0:
            time.sleep_ms(delay)
        os.exitpoint()


def serve_client(client, address, sensor, encoder):
    path = read_path(client)
    print("HTTP client:", address, "path:", path)

    if path == "/stream":
        stream_mjpeg(client, sensor, encoder)
    elif path == "/favicon.ico":
        send_response(client, "204 No Content", "text/plain", b"")
    else:
        send_response(
            client,
            "200 OK",
            "text/html; charset=utf-8",
            INDEX_HTML,
        )


def safe_close(name, resource):
    if resource is None:
        return
    try:
        resource.close()
    except BaseException as error:
        print("Warning: failed to close %s: %s" % (name, error))


def main():
    netif = None
    sensor = None
    encoder = None
    server = None
    client = None

    try:
        netif, ip = connect_wifi()

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            alignment=FRAME_ALIGNMENT,
        )
        sensor.set_pixformat(Sensor.YUV420SP)

        encoder = HardwareJpegEncoder(
            FRAME_WIDTH,
            FRAME_HEIGHT,
            JPEG_QUALITY,
            STREAM_FPS,
        )
        print("JPEG encoder backend:", encoder.backend_name)
        sensor.run()

        # Let automatic exposure and white balance settle before serving.
        print("Warming up the camera...")
        for _ in range(10):
            sensor.snapshot()

        first_jpeg = capture_jpeg(sensor, encoder)
        print("First JPEG size:", len(first_jpeg), "bytes")
        del first_jpeg
        gc.collect()

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(
            socket.getaddrinfo("0.0.0.0", SERVER_PORT)[0][-1]
        )
        server.listen(2)
        server.setblocking(False)

        print("")
        print("Standalone K230 MJPEG server started.")
        print("Open this URL with Chrome or Edge:")
        print("  http://%s:%d/" % (ip, SERVER_PORT))
        print("Press the IDE Stop button to exit.")

        while True:
            try:
                client, address = server.accept()
            except OSError as error:
                if error.errno != 11:
                    raise
                os.exitpoint()
                time.sleep_ms(20)
                continue

            try:
                serve_client(client, address, sensor, encoder)
            except OSError as error:
                # Closing/reloading the browser normally ends an MJPEG stream.
                print("HTTP client disconnected:", error)
            finally:
                safe_close("HTTP client", client)
                client = None
                gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        safe_close("HTTP client", client)
        safe_close("HTTP server", server)

        if sensor is not None:
            try:
                sensor.stop()
            except BaseException as error:
                print("Warning: failed to stop sensor:", error)

        safe_close("JPEG encoder", encoder)

        # Keep the network object referenced until all sockets are closed.
        netif = None
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        gc.collect()
        print("Standalone K230 MJPEG server stopped.")


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
