"""K230 硬件绑定彩色显示与固定 ROI 传统钢球识别程序。

CH0 的 640x480 YUV420SP 彩色视频由硬件送入 LCD；CH2 灰度帧用于固定
ROI 二值识别。ARGB8888 OSD 遮黑 ROI 外的画面，并绘制准线和识别状态。
"""

import gc
import image
import network
import os
try:
    import select
except ImportError:
    select = None
import socket
import struct
import sys
import time
import uctypes
from machine import FPIOA, UART

from media.display import *
from media.media import *
from media.sensor import *
from media.vencoder import *


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SENSOR_FPS = 30

# ST7701、视频层与 OSD 均恢复为原生 640x480。
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
PANEL_WIDTH = 640
PANEL_HEIGHT = 480
DISPLAY_X = (PANEL_WIDTH - DISPLAY_WIDTH) // 2
DISPLAY_Y = (PANEL_HEIGHT - DISPLAY_HEIGHT) // 2

# ==================== Wi-Fi 与 UDP JPEG 图传 ====================
WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD"
WIFI_RETRY_INTERVAL_MS = 30000

STREAM_WIDTH = 320
STREAM_HEIGHT = 240
STREAM_FPS = 15
JPEG_QUALITY = 40

UDP_PORT = 9001
UDP_HELLO_MAGIC = b"KHEL"
UDP_FRAME_MAGIC = b"KUJP"
UDP_HEADER_FORMAT = ">4sIHHHHI"
UDP_HEADER_SIZE = 20
UDP_DATAGRAM_SIZE = 1200
UDP_PAYLOAD_SIZE = UDP_DATAGRAM_SIZE - UDP_HEADER_SIZE
UDP_MAX_FRAGMENT_COUNT = 128
UDP_PEER_TIMEOUT_MS = 5000
UDP_NO_STREAM_WARN_MS = 3000
UDP_ERROR_LOG_PERIOD_MS = 5000

VENC_CHANNEL = VENC_CHN_ID_0
VENC_BUFFER_NUM = 4
VENC_DRAIN_BUDGET_PER_STEP = 2
STREAM_STATS_PERIOD_MS = 5000

# ==================== ESP32视觉控制UART ====================
CONTROL_UART_BAUD = 115200
CONTROL_UART_TX_PIN = 9
CONTROL_UART_RX_PIN = 10
CONTROL_UART_FRAME_MAX = 96
CONTROL_UART_RX_BUDGET = 4

# 旧UDP控制实现暂时保留用于回退，正式运行使用VisionUartLink。
CONTROL_UDP_PORT = 9020
CONTROL_UDP_DATAGRAM_MAX = 96
CONTROL_UDP_RX_BUDGET = 4
CONTROL_HELLO_INTERVAL_MS = 1000
CONTROL_SEND_ERROR_RESET = 3

# 实际每 1 cm 对应屏幕约 2 mm，按 3 mm≈40 px 等差取整。
ACTUAL_CM_GRID_OFFSETS_PX = (
    27, 53, 80, 107, 133, 160, 187, 213, 240, 267
)
ROD_TARGET_LIMIT_MM = 110
CALIBRATED_TARGET_LIMIT_MM = len(ACTUAL_CM_GRID_OFFSETS_PX) * 10

# 原画面中的红框区域，格式为 (x, y, width, height)。
CROP_ROI = (0, 55, 640, 81)
BALL_GUIDE_Y_LOCAL = CROP_ROI[3] // 2
BALL_GUIDE_Y_WEIGHT = 12.0

# 亮度落在该范围内的像素输出为白色，其余输出为黑色。
BINARY_THRESHOLD = (180, 255)

# 先闭运算连接钢球高光碎片，再轻微膨胀增强小目标。
MORPH_CLOSE_SIZE = 1
MORPH_DILATE_SIZE = 1

MIN_BLOB_PIXELS = 20
MIN_BLOB_AREA = 30
MIN_BLOB_WIDTH = 6
MAX_BLOB_WIDTH = 50
MIN_BLOB_HEIGHT = 6
MAX_BLOB_HEIGHT = 36
MIN_BLOB_DENSITY = 0.18
MIN_APPROX_CIRCULARITY = 0.20
MAX_BLOB_RESULTS_PER_FRAME = 64
MAX_BALL_CANDIDATES = 16
MAX_BLOB_GUIDE_DISTANCE_PX = 12

TRACK_ALPHA = 0.70
TRACK_BETA = 0.20
TRACK_BASE_GATE_PX = 60
TRACK_MAX_GATE_PX = 180
TRACK_MISS_GATE_GROWTH_PX = 12
TRACK_MAX_MISSED_FRAMES = 5
TRACK_STALE_MS = 250
TRACK_RESIDUAL_LIMIT_PX = 120
TRACK_MAX_VELOCITY_PX_S = 5000
TRACK_VELOCITY_DEADBAND_PX_S = 45
TRACK_STATIONARY_DELTA_PX = 2
TRACK_STATIONARY_FRAMES = 3
TRACK_PROXIMITY_WEIGHT = 1.2

LOG_PERIOD_FRAMES = 60
GC_PERIOD_FRAMES = 180
OSD_INTERVAL_FRAMES = 6

BORDER_COLOR = (255, 0, 0)
STATUS_COLOR = (180, 180, 180)
CANDIDATE_COLOR = (255, 255, 0)
OUT_OF_GATE_COLOR = (255, 128, 0)
SELECTED_COLOR = (0, 255, 0)
CENTER_COLOR = (255, 0, 255)
FILTER_COLOR = (0, 255, 255)
BALL_GUIDE_COLOR = (0, 128, 255)
SCALE_GRID_COLOR = (128, 128, 128)
CROSSHAIR_COLOR = (255, 0, 0)
TARGET_COLOR = (255, 0, 255)
LINK_OK_COLOR = (0, 255, 0)
LINK_WAIT_COLOR = (255, 255, 255)
OUTSIDE_ROI_MASK_COLOR = (0, 0, 0)


class StreamState:
    """记录 UDP 图传状态，供本地状态栏显示。"""

    def __init__(self):
        self.running = True
        self.stopped = False
        self.connected = False
        self.encoded_fps = 0.0
        self.sent_fps = 0.0
        self.mbps = 0.0
        self.dropped_frames = 0
        self.error = ""


def target_mm_to_display_x(target_mm):
    """用等差标定表插值，将目标毫米坐标映射到屏幕横坐标。"""
    target_mm = int(target_mm)
    absolute_mm = abs(target_mm)
    if absolute_mm > CALIBRATED_TARGET_LIMIT_MM:
        raise ValueError("目标位置超出当前像素标定范围")

    offsets = (0,) + ACTUAL_CM_GRID_OFFSETS_PX
    whole_cm = absolute_mm // 10
    remainder_mm = absolute_mm % 10
    offset = offsets[whole_cm]
    if remainder_mm:
        next_offset = offsets[whole_cm + 1]
        offset += ((next_offset - offset) * remainder_mm + 5) // 10
    if target_mm < 0:
        return FRAME_WIDTH // 2 - offset
    return FRAME_WIDTH // 2 + offset


def frame_x_to_osd(x):
    """将识别坐标映射到 640 宽的本地 OSD。"""
    x = int(x)
    return max(
        0,
        min(DISPLAY_WIDTH - 1, x * DISPLAY_WIDTH // FRAME_WIDTH),
    )


def frame_y_to_osd(y):
    """将识别坐标映射到 480 高的本地 OSD。"""
    y = int(y)
    return max(
        0,
        min(DISPLAY_HEIGHT - 1, y * DISPLAY_HEIGHT // FRAME_HEIGHT),
    )


def frame_rect_to_osd(x, y, width, height):
    """把识别坐标矩形映射到本地 640x480 OSD。"""
    left = frame_x_to_osd(x)
    top = frame_y_to_osd(y)
    right = frame_x_to_osd(x + max(1, width) - 1)
    bottom = frame_y_to_osd(y + max(1, height) - 1)
    return (
        left,
        top,
        max(1, right - left),
        max(1, bottom - top),
    )


def calculate_broadcast_address(ip_address, netmask):
    """根据 DHCP 地址和掩码计算热点子网广播地址。"""
    try:
        ip_parts = [int(part) for part in ip_address.split(".")]
        mask_parts = [int(part) for part in netmask.split(".")]
        if len(ip_parts) != 4 or len(mask_parts) != 4:
            raise ValueError("IPv4 length")
        result = []
        for ip_part, mask_part in zip(ip_parts, mask_parts):
            if not (0 <= ip_part <= 255 and 0 <= mask_part <= 255):
                raise ValueError("IPv4 range")
            result.append((ip_part & mask_part) | (255 ^ mask_part))
        return ".".join(str(part) for part in result)
    except Exception:
        return "255.255.255.255"


class VisionUdpLink:
    """发现车载 ESP32 后双向交换 VIS/TGT。"""

    def __init__(self):
        self.sock = None
        self.peer = None
        self.broadcast_address = "255.255.255.255"
        self.last_hello_ms = None
        self.consecutive_send_errors = 0
        self.target_mm = 0
        self.target_x = FRAME_WIDTH // 2
        self.last_ball_x = None
        self.discovery_count = 0
        self.sent_frames = 0
        self.send_errors = 0
        self.received_hellos = 0
        self.received_targets = 0
        self.receive_errors = 0

    def init(self, wifi_sta, ip_address):
        self.close()
        try:
            self.broadcast_address = calculate_broadcast_address(
                ip_address,
                wifi_sta.ifconfig()[1],
            )
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1,
            )
            try:
                self.sock.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_BROADCAST,
                    1,
                )
            except Exception as error:
                print("[control] SO_BROADCAST unavailable:", error)
            self.sock.bind((ip_address, CONTROL_UDP_PORT))
            self.sock.setblocking(False)
            print(
                "[control] UDP ready %s:%d broadcast=%s" % (
                    ip_address,
                    CONTROL_UDP_PORT,
                    self.broadcast_address,
                )
            )
            return True
        except Exception as error:
            self.close()
            print("[control] init failed, vision continues:", error)
            return False

    def _clear_peer(self, reason):
        if self.peer is not None:
            print("[control] peer cleared (%s): %s" % (
                reason,
                self.peer[0],
            ))
        self.peer = None
        self.last_hello_ms = None
        self.consecutive_send_errors = 0

    def step_discovery(self):
        """未发现 ESP32 时每秒广播一次 HELLO。"""
        if self.sock is None or self.peer is not None:
            return False
        now_ms = time.ticks_ms()
        if (self.last_hello_ms is not None and
                time.ticks_diff(now_ms, self.last_hello_ms) <
                CONTROL_HELLO_INTERVAL_MS):
            return False
        self.last_hello_ms = now_ms
        try:
            self.sock.sendto(
                b"@HELLO,K230\r\n",
                (self.broadcast_address, CONTROL_UDP_PORT),
            )
            self.discovery_count += 1
            if self.discovery_count == 1 or self.discovery_count % 10 == 0:
                print("[control] discovering ESP32 via %s:%d" % (
                    self.broadcast_address,
                    CONTROL_UDP_PORT,
                ))
            return True
        except Exception as error:
            self.send_errors += 1
            if self.send_errors == 1 or self.send_errors % 20 == 0:
                print("[control] HELLO broadcast failed:", error)
            return False

    @staticmethod
    def _clamp_error(error_px):
        return max(
            -(FRAME_WIDTH - 1),
            min(FRAME_WIDTH - 1, int(error_px)),
        )

    def send_detection(self, ball_x, velocity_px_s):
        """发送当前真实测量；丢检时 VALID=0 并保留最后误差。"""
        valid = ball_x is not None
        if valid:
            ball_x = max(0, min(FRAME_WIDTH - 1, int(round(ball_x))))
            self.last_ball_x = ball_x
            error_px = self._clamp_error(ball_x - self.target_x)
        else:
            error_px = 0

        last_error_px = 0
        if self.last_ball_x is not None:
            last_error_px = self._clamp_error(
                self.last_ball_x - self.target_x
            )
        velocity_px_s = max(
            -TRACK_MAX_VELOCITY_PX_S,
            min(TRACK_MAX_VELOCITY_PX_S, int(velocity_px_s)),
        )
        line = "@VIS,%d,%d,%d,%d\r\n" % (
            1 if valid else 0,
            error_px,
            last_error_px,
            velocity_px_s,
        )
        if self.sock is None or self.peer is None:
            return False
        try:
            self.sock.sendto(line.encode("ascii"), self.peer)
            self.sent_frames += 1
            self.consecutive_send_errors = 0
            return True
        except Exception as error:
            self.send_errors += 1
            self.consecutive_send_errors += 1
            if self.send_errors == 1 or self.send_errors % 100 == 0:
                print("[control] VIS unicast failed:", error)
            if self.consecutive_send_errors >= CONTROL_SEND_ERROR_RESET:
                self._clear_peer("send failed")
            return False

    def _record_receive_error(self, reason):
        self.receive_errors += 1
        if self.receive_errors == 1 or self.receive_errors % 20 == 0:
            print("[control] rx invalid:", reason)

    def _parse_control_datagram(self, data, address):
        try:
            text = data.decode("ascii")
        except Exception:
            self._record_receive_error("non-ascii data")
            return False
        if not text.endswith("\r\n"):
            self._record_receive_error("missing CRLF")
            return False
        line = text[:-2]
        if "\r" in line or "\n" in line:
            self._record_receive_error("multiple frames in datagram")
            return False

        parts = line.split(",")
        if len(parts) == 2 and parts[0] == "@HELLO":
            if parts[1] != "ESP32":
                self._record_receive_error("frame=" + line)
                return False
            peer = (address[0], CONTROL_UDP_PORT)
            if self.peer is not None and self.peer[0] != peer[0]:
                self._record_receive_error(
                    "HELLO from non-peer %s" % address[0]
                )
                return False
            if self.peer is None:
                self.peer = peer
                self.consecutive_send_errors = 0
                print("[control] ESP32 discovered:", peer)
            self.received_hellos += 1
            return False

        if len(parts) != 2 or parts[0] != "@TGT":
            self._record_receive_error("frame=" + line)
            return False
        if self.peer is None or address[0] != self.peer[0]:
            self._record_receive_error(
                "TGT from non-peer %s" % address[0]
            )
            return False
        if parts[1].strip() != parts[1]:
            self._record_receive_error("target contains spaces")
            return False
        try:
            target_mm = int(parts[1])
        except Exception:
            self._record_receive_error("target is not integer")
            return False
        if abs(target_mm) > ROD_TARGET_LIMIT_MM:
            self._record_receive_error("target outside rod")
            return False
        if abs(target_mm) > CALIBRATED_TARGET_LIMIT_MM:
            self._record_receive_error(
                "target outside calibrated range: %dmm" % target_mm
            )
            return False

        self.target_mm = target_mm
        self.target_x = target_mm_to_display_x(target_mm)
        self.received_targets += 1
        print("[control] target=%+dmm x=%d from=%s" % (
            self.target_mm,
            self.target_x,
            str(self.peer),
        ))
        return True

    def poll_target(self):
        """非阻塞处理少量 HELLO/TGT 数据报。"""
        if self.sock is None:
            return False
        target_changed = False
        for _ in range(CONTROL_UDP_RX_BUDGET):
            try:
                data, address = self.sock.recvfrom(
                    CONTROL_UDP_DATAGRAM_MAX
                )
            except OSError as error:
                error_number = error.args[0] if error.args else None
                if error_number not in (11, 110, 116):
                    self._record_receive_error("recv failed: %s" % error)
                break
            except Exception as error:
                self._record_receive_error("recv failed: %s" % error)
                break
            if self._parse_control_datagram(data, address):
                target_changed = True
        return target_changed

    def close(self):
        self._clear_peer("closed")
        close_socket(self.sock)
        self.sock = None


class VisionUartLink(VisionUdpLink):
    """通过UART1双向交换VIS/TGT，不依赖Wi-Fi控制配对。"""

    def __init__(self):
        VisionUdpLink.__init__(self)
        self.uart = None
        self.rx_buffer = b""

    def init(self):
        self.close()
        try:
            fpioa = FPIOA()
            fpioa.set_function(
                CONTROL_UART_TX_PIN,
                fpioa.UART1_TXD,
                ie=0,
                oe=1,
                pu=1,
            )
            fpioa.set_function(
                CONTROL_UART_RX_PIN,
                fpioa.UART1_RXD,
                ie=1,
                oe=0,
                pu=1,
            )
            self.uart = UART(UART.UART1, CONTROL_UART_BAUD)
            self.rx_buffer = b""
            self.peer = ("UART1", CONTROL_UART_BAUD)
            self.consecutive_send_errors = 0
            print(
                "[control] UART1 ready TX=IO%d RX=IO%d baud=%d" % (
                    CONTROL_UART_TX_PIN,
                    CONTROL_UART_RX_PIN,
                    CONTROL_UART_BAUD,
                )
            )
            return True
        except Exception as error:
            self.uart = None
            self.peer = None
            print("[control] UART init failed:", error)
            return False

    def step_discovery(self):
        return False

    def send_detection(self, ball_x, velocity_px_s):
        """发送当前真实测量；丢检时VALID=0并保留最后误差。"""
        valid = ball_x is not None
        if valid:
            ball_x = max(0, min(FRAME_WIDTH - 1, int(round(ball_x))))
            self.last_ball_x = ball_x
            error_px = self._clamp_error(ball_x - self.target_x)
        else:
            error_px = 0

        last_error_px = 0
        if self.last_ball_x is not None:
            last_error_px = self._clamp_error(
                self.last_ball_x - self.target_x
            )
        velocity_px_s = max(
            -TRACK_MAX_VELOCITY_PX_S,
            min(TRACK_MAX_VELOCITY_PX_S, int(velocity_px_s)),
        )
        line = "@VIS,%d,%d,%d,%d\r\n" % (
            1 if valid else 0,
            error_px,
            last_error_px,
            velocity_px_s,
        )
        if self.uart is None:
            return False
        try:
            written = self.uart.write(line)
            if written is not None and written != len(line):
                raise OSError("short write %d/%d" % (written, len(line)))
            self.sent_frames += 1
            self.consecutive_send_errors = 0
            return True
        except Exception as error:
            self.send_errors += 1
            self.consecutive_send_errors += 1
            if self.send_errors == 1 or self.send_errors % 100 == 0:
                print("[control] VIS UART write failed:", error)
            return False

    def _parse_target_line(self, line):
        parts = line.split(",")
        if len(parts) != 2 or parts[0] != "@TGT":
            self._record_receive_error("frame=" + line)
            return False
        if parts[1].strip() != parts[1]:
            self._record_receive_error("target contains spaces")
            return False
        try:
            target_mm = int(parts[1])
        except Exception:
            self._record_receive_error("target is not integer")
            return False
        if abs(target_mm) > ROD_TARGET_LIMIT_MM:
            self._record_receive_error("target outside rod")
            return False
        if abs(target_mm) > CALIBRATED_TARGET_LIMIT_MM:
            self._record_receive_error(
                "target outside calibrated range: %dmm" % target_mm
            )
            return False

        self.target_mm = target_mm
        self.target_x = target_mm_to_display_x(target_mm)
        self.received_targets += 1
        print(
            "[control] target=%+dmm x=%d from=UART1" % (
                self.target_mm,
                self.target_x,
            )
        )
        return True

    def poll_target(self):
        """非阻塞拼接CRLF串口帧，每轮最多处理少量目标命令。"""
        if self.uart is None:
            return False
        target_changed = False
        for _ in range(CONTROL_UART_RX_BUDGET):
            newline_index = self.rx_buffer.find(b"\n")
            if newline_index < 0:
                available = self.uart.any()
                if not available:
                    break
                data = self.uart.read(min(int(available), 128))
                if not data:
                    break
                self.rx_buffer += data
                if (len(self.rx_buffer) > CONTROL_UART_FRAME_MAX and
                        self.rx_buffer.find(b"\n") < 0):
                    self._record_receive_error("line overflow")
                    self.rx_buffer = b""
                continue

            frame = self.rx_buffer[:newline_index + 1]
            self.rx_buffer = self.rx_buffer[newline_index + 1:]
            if (len(frame) > CONTROL_UART_FRAME_MAX or
                    not frame.endswith(b"\r\n")):
                self._record_receive_error("missing CRLF or overflow")
                continue
            try:
                line = frame[:-2].decode("ascii")
            except Exception:
                self._record_receive_error("non-ascii data")
                continue
            if self._parse_target_line(line):
                target_changed = True
        return target_changed

    def close(self):
        uart = self.uart
        self.uart = None
        self.peer = None
        self.rx_buffer = b""
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class AlphaBetaTracker1D:
    """钢球横坐标 Alpha-Beta 跟踪器。"""

    def __init__(self):
        self.max_observed_misses = 0
        self.reset()

    def reset(self):
        self.initialized = False
        self.x = 0.0
        self.v = 0.0
        self.state_ms = None
        self.last_measurement_ms = None
        self.last_raw_x = None
        self.missed_frames = 0
        self.stationary_frames = 0

    @staticmethod
    def _elapsed_seconds(now_ms, then_ms):
        if then_ms is None:
            return 0.0
        elapsed_ms = time.ticks_diff(now_ms, then_ms)
        if elapsed_ms <= 0:
            return 0.0
        return elapsed_ms / 1000.0

    def predicted_x(self, now_ms):
        if not self.initialized:
            return None
        dt = self._elapsed_seconds(now_ms, self.state_ms)
        return clamp(self.x + self.v * dt, 0, CROP_ROI[2] - 1)

    def dynamic_gate(self, now_ms):
        if not self.initialized:
            return TRACK_MAX_GATE_PX
        elapsed = self._elapsed_seconds(
            now_ms,
            self.last_measurement_ms,
        )
        gate = (
            TRACK_BASE_GATE_PX
            + abs(self.v) * elapsed
            + self.missed_frames * TRACK_MISS_GATE_GROWTH_PX
        )
        return clamp(
            gate,
            TRACK_BASE_GATE_PX,
            TRACK_MAX_GATE_PX,
        )

    def update(self, measured_x, now_ms):
        measured_x = float(measured_x)
        stale = (
            self.last_measurement_ms is None or
            time.ticks_diff(now_ms, self.last_measurement_ms) >
            TRACK_STALE_MS
        )
        if not self.initialized or stale:
            self.initialized = True
            self.x = measured_x
            self.v = 0.0
            self.state_ms = now_ms
            self.last_measurement_ms = now_ms
            self.last_raw_x = measured_x
            self.missed_frames = 0
            self.stationary_frames = 0
            return self.x, 0

        dt = self._elapsed_seconds(now_ms, self.state_ms)
        if dt <= 0.0:
            dt = 0.001
        predicted = self.x + self.v * dt
        residual = clamp(
            measured_x - predicted,
            -TRACK_RESIDUAL_LIMIT_PX,
            TRACK_RESIDUAL_LIMIT_PX,
        )
        self.x = clamp(
            predicted + TRACK_ALPHA * residual,
            0,
            CROP_ROI[2] - 1,
        )
        self.v += TRACK_BETA * residual / dt
        self.v = clamp(
            self.v,
            -TRACK_MAX_VELOCITY_PX_S,
            TRACK_MAX_VELOCITY_PX_S,
        )

        if (self.last_raw_x is not None and
                abs(measured_x - self.last_raw_x) <=
                TRACK_STATIONARY_DELTA_PX):
            self.stationary_frames += 1
        else:
            self.stationary_frames = 0
        if self.stationary_frames >= TRACK_STATIONARY_FRAMES:
            self.x = measured_x
            self.v = 0.0

        self.state_ms = now_ms
        self.last_measurement_ms = now_ms
        self.last_raw_x = measured_x
        self.missed_frames = 0
        return self.x, self.output_velocity()

    def miss(self, now_ms):
        if not self.initialized:
            return None, 0

        self.x = self.predicted_x(now_ms)
        self.state_ms = now_ms
        self.missed_frames += 1
        self.max_observed_misses = max(
            self.max_observed_misses,
            self.missed_frames,
        )
        stale = (
            self.last_measurement_ms is None or
            time.ticks_diff(now_ms, self.last_measurement_ms) >
            TRACK_STALE_MS
        )
        if self.missed_frames > TRACK_MAX_MISSED_FRAMES or stale:
            self.reset()
            return None, 0
        return self.x, self.output_velocity()

    def output_velocity(self):
        if (not self.initialized or
                abs(self.v) < TRACK_VELOCITY_DEADBAND_PX_S):
            return 0
        return int(self.v)


class WifiRetryManager:
    """非阻塞检查联网状态，离线时每30秒重新发起连接。"""

    def __init__(self):
        self.sta = network.WLAN(0)
        self.connected = False
        self.ip_address = ""
        self.last_attempt_ms = None
        self.attempt_count = 0

    def step(self, now_ms):
        became_connected = False
        became_disconnected = False
        current_ip = ""
        try:
            if self.sta.isconnected():
                current_ip = self.sta.ifconfig()[0]
                if current_ip == "0.0.0.0":
                    current_ip = ""
        except Exception as error:
            if self.connected:
                print("[wifi] status failed:", error)

        if current_ip:
            if (not self.connected or
                    current_ip != self.ip_address):
                became_disconnected = (
                    self.connected and current_ip != self.ip_address
                )
                self.connected = True
                self.ip_address = current_ip
                became_connected = True
                print("[wifi] connected, ip:", current_ip)
            return (
                True,
                self.ip_address,
                became_connected,
                became_disconnected,
            )

        if self.connected:
            print("[wifi] disconnected")
            self.connected = False
            self.ip_address = ""
            self.last_attempt_ms = None
            became_disconnected = True

        retry_due = (
            self.last_attempt_ms is None or
            time.ticks_diff(now_ms, self.last_attempt_ms) >=
            WIFI_RETRY_INTERVAL_MS
        )
        if retry_due:
            self.last_attempt_ms = now_ms
            self.attempt_count += 1
            try:
                print(
                    "[wifi] connect attempt %d, ssid=%s" % (
                        self.attempt_count,
                        WIFI_SSID,
                    )
                )
                self.sta.connect(WIFI_SSID, WIFI_PASSWORD)
            except Exception as error:
                print("[wifi] connect call failed, retry in 30s:", error)
        return False, "", False, became_disconnected


def create_jpeg_attributes(encoder, width, height):
    """创建 quality=40 的硬件 JPEG 编码参数。"""
    profile_none = globals().get("VENC_PROFILE_NONE", 0)
    try:
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_JPEG,
            profile_none,
            width,
            height,
            src_frame_rate=SENSOR_FPS,
            dst_frame_rate=SENSOR_FPS,
            mjpeg_quality_factor=JPEG_QUALITY,
        )
    except TypeError:
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_JPEG,
            profile_none,
            width,
            height,
        )


def create_udp_server(ip_address):
    """创建视频 UDP 端点，等待上位机 KHEL 心跳。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip_address, UDP_PORT))
    server.setblocking(False)
    print("[udp] listening on %s:%d" % (ip_address, UDP_PORT))
    return server


def close_socket(sock):
    if sock is None:
        return
    try:
        sock.close()
    except OSError:
        pass


def copy_jpeg_stream(stream_data):
    """在释放 VENC 码流前复制一幅完整 JPEG。"""
    packets = []
    for pack_index in range(stream_data.pack_cnt):
        packet_size = stream_data.data_size[pack_index]
        if packet_size <= 0:
            continue
        packets.append(bytes(uctypes.bytearray_at(
            stream_data.data[pack_index],
            packet_size,
        )))
    return b"".join(packets)


def send_udp_jpeg(server, peer, sequence, width, height, jpeg_data):
    """按 KUJP 协议把 JPEG 拆成不超过 1200 字节的数据报。"""
    jpeg_size = len(jpeg_data)
    fragment_count = (
        jpeg_size + UDP_PAYLOAD_SIZE - 1
    ) // UDP_PAYLOAD_SIZE
    if (fragment_count <= 0 or
            fragment_count > UDP_MAX_FRAGMENT_COUNT):
        raise ValueError("JPEG 分片数量异常")

    sent_bytes = 0
    for fragment_index in range(fragment_count):
        offset = fragment_index * UDP_PAYLOAD_SIZE
        payload = jpeg_data[offset:offset + UDP_PAYLOAD_SIZE]
        header = struct.pack(
            UDP_HEADER_FORMAT,
            UDP_FRAME_MAGIC,
            sequence,
            fragment_index,
            fragment_count,
            width,
            height,
            jpeg_size,
        )
        datagram = header + payload
        sent = server.sendto(datagram, peer)
        if sent is None or sent != len(datagram):
            raise OSError("UDP 数据报未完整发送")
        sent_bytes += sent
    return sent_bytes


class EncoderDrain:
    """网络服务未启动时取空 VENC 队列，防止编码链路反压 Sensor。"""

    def __init__(self, encoder):
        self.encoder = encoder
        self.stream_data = StreamData()

    def step(self):
        drained = 0
        for _ in range(VENC_DRAIN_BUDGET_PER_STEP):
            result = self.encoder.GetStream(
                VENC_CHANNEL,
                self.stream_data,
                timeout=0,
            )
            if result not in (0, None):
                break
            try:
                drained += 1
            finally:
                self.encoder.ReleaseStream(
                    VENC_CHANNEL,
                    self.stream_data,
                )
        return drained


class UdpStreamPump:
    """顺序取走 JPEG 码流，网络未连接时也不阻塞识别。"""

    def __init__(self, encoder, server, width, height, state):
        self.encoder = encoder
        self.server = server
        self.width = width
        self.height = height
        self.state = state
        self.stream_data = StreamData()
        self.peer = None
        self.peer_last_seen_ms = 0
        self.encoded_frames = 0
        self.sent_frames = 0
        self.sent_bytes = 0
        self.sequence = 0
        self.min_send_interval_ms = max(1, 1000 // STREAM_FPS)
        self.last_send_ms = 0
        self.stats_started_at = time.ticks_ms()
        self.no_stream_since_ms = self.stats_started_at
        self.first_stream_logged = False
        self.first_send_logged = False
        self.network_error_logged = False
        self.last_error_code = None
        self.last_error_log_ms = 0
        self.last_foreign_log_ms = 0
        self.poller = None
        if select is not None:
            try:
                self.poller = select.poll()
                self.poller.register(
                    server,
                    getattr(select, "POLLIN", 1),
                )
            except Exception as error:
                self.poller = None
                print("[udp] heartbeat poll unavailable:", error)
        else:
            print("[udp] select unavailable, streaming disabled")

    def _poll_peer(self, now_ms):
        if self.poller is None:
            return
        try:
            events = self.poller.poll(0)
            if not events:
                return
            data, address = self.server.recvfrom(32)
        except Exception as error:
            if not self.network_error_logged:
                print("[udp] heartbeat poll failed:", error)
                self.network_error_logged = True
            return
        if self.network_error_logged:
            print("[udp] heartbeat poll recovered")
            self.network_error_logged = False
        if data[:4] != UDP_HELLO_MAGIC:
            return
        if self.peer is not None and address != self.peer:
            if (time.ticks_diff(now_ms, self.last_foreign_log_ms) >=
                    UDP_ERROR_LOG_PERIOD_MS):
                self.last_foreign_log_ms = now_ms
                print(
                    "[udp] 忽略额外心跳来源 %s:%d，当前接收端 %s:%d" % (
                        address[0],
                        address[1],
                        self.peer[0],
                        self.peer[1],
                    )
                )
            return
        if self.peer is None:
            print("[udp] receiver registered:", address)
        self.peer = address
        self.peer_last_seen_ms = now_ms

    def _note_getstream_error(self, result, now_ms):
        self.last_error_code = result
        idle_ms = time.ticks_diff(now_ms, self.no_stream_since_ms)
        if (idle_ms >= UDP_NO_STREAM_WARN_MS and
                time.ticks_diff(now_ms, self.last_error_log_ms) >=
                UDP_ERROR_LOG_PERIOD_MS):
            self.last_error_log_ms = now_ms
            print(
                "[udp] encoder 无码流已 %d ms，GetStream 错误码: %r" % (
                    idle_ms,
                    result,
                )
            )

    def step(self):
        now_ms = time.ticks_ms()
        if (self.peer is not None and
                time.ticks_diff(now_ms, self.peer_last_seen_ms) >=
                UDP_PEER_TIMEOUT_MS):
            print("[udp] receiver heartbeat timeout:", self.peer)
            self.peer = None

        self._poll_peer(now_ms)
        self.state.connected = self.peer is not None

        latest_jpeg = None
        want_frame = (
            self.peer is not None and
            time.ticks_diff(now_ms, self.last_send_ms) >=
            self.min_send_interval_ms
        )

        for _ in range(VENC_DRAIN_BUDGET_PER_STEP):
            result = self.encoder.GetStream(
                VENC_CHANNEL,
                self.stream_data,
                timeout=0,
            )
            if result not in (0, None):
                self._note_getstream_error(result, now_ms)
                break
            self.no_stream_since_ms = time.ticks_ms()
            if not self.first_stream_logged:
                print("[udp] first encoded frame ready")
                self.first_stream_logged = True
            try:
                self.encoded_frames += 1
                if want_frame and latest_jpeg is None:
                    latest_jpeg = copy_jpeg_stream(self.stream_data)
            finally:
                self.encoder.ReleaseStream(
                    VENC_CHANNEL,
                    self.stream_data,
                )

        if latest_jpeg is not None:
            try:
                wire_bytes = send_udp_jpeg(
                    self.server,
                    self.peer,
                    self.sequence,
                    self.width,
                    self.height,
                    latest_jpeg,
                )
                self.sent_frames += 1
                self.sent_bytes += wire_bytes
                self.last_send_ms = time.ticks_ms()
                if not self.first_send_logged:
                    print("[udp] first JPEG sent, wire_bytes=%d" % wire_bytes)
                    self.first_send_logged = True
            except Exception as error:
                self.state.dropped_frames += 1
                error_number = error.args[0] if error.args else None
                if error_number not in (11, 110, 116):
                    print("[udp] send failed:", error)
                    self.peer = None
                    self.state.connected = False
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF

        self._update_stats(now_ms)
        return latest_jpeg is not None

    def _update_stats(self, now_ms):
        elapsed_ms = time.ticks_diff(now_ms, self.stats_started_at)
        if elapsed_ms < STREAM_STATS_PERIOD_MS:
            return
        elapsed_s = elapsed_ms / 1000.0
        self.state.encoded_fps = self.encoded_frames / elapsed_s
        self.state.sent_fps = self.sent_frames / elapsed_s
        self.state.mbps = (
            self.sent_bytes * 8.0 / elapsed_s / 1000000.0
        )
        print(
            "[udp] encoded=%.1ffps sent=%.1ffps "
            "rate=%.2fMbps drop=%d" % (
                self.state.encoded_fps,
                self.state.sent_fps,
                self.state.mbps,
                self.state.dropped_frames,
            )
        )
        if self.encoded_frames == 0:
            print("[udp] 本周期无编码帧，最近错误码: %r" % (
                self.last_error_code,
            ))
        self.encoded_frames = 0
        self.sent_frames = 0
        self.sent_bytes = 0
        self.stats_started_at = now_ms

    def close(self):
        self.state.running = False
        self.state.connected = False
        self.state.stopped = True
        if self.poller is not None:
            try:
                self.poller.unregister(self.server)
            except Exception:
                pass
            self.poller = None
        close_socket(self.server)
        print("[udp] stream pump stopped")


def make_binary_roi(frame):
    """裁出指定区域，校验尺寸后生成二值图。"""
    cropped = frame.copy(roi=CROP_ROI)
    actual_width = cropped.width()
    actual_height = cropped.height()
    if (actual_width != CROP_ROI[2] or
            actual_height != CROP_ROI[3]):
        raise RuntimeError(
            "ROI crop size mismatch: expected %dx%d, got %dx%d" % (
                CROP_ROI[2],
                CROP_ROI[3],
                actual_width,
                actual_height,
            )
        )
    cropped.binary([BINARY_THRESHOLD])
    if MORPH_CLOSE_SIZE > 0:
        cropped.close(MORPH_CLOSE_SIZE)
    if MORPH_DILATE_SIZE > 0:
        cropped.dilate(MORPH_DILATE_SIZE)
    return cropped


def find_ball_candidates(binary_roi):
    """筛除细长反光带，并限制扰动帧的候选数量。"""
    blobs = binary_roi.find_blobs(
        [(200, 255)],
        pixels_threshold=MIN_BLOB_PIXELS,
        area_threshold=MIN_BLOB_AREA,
        merge=False,
    )
    raw_blob_count = len(blobs)
    if raw_blob_count > MAX_BLOB_RESULTS_PER_FRAME:
        return [], raw_blob_count, True

    candidates = []
    guide_y = BALL_GUIDE_Y_LOCAL

    for blob in blobs:
        x, y, width, height = blob.rect()
        if (width < MIN_BLOB_WIDTH or width > MAX_BLOB_WIDTH or
                height < MIN_BLOB_HEIGHT or height > MAX_BLOB_HEIGHT):
            continue

        density = blob.density()
        circularity = min(width, height) / max(width, height)
        if (density < MIN_BLOB_DENSITY or
                circularity < MIN_APPROX_CIRCULARITY):
            continue

        center_x = x + width // 2
        blob_center_y = y + height // 2
        bottom_y = y + height - 1
        if y <= guide_y <= bottom_y:
            guide_distance = 0
        else:
            guide_distance = min(
                abs(y - guide_y),
                abs(bottom_y - guide_y),
            )
        if guide_distance > MAX_BLOB_GUIDE_DISTANCE_PX:
            continue
        pixels = blob.pixels()
        score = (
            pixels * 0.25
            + density * 70.0
            + circularity * 90.0
            - guide_distance * BALL_GUIDE_Y_WEIGHT
        )
        candidate = {
            "rect": (x, y, width, height),
            "center": (center_x, guide_y),
            "blob_center_y": blob_center_y,
            "guide_distance": guide_distance,
            "pixels": pixels,
            "density": density,
            "circularity": circularity,
            "score": score,
        }
        if len(candidates) < MAX_BALL_CANDIDATES:
            candidates.append(candidate)
            continue

        weakest_index = 0
        weakest_score = candidates[0]["score"]
        for index in range(1, len(candidates)):
            current_score = candidates[index]["score"]
            if current_score < weakest_score:
                weakest_index = index
                weakest_score = current_score
        if score > weakest_score:
            candidates[weakest_index] = candidate

    return candidates, raw_blob_count, False


def select_tracked_candidate(candidates, tracker, now_ms):
    """首次按外观锁定，之后在预测位置动态门限内关联候选。"""
    predicted_x = tracker.predicted_x(now_ms)
    gate = tracker.dynamic_gate(now_ms)
    best = None
    best_score = None

    for candidate in candidates:
        center_x = candidate["center"][0]
        if predicted_x is None:
            distance = 0.0
            inside_gate = True
            association_score = candidate["score"]
        else:
            distance = abs(center_x - predicted_x)
            inside_gate = distance <= gate
            association_score = (
                candidate["score"]
                + (gate - distance) * TRACK_PROXIMITY_WEIGHT
            )

        candidate["distance"] = distance
        candidate["inside_gate"] = inside_gate
        candidate["association_score"] = association_score
        if not inside_gate:
            continue
        if best_score is None or association_score > best_score:
            best = candidate
            best_score = association_score
    return best


def draw_osd_overlay(osd, candidates, raw_blob_count, best, tracker, raw_x,
                     filtered_x, velocity_px_s, fps, now_ms,
                     vision_control, stream_state,
                     wifi_connected, wifi_ip):
    """清空透明 OSD，再绘制准线、候选、球心与链路状态。"""
    osd.clear()
    frame = osd

    # 视频层仍由硬件整屏绑定，使用不透明OSD仅遮住ROI外的摄像头画面。
    roi_top = frame_y_to_osd(CROP_ROI[1])
    roi_bottom = frame_y_to_osd(CROP_ROI[1] + CROP_ROI[3] - 1)
    if roi_top > 0:
        frame.draw_rectangle(
            0,
            0,
            DISPLAY_WIDTH,
            roi_top,
            color=OUTSIDE_ROI_MASK_COLOR,
            fill=True,
        )
    if roi_bottom < DISPLAY_HEIGHT - 1:
        frame.draw_rectangle(
            0,
            roi_bottom + 1,
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT - roi_bottom - 1,
            color=OUTSIDE_ROI_MASK_COLOR,
            fill=True,
        )

    zero_x = DISPLAY_WIDTH // 2
    center_y = DISPLAY_HEIGHT // 2
    for offset in ACTUAL_CM_GRID_OFFSETS_PX:
        display_offset = offset * DISPLAY_WIDTH // FRAME_WIDTH
        left_x = zero_x - display_offset
        right_x = zero_x + display_offset
        if left_x >= 0:
            frame.draw_line(
                left_x, 0, left_x, DISPLAY_HEIGHT - 1,
                color=SCALE_GRID_COLOR,
                thickness=1,
            )
        if right_x < DISPLAY_WIDTH:
            frame.draw_line(
                right_x, 0, right_x, DISPLAY_HEIGHT - 1,
                color=SCALE_GRID_COLOR,
                thickness=1,
            )
    frame.draw_line(
        0, center_y, DISPLAY_WIDTH - 1, center_y,
        color=CROSSHAIR_COLOR,
        thickness=1,
    )
    frame.draw_line(
        zero_x, 0, zero_x, DISPLAY_HEIGHT - 1,
        color=CROSSHAIR_COLOR,
        thickness=2,
    )
    target_x = frame_x_to_osd(vision_control.target_x)
    if target_x != zero_x:
        frame.draw_line(
            target_x,
            0,
            target_x,
            DISPLAY_HEIGHT - 1,
            color=TARGET_COLOR,
            thickness=2,
        )

    guide_screen_y = frame_y_to_osd(
        CROP_ROI[1] + BALL_GUIDE_Y_LOCAL
    )
    frame.draw_line(
        frame_x_to_osd(CROP_ROI[0]),
        guide_screen_y,
        frame_x_to_osd(CROP_ROI[0] + CROP_ROI[2] - 1),
        guide_screen_y,
        color=BALL_GUIDE_COLOR,
        thickness=2,
    )

    roi_rect = frame_rect_to_osd(*CROP_ROI)
    frame.draw_rectangle(
        *roi_rect,
        color=BORDER_COLOR,
        thickness=2,
    )

    for candidate in candidates:
        x, y, width, height = candidate["rect"]
        color = (
            CANDIDATE_COLOR
            if candidate.get("inside_gate", True)
            else OUT_OF_GATE_COLOR
        )
        candidate_rect = frame_rect_to_osd(
            CROP_ROI[0] + x,
            CROP_ROI[1] + y,
            width,
            height,
        )
        frame.draw_rectangle(
            *candidate_rect, color=color, thickness=1
        )

    if best is not None:
        x, y, width, height = best["rect"]
        center_x, center_y = best["center"]
        selected_rect = frame_rect_to_osd(
            CROP_ROI[0] + x,
            CROP_ROI[1] + y,
            width,
            height,
        )
        frame.draw_rectangle(
            *selected_rect, color=SELECTED_COLOR, thickness=2
        )
        frame.draw_cross(
            frame_x_to_osd(CROP_ROI[0] + center_x),
            frame_y_to_osd(CROP_ROI[1] + center_y),
            color=CENTER_COLOR,
            size=3,
            thickness=1,
        )

    if filtered_x is not None:
        filtered_x_int = int(filtered_x)
        filtered_display_x = frame_x_to_osd(
            CROP_ROI[0] + filtered_x_int
        )
        frame.draw_line(
            filtered_display_x,
            frame_y_to_osd(CROP_ROI[1]),
            filtered_display_x,
            frame_y_to_osd(CROP_ROI[1] + CROP_ROI[3] - 1),
            color=FILTER_COLOR,
            thickness=1,
        )

    raw_text = "--" if raw_x is None else str(raw_x)
    filtered_text = (
        "--" if filtered_x is None else str(int(filtered_x))
    )
    track_state = "TRACK" if tracker.initialized else "SEARCH"
    error_text = "--"
    if raw_x is not None:
        error_text = "%+d" % (raw_x - vision_control.target_x)

    frame.draw_string_advanced(
        8,
        DISPLAY_HEIGHT - 72,
        16,
        "TGT %+dMM E %s RAW %s FILTER %s V %+d" % (
            vision_control.target_mm,
            error_text,
            raw_text,
            filtered_text,
            velocity_px_s,
        ),
        color=TARGET_COLOR,
    )
    frame.draw_string_advanced(
        8,
        DISPLAY_HEIGHT - 48,
        16,
        "%s BLOB %d CAND %d GATE %d MISS %d/%d FPS %.1f" % (
            track_state,
            raw_blob_count,
            len(candidates),
            int(tracker.dynamic_gate(now_ms)),
            tracker.missed_frames,
            tracker.max_observed_misses,
            fps,
        ),
        color=STATUS_COLOR,
    )
    udp_text = "UDP WAIT"
    if stream_state.connected:
        udp_text = "UDP %.1fFPS %.1fM" % (
            stream_state.sent_fps,
            stream_state.mbps,
        )
    control_text = "CTL WAIT"
    if vision_control.peer is not None:
        control_text = "CTL OK"
    wifi_text = wifi_ip if wifi_connected else "FAIL"
    frame.draw_string_advanced(
        8,
        DISPLAY_HEIGHT - 24,
        16,
        "WIFI %s  %s  %s" % (
            wifi_text,
            udp_text,
            control_text,
        ),
        color=(LINK_OK_COLOR if wifi_connected else LINK_WAIT_COLOR),
    )


def main():
    sensor = None
    osd = None
    encoder = None
    media_link = None
    video_server = None
    udp_pump = None
    encoder_drain = None
    stream_state = StreamState()
    vision_control = VisionUartLink()
    wifi_manager = WifiRetryManager()
    wifi_ip = ""
    wifi_connected = False
    network_service_retry_ms = 0
    sensor_started = False
    display_initialized = False
    media_init_attempted = False
    encoder_created = False
    encoder_started = False
    osd_error_count = 0
    tracker = AlphaBetaTracker1D()
    candidate_overload_count = 0

    try:
        print("[main] hardware YUV display with grayscale ROI detection")
        if not vision_control.init():
            raise RuntimeError("视觉控制UART初始化失败")
        print("[crop] source=640x480 roi=%s" % (CROP_ROI,))
        print("[binary] threshold=%s" % (BINARY_THRESHOLD,))
        print(
            "[track] alpha=%.2f beta=%.2f gate=%d..%dpx "
            "miss=%d stale=%dms" % (
                TRACK_ALPHA,
                TRACK_BETA,
                TRACK_BASE_GATE_PX,
                TRACK_MAX_GATE_PX,
                TRACK_MAX_MISSED_FRAMES,
                TRACK_STALE_MS,
            )
        )

        display_width = ALIGN_UP(DISPLAY_WIDTH, 16)
        stream_width = ALIGN_UP(STREAM_WIDTH, 16)
        sensor = Sensor(fps=SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(
            width=display_width,
            height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0,
            alignment=12,
        )
        sensor.set_pixformat(
            Sensor.YUV420SP,
            chn=CAM_CHN_ID_0,
        )

        sensor.set_framesize(
            width=stream_width,
            height=STREAM_HEIGHT,
            chn=CAM_CHN_ID_1,
            alignment=12,
        )
        sensor.set_pixformat(
            Sensor.YUV420SP,
            chn=CAM_CHN_ID_1,
        )

        sensor.set_framesize(
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            chn=CAM_CHN_ID_2,
        )
        sensor.set_pixformat(
            Sensor.GRAYSCALE,
            chn=CAM_CHN_ID_2,
        )

        Display.bind_layer(
            **sensor.bind_info(
                x=DISPLAY_X,
                y=DISPLAY_Y,
                chn=CAM_CHN_ID_0,
            ),
            layer=Display.LAYER_VIDEO1
        )

        Display.init(
            Display.ST7701,
            width=PANEL_WIDTH,
            height=PANEL_HEIGHT,
            osd_num=1,
            to_ide=True,
        )
        display_initialized = True
        osd = image.Image(
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            image.ARGB8888,
        )
        print(
            "[display] image=%dx%d panel=%dx%d offset=(%d,%d) "
            "hardware bind + ARGB OSD" % (
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT,
                Display.width(),
                Display.height(),
                DISPLAY_X,
                DISPLAY_Y,
            )
        )

        encoder = Encoder()
        encoder.SetOutBufs(
            VENC_CHANNEL,
            VENC_BUFFER_NUM,
            stream_width,
            STREAM_HEIGHT,
        )
        media_link = MediaManager.link(
            sensor.bind_info(chn=CAM_CHN_ID_1)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_CHANNEL),
        )

        media_init_attempted = True
        MediaManager.init()

        encoder.Create(
            VENC_CHANNEL,
            create_jpeg_attributes(
                encoder,
                stream_width,
                STREAM_HEIGHT,
            ),
        )
        encoder_created = True
        encoder.Start(VENC_CHANNEL)
        encoder_started = True

        sensor.run()
        sensor_started = True

        encoder_drain = EncoderDrain(encoder)
        print(
            "[jpeg] UDP %dx%d sensor=%dfps send=%dfps quality=%d" % (
                stream_width,
                STREAM_HEIGHT,
                SENSOR_FPS,
                STREAM_FPS,
                JPEG_QUALITY,
            )
        )

        clock = time.clock()
        frame_number = 0

        while True:
            os.exitpoint()
            clock.tick()

            detection_frame = sensor.snapshot(chn=CAM_CHN_ID_2)
            cropped = make_binary_roi(detection_frame)
            now_ms = time.ticks_ms()

            (wifi_connected, wifi_ip,
             became_connected,
             became_disconnected) = wifi_manager.step(now_ms)
            if became_disconnected:
                if udp_pump is not None:
                    udp_pump.close()
                    udp_pump = None
                    video_server = None
                stream_state = StreamState()
                network_service_retry_ms = now_ms
            if became_connected:
                network_service_retry_ms = now_ms

            services_missing = udp_pump is None
            if (wifi_connected and services_missing and
                    time.ticks_diff(
                        now_ms,
                        network_service_retry_ms,
                    ) >= 0):
                network_service_retry_ms = (
                    now_ms + WIFI_RETRY_INTERVAL_MS
                )
                if udp_pump is None:
                    try:
                        video_server = create_udp_server(wifi_ip)
                        stream_state = StreamState()
                        udp_pump = UdpStreamPump(
                            encoder,
                            video_server,
                            stream_width,
                            STREAM_HEIGHT,
                            stream_state,
                        )
                        print("[udp] video service started")
                    except Exception as error:
                        close_socket(video_server)
                        video_server = None
                        udp_pump = None
                        print(
                            "[udp] service start failed, retry in 30s:",
                            error,
                        )

            (candidates,
             raw_blob_count,
             candidate_overloaded) = find_ball_candidates(cropped)
            if candidate_overloaded:
                candidate_overload_count += 1
            best = select_tracked_candidate(
                candidates,
                tracker,
                now_ms,
            )
            if best is not None:
                raw_x = best["center"][0]
                filtered_x, velocity_px_s = tracker.update(
                    raw_x,
                    now_ms,
                )
            else:
                raw_x = None
                filtered_x, velocity_px_s = tracker.miss(now_ms)

            vision_control.poll_target()
            vision_control.step_discovery()
            vision_control.send_detection(raw_x, velocity_px_s)
            if udp_pump is not None:
                udp_pump.step()
            elif encoder_drain is not None:
                encoder_drain.step()

            frame_number += 1

            if frame_number % OSD_INTERVAL_FRAMES == 0:
                draw_osd_overlay(
                    osd,
                    candidates,
                    raw_blob_count,
                    best,
                    tracker,
                    raw_x,
                    filtered_x,
                    velocity_px_s,
                    clock.fps(),
                    now_ms,
                    vision_control,
                    stream_state,
                    wifi_connected,
                    wifi_ip,
                )
                try:
                    Display.show_image(
                        osd,
                        DISPLAY_X,
                        DISPLAY_Y,
                        Display.LAYER_OSD3,
                    )
                except RuntimeError as error:
                    osd_error_count += 1
                    if (osd_error_count == 1 or
                            osd_error_count % 60 == 0):
                        print(
                            "[display] OSD submit failed %d: %s" % (
                                osd_error_count,
                                error,
                            )
                        )

            if frame_number % LOG_PERIOD_FRAMES == 0:
                print(
                    "[track] blob=%d cand=%d overload=%d "
                    "raw=%s filter=%s v=%+d "
                    "gate=%d miss=%d/%d fps=%.1f" % (
                        raw_blob_count,
                        len(candidates),
                        candidate_overload_count,
                        ("--" if raw_x is None else str(raw_x)),
                        ("--" if filtered_x is None
                         else str(int(filtered_x))),
                        velocity_px_s,
                        int(tracker.dynamic_gate(now_ms)),
                        tracker.missed_frames,
                        tracker.max_observed_misses,
                        clock.fps(),
                    )
                )
                print(
                    "[binary] threshold=%d..%d morph=%d/%d" % (
                        BINARY_THRESHOLD[0],
                        BINARY_THRESHOLD[1],
                        MORPH_CLOSE_SIZE,
                        MORPH_DILATE_SIZE,
                    )
                )
                print(
                    "[link] wifi=%s udp=%.1ffps %.2fMbps "
                    "ctl=%s vis=%d/%d tgt=%d/%d target=%+dmm" % (
                        wifi_ip if wifi_connected else "FAIL",
                        stream_state.sent_fps,
                        stream_state.mbps,
                        (vision_control.peer[0]
                         if vision_control.peer is not None else "WAIT"),
                        vision_control.sent_frames,
                        vision_control.send_errors,
                        vision_control.received_targets,
                        vision_control.receive_errors,
                        vision_control.target_mm,
                    )
                )
            detection_frame = None
            cropped = None
            candidates = None
            best = None
            if frame_number % GC_PERIOD_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("[main] 用户停止集成程序")
    except Exception as error:
        print("[main] 集成程序异常:", repr(error))
        try:
            sys.print_exception(error)
        except Exception:
            pass
    finally:
        vision_control.close()
        if udp_pump is not None:
            udp_pump.close()
        else:
            close_socket(video_server)

        if sensor_started and sensor is not None:
            sensor.stop()
        if display_initialized:
            Display.deinit()
            time.sleep_ms(50)
        if media_link is not None:
            try:
                media_link.destroy()
            except Exception as error:
                print("[main] media link cleanup failed:", error)
        if encoder_started and encoder is not None:
            try:
                encoder.Stop(VENC_CHANNEL)
            except Exception as error:
                print("[main] encoder stop failed:", error)
        if encoder_created and encoder is not None:
            try:
                encoder.Destroy(VENC_CHANNEL)
            except Exception as error:
                print("[main] encoder destroy failed:", error)
        if media_init_attempted:
            try:
                MediaManager.deinit()
            except Exception as error:
                print("[main] MediaManager cleanup failed:", error)
        gc.collect()
        print("[main] 资源已释放")


if __name__ == "__main__":
    main()
