"""K230 钢珠识别、本地显示和低延迟 JPEG 图传集成程序。

摄像头通道分工：
- CH0：640x480 YUV420SP，绑定 K230 LCD；
- CH1：320x240 YUV420SP，绑定 JPEG 硬件编码器并通过 UDP/TCP 发送；
- CH2：320x240 RGB888P，供 KPU 钢珠模型推理。

电脑端收到的是不带 OSD 的原始画面；红色 ROI、绿色检测框、黄色全画幅
十字准星和状态文字只显示在 K230 本地屏幕上。
"""

import _thread
import gc
import image
import network
import nncase_runtime as nn
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
import ujson

from libs.PlatTasks import DetectionApp
from media.display import *
from media.media import *
from media.sensor import *
from media.vencoder import *


# ==================== Wi-Fi 与图传协议参数 ====================
WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD"
WIFI_TIMEOUT_MS = 15000

# 默认使用 UDP；改为 "tcp" 可回退到原 TCP 图传。
STREAM_TRANSPORT = "udp"

TCP_PORT = 9000
TCP_BACKLOG = 1
FRAME_MAGIC = b"KJPG"
HEADER_FORMAT = ">4sIHHI"
CLIENT_SOCKET_TIMEOUT_SECONDS = 1
SEND_STALL_TIMEOUT_MS = 5000

UDP_PORT = 9001
UDP_HELLO_MAGIC = b"KHEL"
UDP_FRAME_MAGIC = b"KUJP"
UDP_HEADER_FORMAT = ">4sIHHHHI"
UDP_HEADER_SIZE = 20
UDP_DATAGRAM_SIZE = 1200
UDP_PAYLOAD_SIZE = UDP_DATAGRAM_SIZE - UDP_HEADER_SIZE
UDP_PEER_TIMEOUT_MS = 5000
# 编码器持续无码流超过该时长后，改用阻塞探测并周期性打印错误码。
UDP_NO_STREAM_WARN_MS = 3000
UDP_PROBE_TIMEOUT_MS = 50
UDP_ERROR_LOG_PERIOD_MS = 5000

# ==================== 视觉控制 UDP 参数 ====================
# 端口 9020 只承载 @HELLO/@VIS/@TGT，不与 9001 JPEG 图传混用。
CONTROL_UDP_PORT = 9020
CONTROL_UDP_DATAGRAM_MAX = 96
CONTROL_UDP_RX_BUDGET = 4
CONTROL_HELLO_INTERVAL_MS = 1000
CONTROL_SEND_ERROR_RESET = 3
VELOCITY_ALPHA = 0.70
VELOCITY_BETA = 0.20
VELOCITY_STALE_MS = 200
VELOCITY_MAX_ABS_PX_S = 5000
VELOCITY_DEADBAND_PX_S = 45
VELOCITY_RESIDUAL_LIMIT_PX = 120
VELOCITY_STATIONARY_DELTA_PX = 2
VELOCITY_STATIONARY_FRAMES = 3

# ==================== 摄像头、显示与编码参数 ====================
# FRAME 是 ROI 与本地显示使用的 640x480 坐标系。
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
PANEL_WIDTH = 640
PANEL_HEIGHT = 480
# 当前先按等差标定：实际每 1 cm 对应屏幕 0.2 cm（2 mm）。
# 按屏幕 3 mm 约等于 40 px 换算，累计位置取整以避免误差累积。
ACTUAL_CM_GRID_OFFSETS_PX = (
    27, 53, 80, 107, 133, 160, 187, 213, 240, 267
)
ROD_TARGET_LIMIT_MM = 110
CALIBRATED_TARGET_LIMIT_MM = len(ACTUAL_CM_GRID_OFFSETS_PX) * 10
STREAM_WIDTH = 320
STREAM_HEIGHT = 240
AI_FRAME_WIDTH = 320
AI_FRAME_HEIGHT = 240
SENSOR_FPS = 30
STREAM_FPS = 15
JPEG_QUALITY = 40

VENC_CHANNEL = VENC_CHN_ID_0
VENC_BUFFER_NUM = 8
ENCODER_TIMEOUT_MS = 500
STREAM_STATS_PERIOD_MS = 5000

CAMERA_EXPOSURE_US = 2500
CAMERA_GAIN_DB = 12

# ==================== 模型与检测参数 ====================
MODEL_ROOT_CANDIDATES = (
    "/sdcard/st/mp_deployment_source/",
    "/sdcard/mp_deployment_source/",
)
CONFIDENCE_THRESHOLD = 0.40
ROI_CONFIDENCE_THRESHOLD = 0.50
POST_NMS_IOU_THRESHOLD = 0.30

MIN_BOX_WIDTH = 9
MAX_BOX_WIDTH = 50
MIN_BOX_HEIGHT = 8
MAX_BOX_HEIGHT = 35
MIN_ASPECT_RATIO = 0.50
MAX_ASPECT_RATIO = 3.00

ROI_X_MIN = 0
ROI_Y_MIN = 55
ROI_X_MAX = FRAME_WIDTH - 1
ROI_Y_MAX = 135
AI_ROI_X_MIN = ROI_X_MIN * AI_FRAME_WIDTH // FRAME_WIDTH
AI_ROI_Y_MIN = ROI_Y_MIN * AI_FRAME_HEIGHT // FRAME_HEIGHT
AI_ROI_X_MAX = ROI_X_MAX * AI_FRAME_WIDTH // FRAME_WIDTH
AI_ROI_Y_MAX = ROI_Y_MAX * AI_FRAME_HEIGHT // FRAME_HEIGHT

PRESENCE_HOLD_FRAMES = 4
OSD_INTERVAL = 6
WIFI_CHECK_PERIOD = 120
FPS_PRINT_PERIOD = 120
GC_PERIOD = 240
SNAPSHOT_FAILURE_LIMIT = 60
DEBUG_MODE = 0

ROI_COLOR = (255, 0, 0)
TRACK_COLOR = (0, 255, 0)
CROSSHAIR_COLOR = (255, 255, 0)
SCALE_GRID_COLOR = (128, 128, 128)
TARGET_COLOR = (255, 0, 255)
STATUS_COLOR = (255, 255, 255)
ERROR_COLOR = (255, 0, 0)


class StreamState:
    """主线程与 JPEG 图传线程之间共享的轻量状态。"""

    def __init__(self):
        self.transport = STREAM_TRANSPORT.upper()
        self.running = True
        self.stopped = False
        self.connected = False
        self.client = None
        self.encoded_fps = 0.0
        self.sent_fps = 0.0
        self.mbps = 0.0
        self.dropped_frames = 0
        self.error = ""


def target_mm_to_display_x(target_mm):
    """用厘米标定表插值，将目标毫米坐标映射到屏幕横坐标。"""
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
        offset += (
            (next_offset - offset) * remainder_mm + 5
        ) // 10

    center_x = FRAME_WIDTH // 2
    if target_mm < 0:
        return center_x - offset
    return center_x + offset


def calculate_broadcast_address(ip_address, netmask):
    """根据 DHCP 地址和掩码计算当前热点子网广播地址。"""
    try:
        ip_parts = [int(part) for part in ip_address.split(".")]
        mask_parts = [int(part) for part in netmask.split(".")]
        if len(ip_parts) != 4 or len(mask_parts) != 4:
            raise ValueError("IPv4 length")
        broadcast_parts = []
        for ip_part, mask_part in zip(ip_parts, mask_parts):
            if not (0 <= ip_part <= 255 and 0 <= mask_part <= 255):
                raise ValueError("IPv4 range")
            broadcast_parts.append(
                (ip_part & mask_part) | (255 ^ mask_part)
            )
        return ".".join(str(part) for part in broadcast_parts)
    except Exception:
        return "255.255.255.255"


class VisionUdpLink:
    """发现车载 ESP32 后，在同一 UDP socket 中双向交换 VIS/TGT。"""

    def __init__(self):
        self.sock = None
        self.local_ip = ""
        self.broadcast_address = "255.255.255.255"
        self.peer = None
        self.last_hello_ms = None
        self.consecutive_send_errors = 0
        self.target_mm = 0
        self.target_x = FRAME_WIDTH // 2
        self.last_ball_x = None
        self.last_measurement_ms = None
        self.velocity_position_x = 0.0
        self.velocity_px_s = 0.0
        self.velocity_initialized = False
        self.velocity_stationary_frames = 0
        self.output_velocity_px_s = 0
        self.discovery_count = 0
        self.sent_frames = 0
        self.send_errors = 0
        self.received_hellos = 0
        self.received_targets = 0
        self.receive_errors = 0
        self.last_controller = None

    def init(self, wifi_sta, ip_address):
        self._close_socket()
        self.peer = None
        self.last_hello_ms = None
        self.consecutive_send_errors = 0
        try:
            netmask = wifi_sta.ifconfig()[1]
            self.broadcast_address = calculate_broadcast_address(
                ip_address,
                netmask,
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
            self.local_ip = ip_address
            print(
                "[control] UDP ready %s:%d broadcast=%s" % (
                    ip_address,
                    CONTROL_UDP_PORT,
                    self.broadcast_address,
                )
            )
            return True
        except Exception as error:
            self._close_socket()
            print("[control] init failed, vision continues:", error)
            return False

    def _close_socket(self):
        close_socket(self.sock)
        self.sock = None
        self.local_ip = ""

    def _clear_peer(self, reason):
        if self.peer is not None:
            print(
                "[control] peer cleared (%s): %s" % (
                    reason,
                    self.peer[0],
                )
            )
        self.peer = None
        self.last_controller = None
        self.last_hello_ms = None
        self.consecutive_send_errors = 0

    def update_network(self, wifi_sta, connected, ip_address):
        """在 Wi-Fi 状态变化时重建控制 socket，不阻塞视觉循环。"""
        if not connected or not ip_address:
            if self.sock is not None or self.peer is not None:
                self._clear_peer("wifi disconnected")
                self._close_socket()
                print("[control] waiting for Wi-Fi")
            return False

        if self.sock is not None and self.local_ip == ip_address:
            return True

        if self.local_ip and self.local_ip != ip_address:
            print(
                "[control] local IP changed: %s -> %s" % (
                    self.local_ip,
                    ip_address,
                )
            )
        self._clear_peer("network changed")
        return self.init(wifi_sta, ip_address)

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
                print(
                    "[control] discovering ESP32 via %s:%d" % (
                        self.broadcast_address,
                        CONTROL_UDP_PORT,
                    )
                )
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

    def _update_velocity(self, ball_x, now_ms):
        """用 Alpha-Beta 状态估计器计算640坐标系横向速度。"""
        if ball_x is not None:
            if (not self.velocity_initialized or
                    self.last_measurement_ms is None):
                self.velocity_position_x = float(ball_x)
                self.velocity_px_s = 0.0
                self.velocity_initialized = True
                self.velocity_stationary_frames = 0
            else:
                elapsed_ms = time.ticks_diff(
                    now_ms,
                    self.last_measurement_ms,
                )
                if elapsed_ms <= 0 or elapsed_ms > VELOCITY_STALE_MS:
                    self.velocity_position_x = float(ball_x)
                    self.velocity_px_s = 0.0
                    self.velocity_stationary_frames = 0
                else:
                    dt = elapsed_ms / 1000.0
                    predicted_x = (
                        self.velocity_position_x +
                        self.velocity_px_s * dt
                    )
                    residual = ball_x - predicted_x
                    residual = max(
                        -VELOCITY_RESIDUAL_LIMIT_PX,
                        min(VELOCITY_RESIDUAL_LIMIT_PX, residual),
                    )
                    self.velocity_position_x = (
                        predicted_x + VELOCITY_ALPHA * residual
                    )
                    self.velocity_px_s += (
                        VELOCITY_BETA * residual / dt
                    )
                    self.velocity_px_s = max(
                        -VELOCITY_MAX_ABS_PX_S,
                        min(VELOCITY_MAX_ABS_PX_S, self.velocity_px_s),
                    )

                    if (self.last_ball_x is not None and
                            abs(ball_x - self.last_ball_x) <=
                            VELOCITY_STATIONARY_DELTA_PX):
                        self.velocity_stationary_frames += 1
                    else:
                        self.velocity_stationary_frames = 0
                    if (self.velocity_stationary_frames >=
                            VELOCITY_STATIONARY_FRAMES):
                        self.velocity_position_x = float(ball_x)
                        self.velocity_px_s = 0.0
            self.last_measurement_ms = now_ms

        velocity_is_fresh = (
            self.last_measurement_ms is not None and
            time.ticks_diff(now_ms, self.last_measurement_ms) <=
            VELOCITY_STALE_MS
        )
        if not velocity_is_fresh:
            self.velocity_position_x = 0.0
            self.velocity_px_s = 0.0
            self.velocity_initialized = False
            self.velocity_stationary_frames = 0
            self.output_velocity_px_s = 0
        elif self.velocity_initialized:
            if abs(self.velocity_px_s) < VELOCITY_DEADBAND_PX_S:
                self.output_velocity_px_s = 0
            else:
                self.output_velocity_px_s = int(round(self.velocity_px_s))
        else:
            self.output_velocity_px_s = 0
        return self.output_velocity_px_s

    def send_detection(self, ball_x):
        now_ms = time.ticks_ms()
        valid = ball_x is not None
        if valid:
            ball_x = max(0, min(FRAME_WIDTH - 1, int(round(ball_x))))
        velocity_px_s = self._update_velocity(ball_x, now_ms)
        if valid:
            self.last_ball_x = ball_x
            error_px = self._clamp_error(ball_x - self.target_x)
        else:
            error_px = 0

        last_error_px = 0
        if self.last_ball_x is not None:
            last_error_px = self._clamp_error(
                self.last_ball_x - self.target_x
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
            if (self.send_errors == 1 or
                    self.send_errors % 100 == 0):
                print("[control] VIS unicast failed:", error)
            if self.consecutive_send_errors >= CONTROL_SEND_ERROR_RESET:
                self._clear_peer("send failed")
            return False

    def _record_receive_error(self, reason):
        self.receive_errors += 1
        if (self.receive_errors == 1 or
                self.receive_errors % 20 == 0):
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
                self.last_controller = peer
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
        self.last_controller = self.peer
        print(
            "[control] target=%+dmm x=%d from=%s" % (
                self.target_mm,
                self.target_x,
                str(self.peer),
            )
        )
        return True

    def poll_target(self):
        """非阻塞处理少量 HELLO/TGT，防止控制接收占用视觉循环。"""
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
                    self._record_receive_error(
                        "recv failed: %s" % error
                    )
                break
            except Exception as error:
                self._record_receive_error("recv failed: %s" % error)
                break
            if self._parse_control_datagram(data, address):
                target_changed = True
        return target_changed

    def close(self):
        self._clear_peer("closed")
        self._close_socket()


def path_exists(path):
    """兼容 CanMV MicroPython 的文件存在性检查。"""
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def find_model_root():
    """查找同时包含部署配置和 kmodel 的目录。"""
    for root in MODEL_ROOT_CANDIDATES:
        config_path = root + "deploy_config.json"
        if not path_exists(config_path):
            continue
        with open(config_path, "r") as config_file:
            config = ujson.load(config_file)
        model_path = root + config["kmodel_path"]
        if path_exists(model_path):
            return root, config, model_path
    raise OSError(
        "未找到模型，请把 mp_deployment_source 放到 /sdcard/st/"
    )


def validate_config(config):
    """在分配 KPU 内存前检查关键模型参数。"""
    if config.get("chip_type") != "k230":
        raise ValueError("deploy_config.json 不是 K230 模型")
    if config.get("nncase_version") != "2.9.0":
        raise ValueError("模型 nncase 版本不是 2.9.0")
    if config.get("model_type") != "AnchorBaseDet":
        raise ValueError("当前脚本仅支持 AnchorBaseDet")
    if config.get("categories") != ["ball"]:
        raise ValueError("模型类别不是单类别 ball")


def flatten_anchors(config):
    anchors = []
    for group in config["anchors"]:
        anchors.extend(group)
    return anchors


def result_to_detections(result):
    """过滤置信度和异常尺寸，转换为中心点格式。"""
    detections = []
    if not result:
        return 0, detections

    boxes = result["boxes"]
    for index in range(len(boxes)):
        box = boxes[index]
        score = float(result["scores"][index])
        if score < ROI_CONFIDENCE_THRESHOLD:
            continue

        width = int(box[2] - box[0])
        height = int(box[3] - box[1])
        if (width < MIN_BOX_WIDTH or width > MAX_BOX_WIDTH or
                height < MIN_BOX_HEIGHT or height > MAX_BOX_HEIGHT):
            continue

        aspect_ratio = width / height
        if (aspect_ratio < MIN_ASPECT_RATIO or
                aspect_ratio > MAX_ASPECT_RATIO):
            continue

        center_x = int(box[0] + width // 2)
        center_y = int(box[1] + height // 2)
        detections.append(
            (center_x, center_y, width, height, score)
        )
    return len(boxes), detections


def filter_roi_detections(detections):
    """只保留中心点位于当前红色 ROI 内的钢珠候选。"""
    roi_detections = []
    for detection in detections:
        center_x = detection[0]
        center_y = detection[1]
        if (AI_ROI_X_MIN <= center_x <= AI_ROI_X_MAX and
                AI_ROI_Y_MIN <= center_y <= AI_ROI_Y_MAX):
            roi_detections.append(detection)
    return roi_detections


def detection_iou(first, second):
    """计算两个中心点格式检测框的交并比。"""
    first_left = first[0] - first[2] / 2
    first_top = first[1] - first[3] / 2
    first_right = first[0] + first[2] / 2
    first_bottom = first[1] + first[3] / 2

    second_left = second[0] - second[2] / 2
    second_top = second[1] - second[3] / 2
    second_right = second[0] + second[2] / 2
    second_bottom = second[1] + second[3] / 2

    intersection_width = max(
        0, min(first_right, second_right) - max(first_left, second_left)
    )
    intersection_height = max(
        0, min(first_bottom, second_bottom) - max(first_top, second_top)
    )
    intersection_area = intersection_width * intersection_height
    if intersection_area <= 0:
        return 0.0

    union_area = (
        first[2] * first[3]
        + second[2] * second[3]
        - intersection_area
    )
    return intersection_area / union_area if union_area > 0 else 0.0


def suppress_overlapping_detections(detections):
    """执行一次更严格的 NMS，消除钢珠附近的重叠框。"""
    ordered = sorted(detections, key=lambda item: item[4], reverse=True)
    kept = []
    for detection in ordered:
        overlaps = False
        for selected in kept:
            if detection_iou(detection, selected) > POST_NMS_IOU_THRESHOLD:
                overlaps = True
                break
        if not overlaps:
            kept.append(detection)
    return kept


def select_best_roi_detection(detections):
    """物理上只有一颗钢珠，只保留 ROI 内置信度最高的目标。"""
    roi_detections = filter_roi_detections(detections)
    roi_detections = suppress_overlapping_detections(roi_detections)
    if not roi_detections:
        return []
    return [roi_detections[0]]


def connect_wifi():
    """连接热点并等待 DHCP 地址。"""
    sta = network.WLAN(0)
    print("[wifi] connecting ssid:", WIFI_SSID)
    sta.connect(WIFI_SSID, WIFI_PASSWORD)
    started_at = time.ticks_ms()

    while True:
        if sta.isconnected():
            ip_address = sta.ifconfig()[0]
            if ip_address and ip_address != "0.0.0.0":
                print("[wifi] connected, ip:", ip_address)
                return sta, ip_address

        if time.ticks_diff(time.ticks_ms(), started_at) >= WIFI_TIMEOUT_MS:
            raise RuntimeError("Wi-Fi 连接超时")
        time.sleep_ms(100)


def read_wifi_status(sta):
    """非阻塞读取当前 Wi-Fi 状态，不在识别循环内重连。"""
    if sta is None:
        return False, ""
    try:
        if not sta.isconnected():
            return False, ""
        ip_address = sta.ifconfig()[0]
        if not ip_address or ip_address == "0.0.0.0":
            return False, ""
        return True, ip_address
    except Exception as error:
        print("[wifi] status check failed:", error)
        return False, ""


def create_jpeg_attributes(encoder, width, height):
    """创建 JPEG 编码参数，并兼容不同版本的 ChnAttrStr。"""
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


def create_tcp_server(ip_address):
    """创建非阻塞 TCP 监听套接字。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip_address, TCP_PORT))
    server.listen(TCP_BACKLOG)
    server.setblocking(False)
    print("[tcp] listening on %s:%d" % (ip_address, TCP_PORT))
    return server


def create_udp_server(ip_address):
    """创建 UDP 端点，等待电脑心跳登记其地址。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip_address, UDP_PORT))
    server.setblocking(False)
    print("[udp] listening on %s:%d" % (ip_address, UDP_PORT))
    return server


def try_accept_client(server):
    """非阻塞接收一个电脑客户端。"""
    try:
        client, address = server.accept()
    except OSError as error:
        if error.args and error.args[0] in (11, 110, 116):
            return None
        raise

    try:
        client.setblocking(True)
        client.settimeout(CLIENT_SOCKET_TIMEOUT_SECONDS)
        if (hasattr(socket, "IPPROTO_TCP") and
                hasattr(socket, "TCP_NODELAY")):
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass
    print("[tcp] client connected:", address)
    return client


def close_socket(sock):
    if sock is None:
        return
    try:
        sock.close()
    except OSError:
        pass


def send_all(sock, data):
    """处理 TCP 短写，确保一段数据完整发出。"""
    offset = 0
    length = len(data)
    last_progress_ms = time.ticks_ms()
    while offset < length:
        try:
            sent = sock.send(data[offset:])
        except OSError as error:
            error_number = error.args[0] if error.args else None
            stalled_ms = time.ticks_diff(
                time.ticks_ms(),
                last_progress_ms,
            )
            if (error_number in (11, 110, 116) and
                    stalled_ms < SEND_STALL_TIMEOUT_MS):
                time.sleep_ms(10)
                continue
            raise
        if sent is None or sent <= 0:
            raise OSError("TCP 连接已关闭")
        offset += sent
        last_progress_ms = time.ticks_ms()


def copy_jpeg_stream(stream_data):
    """在释放 VENC 码流前复制当前完整 JPEG。"""
    packets = []
    for pack_index in range(stream_data.pack_cnt):
        packet_size = stream_data.data_size[pack_index]
        if packet_size <= 0:
            continue
        packets.append(
            bytes(
                uctypes.bytearray_at(
                    stream_data.data[pack_index],
                    packet_size,
                )
            )
        )
    return b"".join(packets)


def jpeg_tcp_worker(encoder, server, width, height, state):
    """持续取走硬件 JPEG 码流；网络阻塞不会卡住 KPU 主循环。"""
    client = None
    stream_data = StreamData()
    encoded_frames = 0
    sent_frames = 0
    sent_bytes = 0
    sequence = 0
    send_divider = max(1, SENSOR_FPS // STREAM_FPS)
    stats_started_at = time.ticks_ms()

    try:
        while state.running:
            if client is None:
                client = try_accept_client(server)
                state.client = client
                state.connected = client is not None

            result = encoder.GetStream(
                VENC_CHANNEL,
                stream_data,
                timeout=ENCODER_TIMEOUT_MS,
            )
            if result not in (0, None):
                time.sleep_ms(2)
                continue

            jpeg_data = None
            try:
                encoded_frames += 1
                should_send = (
                    client is not None
                    and encoded_frames % send_divider == 0
                )
                if should_send:
                    jpeg_data = copy_jpeg_stream(stream_data)
            finally:
                encoder.ReleaseStream(VENC_CHANNEL, stream_data)

            if jpeg_data:
                header = struct.pack(
                    HEADER_FORMAT,
                    FRAME_MAGIC,
                    sequence,
                    width,
                    height,
                    len(jpeg_data),
                )
                try:
                    send_all(client, header)
                    send_all(client, jpeg_data)
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    sent_frames += 1
                    sent_bytes += len(jpeg_data)
                except OSError as error:
                    print("[tcp] client disconnected:", error)
                    close_socket(client)
                    client = None
                    state.client = None
                    state.connected = False

            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, stats_started_at)
            if elapsed_ms >= STREAM_STATS_PERIOD_MS:
                elapsed_s = elapsed_ms / 1000.0
                state.encoded_fps = encoded_frames / elapsed_s
                state.sent_fps = sent_frames / elapsed_s
                state.mbps = sent_bytes * 8.0 / elapsed_s / 1000000.0
                print(
                    "[jpeg] encoded=%.1ffps sent=%.1ffps rate=%.2fMbps" % (
                        state.encoded_fps,
                        state.sent_fps,
                        state.mbps,
                    )
                )
                encoded_frames = 0
                sent_frames = 0
                sent_bytes = 0
                stats_started_at = now_ms

    except Exception as error:
        if state.running:
            state.error = repr(error)
            print("[jpeg] worker exception:", state.error)
    finally:
        state.connected = False
        state.client = None
        close_socket(client)
        close_socket(server)
        state.stopped = True
        print("[jpeg] worker stopped")


def send_udp_jpeg(server, peer, sequence, width, height, jpeg_data):
    """把一幅 JPEG 拆成小于 MTU 的数据报；不在 UDP 层重传。"""
    jpeg_size = len(jpeg_data)
    fragment_count = (
        jpeg_size + UDP_PAYLOAD_SIZE - 1
    ) // UDP_PAYLOAD_SIZE
    if fragment_count <= 0 or fragment_count > 65535:
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


class UdpStreamPump:
    """在主循环中顺序取 JPEG 码流，避免与 Sensor API 跨线程并发。"""

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
        # 按时间节流发送，保证不超过 STREAM_FPS。
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
        """零等待读取至多一个心跳；已有活跃接收端时忽略其它来源。"""
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
            # 电脑上重复运行接收程序会出现第二个心跳来源；若平分
            # 分片将导致两端都无法组帧，因此保持绑定当前接收端。
            if (time.ticks_diff(now_ms, self.last_foreign_log_ms)
                    >= UDP_ERROR_LOG_PERIOD_MS):
                self.last_foreign_log_ms = now_ms
                print(
                    "[udp] 忽略额外心跳来源 %s:%d，当前接收端 %s:%d；"
                    "请确认电脑上只运行了一个接收程序" % (
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
        self.peer_last_seen_ms = time.ticks_ms()

    def _note_getstream_error(self, result, now_ms):
        """记录取流错误码；持续无码流时周期性打印，便于定位链路问题。"""
        self.last_error_code = result
        idle_ms = time.ticks_diff(now_ms, self.no_stream_since_ms)
        if (idle_ms >= UDP_NO_STREAM_WARN_MS and
                time.ticks_diff(now_ms, self.last_error_log_ms)
                >= UDP_ERROR_LOG_PERIOD_MS):
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
                time.ticks_diff(now_ms, self.peer_last_seen_ms)
                >= UDP_PEER_TIMEOUT_MS):
            print("[udp] receiver heartbeat timeout:", self.peer)
            self.peer = None

        self._poll_peer(now_ms)
        self.state.connected = self.peer is not None

        # 长时间取不到帧时用阻塞探测兜底，区分“编码链路真无输出”
        # 与“非阻塞取流路径异常”两种情况。
        starved = (
            time.ticks_diff(now_ms, self.no_stream_since_ms)
            >= UDP_NO_STREAM_WARN_MS
        )
        getstream_timeout = UDP_PROBE_TIMEOUT_MS if starved else 0

        # 尽量取空编码队列，只保留最新一帧，避免旧帧积压增加延迟。
        latest_jpeg = None
        want_frame = (
            self.peer is not None and
            time.ticks_diff(now_ms, self.last_send_ms)
            >= self.min_send_interval_ms
        )
        while True:
            result = self.encoder.GetStream(
                VENC_CHANNEL,
                self.stream_data,
                timeout=getstream_timeout,
            )
            getstream_timeout = 0
            if result not in (0, None):
                self._note_getstream_error(result, now_ms)
                break
            self.no_stream_since_ms = time.ticks_ms()
            if not self.first_stream_logged:
                print("[udp] first encoded frame ready")
                self.first_stream_logged = True
            try:
                self.encoded_frames += 1
                if want_frame:
                    latest_jpeg = copy_jpeg_stream(self.stream_data)
            finally:
                self.encoder.ReleaseStream(VENC_CHANNEL, self.stream_data)

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
                    print(
                        "[udp] first JPEG sent, wire_bytes=%d" % wire_bytes
                    )
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
            print(
                "[udp] 本周期无编码帧，最近 GetStream 错误码: %r" % (
                    self.last_error_code,
                )
            )
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


def calculate_detection_center_x(detections):
    """返回钢球中心的 640 像素屏幕横坐标；未检测时返回 None。"""
    if not detections:
        return None
    center_x = detections[0][0]
    display_x = int(
        center_x * FRAME_WIDTH // AI_FRAME_WIDTH
    )
    return max(0, min(FRAME_WIDTH - 1, display_x))


def draw_local_overlay(
        osd, display_size, display_origin, detections, present, ai_fps,
        wifi_connected, wifi_ip, stream_state, target_x, target_mm,
        control_peer, ball_velocity_px_s):
    """在本地视频区域上方绘制准星、检测和网络状态。"""
    osd.clear()
    origin_x = display_origin[0]
    origin_y = display_origin[1]

    roi_x = origin_x + ROI_X_MIN * display_size[0] // FRAME_WIDTH
    roi_y = origin_y + ROI_Y_MIN * display_size[1] // FRAME_HEIGHT
    roi_width = (
        (ROI_X_MAX - ROI_X_MIN + 1) * display_size[0] // FRAME_WIDTH
    )
    roi_height = (
        (ROI_Y_MAX - ROI_Y_MIN + 1) * display_size[1] // FRAME_HEIGHT
    )
    osd.draw_rectangle(
        roi_x,
        roi_y,
        roi_width,
        roi_height,
        color=ROI_COLOR,
        thickness=3,
    )

    detection_centers = []
    for detection in detections:
        center_x, center_y, width, height, _ = detection
        x = origin_x + max(
            0,
            int((center_x - width / 2) *
                display_size[0] // AI_FRAME_WIDTH),
        )
        y = origin_y + max(
            0,
            int((center_y - height / 2) *
                display_size[1] // AI_FRAME_HEIGHT),
        )
        box_width = max(
            1, int(width * display_size[0] // AI_FRAME_WIDTH)
        )
        box_height = max(
            1, int(height * display_size[1] // AI_FRAME_HEIGHT)
        )
        osd.draw_rectangle(
            x,
            y,
            box_width,
            box_height,
            color=TRACK_COLOR,
            thickness=2,
        )
        detection_centers.append((
            origin_x + int(
                center_x * display_size[0] // AI_FRAME_WIDTH
            ),
            origin_y + int(
                center_y * display_size[1] // AI_FRAME_HEIGHT
            ),
        ))

    zero_x = origin_x + display_size[0] // 2
    center_y = origin_y + display_size[1] // 2
    bottom_y = origin_y + display_size[1] - 1
    right_x = origin_x + display_size[0] - 1
    for calibrated_offset in ACTUAL_CM_GRID_OFFSETS_PX:
        offset = (
            calibrated_offset * display_size[0] // FRAME_WIDTH
        )
        left_grid_x = zero_x - offset
        right_grid_x = zero_x + offset
        if left_grid_x >= origin_x:
            osd.draw_line(
                left_grid_x,
                origin_y,
                left_grid_x,
                bottom_y,
                color=SCALE_GRID_COLOR,
                thickness=1,
            )
        if right_grid_x <= right_x:
            osd.draw_line(
                right_grid_x,
                origin_y,
                right_grid_x,
                bottom_y,
                color=SCALE_GRID_COLOR,
                thickness=1,
            )

    osd.draw_line(
        origin_x,
        center_y,
        right_x,
        center_y,
        color=CROSSHAIR_COLOR,
        thickness=1,
    )
    osd.draw_line(
        zero_x,
        origin_y,
        zero_x,
        bottom_y,
        color=CROSSHAIR_COLOR,
        thickness=1,
    )

    target_display_x = origin_x + (
        target_x * display_size[0] // FRAME_WIDTH
    )
    if target_display_x != zero_x:
        osd.draw_line(
            target_display_x,
            origin_y,
            target_display_x,
            bottom_y,
            color=TARGET_COLOR,
            thickness=2,
        )

    for point_x, point_y in detection_centers:
        osd.draw_circle(
            point_x,
            point_y,
            3,
            color=TRACK_COLOR,
            fill=True,
        )

    error_px_text = "--"
    if detection_centers:
        error_px_text = "%+d" % (
            detection_centers[0][0] - target_display_x
        )
    status_y = origin_y + display_size[1] - 96
    osd.draw_string_advanced(
        origin_x + 4,
        status_y,
        20,
        "TARGET %+dMM" % target_mm,
        color=TARGET_COLOR,
    )
    osd.draw_string_advanced(
        origin_x + 4,
        status_y + 24,
        20,
        "BALL %s N%d AI %.1f E_PX %s V %+d" % (
            "YES" if present else "NO",
            len(detections),
            ai_fps,
            error_px_text,
            ball_velocity_px_s,
        ),
        color=TRACK_COLOR if present else STATUS_COLOR,
    )

    wifi_text = "WIFI FAIL"
    wifi_color = ERROR_COLOR
    if wifi_connected:
        wifi_text = "WIFI OK " + wifi_ip
        wifi_color = TRACK_COLOR
    osd.draw_string_advanced(
        origin_x + 4,
        status_y + 48,
        20,
        wifi_text,
        color=wifi_color,
    )

    control_text = "CTL WAIT %d" % CONTROL_UDP_PORT
    control_color = STATUS_COLOR
    if control_peer is not None:
        control_text = "CTL OK " + control_peer[0]
        control_color = TRACK_COLOR
    osd.draw_string_advanced(
        origin_x + display_size[0] // 2,
        status_y + 48,
        20,
        control_text,
        color=control_color,
    )

    transport = stream_state.transport
    if stream_state.error:
        stream_text = transport + " ERROR"
        stream_color = ERROR_COLOR
    elif stream_state.connected:
        stream_text = "%s ON %.1fFPS %.2fM D%d" % (
            transport,
            stream_state.sent_fps,
            stream_state.mbps,
            stream_state.dropped_frames,
        )
        stream_color = TRACK_COLOR
    else:
        port = UDP_PORT if transport == "UDP" else TCP_PORT
        stream_text = "%s WAIT %d" % (transport, port)
        stream_color = STATUS_COLOR
    osd.draw_string_advanced(
        origin_x + 4,
        status_y + 72,
        20,
        stream_text,
        color=stream_color,
    )


def apply_camera_exposure(sensor):
    """尽量固定曝光，减少高速钢珠运动模糊；失败时继续自动曝光。"""
    try:
        sensor.set_auto_exposure(False, exposure_us=CAMERA_EXPOSURE_US)
        print("[ball] exposure=%dus" % CAMERA_EXPOSURE_US)
    except Exception as error:
        print("[ball] fixed exposure unavailable:", error)

    try:
        sensor.set_auto_gain(False, gain_db=CAMERA_GAIN_DB)
        print("[ball] gain=%ddB" % CAMERA_GAIN_DB)
    except Exception as error:
        print("[ball] fixed gain unavailable:", error)


def wait_stream_worker(state, thread_started, server):
    """关闭网络套接字解除阻塞，再等待编码线程退出。"""
    if state is None:
        return
    state.running = False
    if not thread_started:
        state.stopped = True
        return

    close_socket(state.client)
    close_socket(server)
    started_at = time.ticks_ms()
    timeout_ms = (
        SEND_STALL_TIMEOUT_MS + ENCODER_TIMEOUT_MS + 1500
    )
    while not state.stopped:
        if time.ticks_diff(time.ticks_ms(), started_at) >= timeout_ms:
            print("[jpeg] worker stop timeout")
            break
        time.sleep_ms(20)


def main():
    sensor = None
    encoder = None
    detector = None
    osd = None
    media_link = None
    server = None
    wifi_sta = None
    vision_control = VisionUdpLink()
    stream_state = None
    udp_pump = None
    stream_thread_started = False
    display_initialized = False
    media_init_attempted = False
    encoder_created = False
    encoder_started = False
    sensor_started = False

    try:
        model_root, config, model_path = find_model_root()
        validate_config(config)
        model_input_size = config["img_size"]

        print("[ball] model root:", model_root)
        print("[ball] model:", model_path)
        print(
            "[ball] roi=(%d,%d)-(%d,%d) ai=%dx%d" % (
                ROI_X_MIN,
                ROI_Y_MIN,
                ROI_X_MAX,
                ROI_Y_MAX,
                AI_FRAME_WIDTH,
                AI_FRAME_HEIGHT,
            )
        )

        nn.shrink_memory_pool()
        detector = DetectionApp(
            mode="video",
            kmodel_path=model_path,
            labels=config["categories"],
            model_input_size=model_input_size,
            anchors=flatten_anchors(config),
            model_type=config["model_type"],
            confidence_threshold=CONFIDENCE_THRESHOLD,
            nms_threshold=min(
                config["nms_threshold"], POST_NMS_IOU_THRESHOLD
            ),
            rgb888p_size=[AI_FRAME_WIDTH, AI_FRAME_HEIGHT],
            display_size=[DISPLAY_WIDTH, DISPLAY_HEIGHT],
            debug_mode=DEBUG_MODE,
        )
        detector.nms_option = config.get("nms_option", False)
        detector.config_preprocess()

        wifi_sta, wifi_ip = connect_wifi()
        vision_control.init(wifi_sta, wifi_ip)
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
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)

        sensor.set_framesize(
            width=stream_width,
            height=STREAM_HEIGHT,
            chn=CAM_CHN_ID_1,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)

        sensor.set_framesize(
            width=AI_FRAME_WIDTH,
            height=AI_FRAME_HEIGHT,
            chn=CAM_CHN_ID_2,
        )
        sensor.set_pixformat(
            PIXEL_FORMAT_RGB_888_PLANAR,
            chn=CAM_CHN_ID_2,
        )

        display_x = (PANEL_WIDTH - DISPLAY_WIDTH) // 2
        display_y = (PANEL_HEIGHT - DISPLAY_HEIGHT) // 2
        Display.bind_layer(
            **sensor.bind_info(
                x=display_x,
                y=display_y,
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
        display_size = [DISPLAY_WIDTH, DISPLAY_HEIGHT]
        display_origin = [display_x, display_y]
        osd = image.Image(
            Display.width(),
            Display.height(),
            image.ARGB8888,
        )
        print(
            "[display] image=%dx%d panel=%dx%d offset=(%d,%d)" % (
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT,
                Display.width(),
                Display.height(),
                display_x,
                display_y,
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
        apply_camera_exposure(sensor)

        if STREAM_TRANSPORT not in ("udp", "tcp"):
            raise ValueError("STREAM_TRANSPORT 必须是 udp 或 tcp")

        stream_state = StreamState()
        if STREAM_TRANSPORT == "udp":
            server = create_udp_server(wifi_ip)
            udp_pump = UdpStreamPump(
                encoder,
                server,
                stream_width,
                STREAM_HEIGHT,
                stream_state,
            )
        else:
            server = create_tcp_server(wifi_ip)
            _thread.start_new_thread(
                jpeg_tcp_worker,
                (
                    encoder,
                    server,
                    stream_width,
                    STREAM_HEIGHT,
                    stream_state,
                ),
            )
            stream_thread_started = True

        print(
            "[jpeg] transport=%s frame=%dx%d sensor=%dfps "
            "send=%dfps quality=%d" % (
                STREAM_TRANSPORT.upper(),
                stream_width,
                STREAM_HEIGHT,
                SENSOR_FPS,
                STREAM_FPS,
                JPEG_QUALITY,
            )
        )

        wifi_connected = True
        frame_number = 0
        missed_frames = PRESENCE_HOLD_FRAMES + 1
        previous_present = None
        clock = time.clock()

        draw_local_overlay(
            osd,
            display_size,
            display_origin,
            [],
            False,
            0.0,
            wifi_connected,
            wifi_ip,
            stream_state,
            vision_control.target_x,
            vision_control.target_mm,
            vision_control.peer,
            vision_control.output_velocity_px_s,
        )
        Display.show_image(osd, 0, 0, Display.LAYER_OSD3)

        consecutive_snapshot_failures = 0

        while True:
            os.exitpoint()
            if (udp_pump is None and
                    stream_state.stopped and stream_state.error):
                raise RuntimeError(
                    "JPEG 图传线程异常: " + stream_state.error
                )

            clock.tick()
            try:
                frame_image = sensor.snapshot(chn=CAM_CHN_ID_2)
            except RuntimeError as error:
                consecutive_snapshot_failures += 1
                if (consecutive_snapshot_failures == 1 or
                        consecutive_snapshot_failures % 10 == 0):
                    print(
                        "[ball] snapshot retry %d/%d: %s" % (
                            consecutive_snapshot_failures,
                            SNAPSHOT_FAILURE_LIMIT,
                            error,
                        )
                    )
                if consecutive_snapshot_failures >= SNAPSHOT_FAILURE_LIMIT:
                    raise
                time.sleep_ms(2)
                continue
            if frame_image is None:
                consecutive_snapshot_failures += 1
                if consecutive_snapshot_failures >= SNAPSHOT_FAILURE_LIMIT:
                    raise RuntimeError("AI 通道连续返回空帧")
                time.sleep_ms(2)
                continue
            consecutive_snapshot_failures = 0
            frame = frame_image.to_numpy_ref()

            result = detector.run(frame)
            _, detections = result_to_detections(result)
            roi_detections = select_best_roi_detection(detections)
            frame = None
            frame_image = None
            result = None

            if udp_pump is not None:
                udp_pump.step()
            vision_control.poll_target()
            vision_control.step_discovery()

            if roi_detections:
                missed_frames = 0
            else:
                missed_frames += 1
            present = missed_frames <= PRESENCE_HOLD_FRAMES
            current_ball_x = calculate_detection_center_x(
                roi_detections
            )
            vision_control.send_detection(current_ball_x)

            frame_number += 1
            if frame_number % WIFI_CHECK_PERIOD == 0:
                old_wifi_connected = wifi_connected
                old_wifi_ip = wifi_ip
                wifi_connected, wifi_ip = read_wifi_status(wifi_sta)
                vision_control.update_network(
                    wifi_sta,
                    wifi_connected,
                    wifi_ip,
                )
                if (wifi_connected != old_wifi_connected or
                        wifi_ip != old_wifi_ip):
                    print(
                        "[wifi] link state=%s ip=%s" % (
                            "connected" if wifi_connected else "disconnected",
                            wifi_ip if wifi_ip else "--",
                        )
                    )

            if frame_number % OSD_INTERVAL == 0:
                draw_local_overlay(
                    osd,
                    display_size,
                    display_origin,
                    roi_detections,
                    present,
                    clock.fps(),
                    wifi_connected,
                    wifi_ip,
                    stream_state,
                    vision_control.target_x,
                    vision_control.target_mm,
                    vision_control.peer,
                    vision_control.output_velocity_px_s,
                )
                Display.show_image(osd, 0, 0, Display.LAYER_OSD3)

            if present != previous_present:
                print(
                    "[ball] present=%d count=%d" % (
                        1 if present else 0,
                        len(roi_detections),
                    )
                )
                previous_present = present

            if frame_number % FPS_PRINT_PERIOD == 0:
                print(
                    "[ball] ai=%.1ffps %s=%.1ffps "
                    "ctl_peer=%s vis_tx=%d/%d tgt_rx=%d/%d "
                    "target=%+dmm v=%+dpx/s" % (
                        clock.fps(),
                        STREAM_TRANSPORT,
                        stream_state.sent_fps,
                        (vision_control.peer[0]
                         if vision_control.peer is not None else "WAIT"),
                        vision_control.sent_frames,
                        vision_control.send_errors,
                        vision_control.received_targets,
                        vision_control.receive_errors,
                        vision_control.target_mm,
                        vision_control.output_velocity_px_s,
                    )
                )

            frame = None
            frame_image = None
            result = None
            if frame_number % GC_PERIOD == 0:
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
        wait_stream_worker(stream_state, stream_thread_started, server)
        close_socket(server)

        if detector is not None:
            detector.deinit()
        detector = None

        if sensor_started:
            sensor.stop()
        if display_initialized:
            Display.deinit()
            time.sleep_ms(50)
        if media_link is not None:
            try:
                media_link.destroy()
            except Exception:
                pass
            media_link = None
        if encoder_started:
            encoder.Stop(VENC_CHANNEL)
        if encoder_created:
            encoder.Destroy(VENC_CHANNEL)
        if media_init_attempted:
            try:
                MediaManager.deinit()
            except Exception as error:
                print("[main] MediaManager cleanup failed:", error)

        gc.collect()
        try:
            nn.shrink_memory_pool()
        except Exception:
            pass
        print("[main] 资源已释放")


if __name__ == "__main__":
    main()
