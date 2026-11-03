"""K230 UDP 图传上位机。

提供实时画面、链路指标和录像功能。录像保存在程序同目录的 Save 文件夹，
停止录制后按结束时间命名为 YYYYMMDD_HHMMSS.mp4。
"""

import argparse
import os
import socket
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtCore import QLockFile, QSize, Qt, QThread, QTimer, QUrl, Signal
from PySide6.QtGui import QCloseEvent, QDesktopServices, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from pc_receiver import (
    DEFAULT_HOST,
    DEFAULT_UDP_PORT,
    MAX_JPEG_SIZE,
    MAX_UDP_FRAGMENTS,
    UDP_DATAGRAM_SIZE,
    UDP_FRAME_MAGIC,
    UDP_FRAME_TIMEOUT_SECONDS,
    UDP_HEADER_STRUCT,
    UDP_HELLO_INTERVAL_SECONDS,
    UDP_HELLO_MAGIC,
    UDP_PAYLOAD_SIZE,
    is_transient_udp_error,
    sequence_gap,
    sequence_is_newer,
)


# 打包后使用 EXE 所在目录，确保录像和日志写在发布包旁边。
if getattr(sys, "frozen", False):
    APP_DIR = Path(sys.executable).resolve().parent
else:
    APP_DIR = Path(__file__).resolve().parent
SAVE_DIR = APP_DIR / "Save"
LOG_DIR = APP_DIR / "Logs"
DEFAULT_RECORD_FPS = 12.5
STREAM_STALE_SECONDS = 2.0
SOCKET_TIMEOUT_SECONDS = 0.15
MAX_VIDEO_DISPLAY_SIZE = QSize(960, 720)
UDP_LOG_PORT = 9011
UDP_LOG_DATAGRAM_SIZE = 1024
UDP_LOG_SOCKET_TIMEOUT_SECONDS = 0.2
UDP_VIDEO_LOCAL_PORT = DEFAULT_UDP_PORT
VALID_TASK_TAGS = frozenset(
    ("T1", "T2", "T3", "T4", "T5", "T6", "TEST")
)
TRANSPORT_AUTO_RECORD_TASKS = frozenset(("T4", "T5", "T6"))
TRANSPORT_AUTO_RECORD_TARGET_STATES = frozenset(("TRACK", "FROZEN"))


def parse_protocol_fields(line, marker):
    """从带协议标记的控制台行中提取空格分隔字段。"""
    marker_index = line.find(marker)
    if marker_index < 0:
        return None

    fields = {}
    payload = line[marker_index + len(marker):]
    for token in payload.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


def parse_task_event(line):
    """从日志行中提取 TASK_BEGIN/TASK_END 事件。"""
    fields = parse_protocol_fields(line, "@TASK ")
    if fields is None:
        return None

    event_type = fields.get("type")
    if event_type == "TASK_BEGIN":
        return "start", fields
    if event_type == "TASK_END":
        return "stop", fields
    return None


def parse_transport_record_start(line):
    """在TASK_BEGIN丢包时，从运输闭环运行日志恢复录像触发。"""
    fields = parse_protocol_fields(line, "@TARGET ")
    if fields is not None:
        task = fields.get("task", "").upper()
        state = fields.get("state", "").upper()
        if (task in TRANSPORT_AUTO_RECORD_TASKS and
                state in TRANSPORT_AUTO_RECORD_TARGET_STATES):
            return fields

    fields = parse_protocol_fields(line, "@CTRL ")
    if fields is None:
        return None
    task = fields.get("task", "").upper()
    stage = fields.get("stage", "").upper()
    try:
        run = int(fields.get("run", "0"))
    except ValueError:
        return None
    if task in TRANSPORT_AUTO_RECORD_TASKS and stage == "TRACK" and run > 0:
        return fields
    return None


def infer_log_task_tag(car_lines, ctrl_lines, event_lines):
    """从当前日志缓冲中推断最近一次任务编号。"""
    marker_groups = (
        (event_lines, ("@TASK ", "@MODE ", "@PARAM ", "@TARGET ")),
        (ctrl_lines, ("@CTRL ", "@TEST ")),
        (car_lines, ("@CAR ",)),
    )
    for lines, markers in marker_groups:
        for line in reversed(lines):
            for marker in markers:
                fields = parse_protocol_fields(line, marker)
                if fields is None:
                    continue
                task = fields.get("task", "").upper()
                if task in VALID_TASK_TAGS:
                    return task
    return "TX"


def parse_prepare_event(line):
    """识别进入任务准备页的MODE事件。"""
    fields = parse_protocol_fields(line, "@MODE ")
    if fields is None:
        return None
    if fields.get("type") != "ENTER" or fields.get("mode") != "PREP":
        return None
    return fields


class UdpLogReceiverThread(QThread):
    """监听车载ESP32广播的控制日志。"""

    line_ready = Signal(int, str)
    state_changed = Signal(str, str)

    def __init__(self, port=UDP_LOG_PORT, parent=None):
        super().__init__(parent)
        self.port = port
        self._last_sequence = 0

    @property
    def last_sequence(self):
        return self._last_sequence

    def run(self):
        final_state = ("stopped", "Wi-Fi日志已停止")
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.port))
            sock.settimeout(UDP_LOG_SOCKET_TIMEOUT_SECONDS)
            self.state_changed.emit(
                "listening", "监听 UDP %d" % self.port
            )
            active_peer = None
            while not self.isInterruptionRequested():
                try:
                    payload, address = sock.recvfrom(UDP_LOG_DATAGRAM_SIZE)
                except socket.timeout:
                    continue
                if address[0] != active_peer:
                    active_peer = address[0]
                    self.state_changed.emit(
                        "connected", "ESP32 %s" % active_peer
                    )
                text = payload.decode("utf-8", errors="replace")
                for line in text.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    self._last_sequence += 1
                    self.line_ready.emit(self._last_sequence, line)
        except OSError as error:
            final_state = ("error", "UDP日志监听失败: %s" % error)
        finally:
            if sock is not None:
                sock.close()
            self.state_changed.emit(*final_state)


class UdpReceiverThread(QThread):
    """独立接收 UDP 数据，主线程只负责显示和录像。"""

    frame_ready = Signal(object)
    metrics_ready = Signal(float, int, float, int, int)
    state_changed = Signal(str, str)
    local_port_ready = Signal(int)

    def __init__(self, host, port, local_port=UDP_VIDEO_LOCAL_PORT,
                 parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.local_port = local_port
        self._state_code = ""
        self._state_text = ""

    def _set_state(self, code, text):
        if code == self._state_code and text == self._state_text:
            return
        self._state_code = code
        self._state_text = text
        self.state_changed.emit(code, text)

    def run(self):
        sock = None
        try:
            target_ip = socket.gethostbyname(self.host)
            target = (target_ip, self.port)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_RCVBUF,
                    1024 * 1024,
                )
            except OSError:
                pass
            # 刷新前后保持相同源端口，K230才能把新心跳识别为原接收端。
            sock.bind(("", self.local_port))
            sock.settimeout(SOCKET_TIMEOUT_SECONDS)
            self.local_port_ready.emit(sock.getsockname()[1])
            self._set_state("waiting", "正在等待 K230 视频流")

            assemblies = {}
            newest_sequence = None
            displayed_sequence = None
            dropped_frames = 0
            last_hello_at = 0.0
            last_frame_at = 0.0
            waiting_message_shown = False

            interval_started_at = time.perf_counter()
            interval_frames = 0
            interval_bytes = 0
            current_width = 0
            current_height = 0

            while not self.isInterruptionRequested():
                now = time.perf_counter()
                if now - last_hello_at >= UDP_HELLO_INTERVAL_SECONDS:
                    try:
                        sock.sendto(UDP_HELLO_MAGIC, target)
                    except OSError as error:
                        if not is_transient_udp_error(error):
                            self._set_state("error", "UDP 心跳发送失败")
                        waiting_message_shown = True
                    last_hello_at = now

                try:
                    datagram, address = sock.recvfrom(
                        UDP_DATAGRAM_SIZE + 64
                    )
                except socket.timeout:
                    self._expire_assembly(
                        assemblies,
                        now,
                    )
                    if (last_frame_at > 0 and
                            now - last_frame_at >= STREAM_STALE_SECONDS):
                        self._set_state("waiting", "视频流暂时中断")
                    self._emit_metrics_if_due(
                        now,
                        interval_started_at,
                        interval_frames,
                        interval_bytes,
                        dropped_frames,
                        current_width,
                        current_height,
                    )
                    if now - interval_started_at >= 1.0:
                        interval_started_at = now
                        interval_frames = 0
                        interval_bytes = 0
                    continue
                except OSError as error:
                    if not is_transient_udp_error(error):
                        self._set_state("error", "UDP 接收异常")
                    waiting_message_shown = True
                    continue

                interval_bytes += len(datagram)
                if waiting_message_shown:
                    waiting_message_shown = False
                    self._set_state("waiting", "K230 UDP 端口已响应")

                if address[0] != target_ip:
                    continue
                parsed = self._parse_datagram(datagram)
                if parsed is None:
                    continue

                (
                    sequence,
                    fragment_index,
                    fragment_count,
                    width,
                    height,
                    jpeg_size,
                    payload,
                ) = parsed

                if newest_sequence is None:
                    newest_sequence = sequence
                elif sequence != newest_sequence:
                    if not sequence_is_newer(sequence, newest_sequence):
                        continue
                    assemblies.clear()
                    newest_sequence = sequence

                assembly = assemblies.get(sequence)
                if assembly is None:
                    assembly = {
                        "chunks": [None] * fragment_count,
                        "received": 0,
                        "jpeg_size": jpeg_size,
                        "width": width,
                        "height": height,
                        "created_at": now,
                    }
                    assemblies[sequence] = assembly
                elif (assembly["jpeg_size"] != jpeg_size or
                      len(assembly["chunks"]) != fragment_count):
                    del assemblies[sequence]
                    continue

                if assembly["chunks"][fragment_index] is None:
                    assembly["chunks"][fragment_index] = payload
                    assembly["received"] += 1
                if assembly["received"] != fragment_count:
                    continue

                jpeg_data = b"".join(assembly["chunks"])
                del assemblies[sequence]
                if len(jpeg_data) != jpeg_size:
                    continue

                encoded = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                if displayed_sequence is not None:
                    dropped_frames += sequence_gap(
                        displayed_sequence,
                        sequence,
                    )
                displayed_sequence = sequence
                current_width = width
                current_height = height
                interval_frames += 1
                last_frame_at = now
                self._set_state("connected", "视频流已连接")
                self.frame_ready.emit(frame)

                if now - interval_started_at >= 1.0:
                    self._emit_metrics_if_due(
                        now,
                        interval_started_at,
                        interval_frames,
                        interval_bytes,
                        dropped_frames,
                        current_width,
                        current_height,
                        force=True,
                    )
                    interval_started_at = now
                    interval_frames = 0
                    interval_bytes = 0

        except Exception as error:
            self._set_state("error", "接收器启动失败: %s" % error)
        finally:
            if sock is not None:
                sock.close()
            self._set_state("stopped", "接收已停止")

    @staticmethod
    def _expire_assembly(assemblies, now):
        if not assemblies:
            return
        sequence, assembly = next(iter(assemblies.items()))
        if now - assembly["created_at"] >= UDP_FRAME_TIMEOUT_SECONDS:
            del assemblies[sequence]

    @staticmethod
    def _parse_datagram(datagram):
        if len(datagram) < UDP_HEADER_STRUCT.size:
            return None
        (
            magic,
            sequence,
            fragment_index,
            fragment_count,
            width,
            height,
            jpeg_size,
        ) = UDP_HEADER_STRUCT.unpack_from(datagram)
        if magic != UDP_FRAME_MAGIC:
            return None
        if jpeg_size <= 0 or jpeg_size > MAX_JPEG_SIZE:
            return None

        expected_count = (
            jpeg_size + UDP_PAYLOAD_SIZE - 1
        ) // UDP_PAYLOAD_SIZE
        if (fragment_count != expected_count or
                fragment_count <= 0 or
                fragment_count > MAX_UDP_FRAGMENTS or
                fragment_index >= fragment_count):
            return None

        payload = datagram[UDP_HEADER_STRUCT.size:]
        expected_payload_size = UDP_PAYLOAD_SIZE
        if fragment_index == fragment_count - 1:
            expected_payload_size = (
                jpeg_size - UDP_PAYLOAD_SIZE * (fragment_count - 1)
            )
        if len(payload) != expected_payload_size:
            return None
        return (
            sequence,
            fragment_index,
            fragment_count,
            width,
            height,
            jpeg_size,
            payload,
        )

    def _emit_metrics_if_due(
            self, now, started_at, frames, received_bytes,
            dropped_frames, width, height, force=False):
        elapsed = now - started_at
        if elapsed < 1.0 and not force:
            return
        if elapsed <= 0:
            return
        fps = frames / elapsed
        mbps = received_bytes * 8.0 / elapsed / 1000000.0
        self.metrics_ready.emit(
            fps,
            dropped_frames,
            mbps,
            width,
            height,
        )


class VideoRecorder:
    """使用临时文件录制，停止时按结束时间生成最终文件名。"""

    def __init__(self, save_dir=SAVE_DIR):
        self.save_dir = Path(save_dir)
        self.writer = None
        self.temp_path = None
        self.frame_size = None
        self.frame_count = 0
        self.started_at = None
        self.fps = DEFAULT_RECORD_FPS
        self.last_frame = None
        self.file_prefix = None

    @property
    def active(self):
        return self.writer is not None

    def start(self, width, height, fps, file_prefix=None):
        if self.active:
            return
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.frame_size = (int(width), int(height))
        self.fps = max(5.0, min(30.0, float(fps)))
        normalized_prefix = str(file_prefix or "").upper()
        self.file_prefix = (
            normalized_prefix
            if normalized_prefix in ("T1", "T2", "T3", "T4", "T5", "T6")
            else None
        )
        self.started_at = time.perf_counter()
        self.frame_count = 0
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.temp_path = self.save_dir / (".recording_%s.mp4" % stamp)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(
            str(self.temp_path),
            fourcc,
            self.fps,
            self.frame_size,
        )
        if not self.writer.isOpened():
            self.writer.release()
            self.writer = None
            self._reset()
            raise RuntimeError("无法创建 MP4 录像文件")

    def write(self, frame):
        if not self.active:
            return
        if (frame.shape[1], frame.shape[0]) != self.frame_size:
            frame = cv2.resize(frame, self.frame_size)
        now = time.perf_counter()
        if self.last_frame is None:
            self._write_raw(frame)
            self.last_frame = frame.copy()
            return

        # MP4是固定帧率容器。按墙钟时间补写上一帧，避免网络低帧率
        # 或接收器重启把一段真实任务时间压缩成几秒回放。
        target_count = max(
            1,
            int((now - self.started_at) * self.fps) + 1,
        )
        while self.frame_count < target_count - 1:
            self._write_raw(self.last_frame)
        if self.frame_count < target_count:
            self._write_raw(frame)
        self.last_frame = frame.copy()

    def stop(self, ended_at=None):
        if not self.active:
            return None
        if self.last_frame is not None and self.started_at is not None:
            elapsed = max(0.0, time.perf_counter() - self.started_at)
            target_count = max(1, int(round(elapsed * self.fps)))
            while self.frame_count < target_count:
                self._write_raw(self.last_frame)
        self.writer.release()
        self.writer = None
        ended_at = ended_at or datetime.now()

        if self.frame_count <= 0:
            self._remove_temp_file()
            self._reset()
            return None

        base_name = ended_at.strftime("%Y%m%d_%H%M%S")
        if self.file_prefix is not None:
            base_name = "%s_%s" % (self.file_prefix, base_name)
        target = self.save_dir / (base_name + ".mp4")
        suffix = 1
        while target.exists():
            target = self.save_dir / (base_name + "_%d.mp4" % suffix)
            suffix += 1
        temp_path = self.temp_path
        try:
            os.replace(str(temp_path), str(target))
        except OSError as error:
            self._reset()
            raise RuntimeError(
                "录像已写入临时文件，但重命名失败：%s\n%s"
                % (temp_path, error)
            ) from error
        self._reset()
        return target

    def elapsed_seconds(self):
        if not self.active or self.started_at is None:
            return 0.0
        return time.perf_counter() - self.started_at

    def _write_raw(self, frame):
        self.writer.write(frame)
        self.frame_count += 1

    def _remove_temp_file(self):
        if self.temp_path is None:
            return
        try:
            self.temp_path.unlink()
        except OSError:
            pass

    def _reset(self):
        self.temp_path = None
        self.frame_size = None
        self.frame_count = 0
        self.started_at = None
        self.last_frame = None
        self.fps = DEFAULT_RECORD_FPS
        self.file_prefix = None


class MotorLogWindow(QMainWindow):
    """独立显示巡线、滚球控制和任务事件日志。"""

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Window)
        self.car_count = 0
        self.ctrl_count = 0
        self.event_count = 0

        self.setWindowTitle("电机监控")
        self.setMinimumSize(1000, 500)
        self.resize(1280, 640)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        toolbar = QFrame()
        toolbar.setObjectName("LogToolbar")
        toolbar_layout = QVBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(14, 12, 14, 12)
        toolbar_layout.setSpacing(10)

        title_row = QHBoxLayout()
        title = QLabel("电机监控")
        title.setObjectName("AppTitle")
        title_row.addWidget(title)
        title_row.addStretch()
        self.udp_state_value = QLabel("监听 UDP %d" % UDP_LOG_PORT)
        self.udp_state_value.setObjectName("HeaderState")
        title_row.addWidget(self.udp_state_value)
        toolbar_layout.addLayout(title_row)

        controls = QHBoxLayout()
        controls.setSpacing(8)
        source = QLabel("UDP %d" % UDP_LOG_PORT)
        source.setObjectName("LogMarker")
        controls.addWidget(source)
        controls.addStretch()

        self.clear_log_button = QPushButton("清理")
        self.clear_log_button.setObjectName("CompactButton")
        self.clear_log_button.setIcon(
            self.style().standardIcon(
                QStyle.StandardPixmap.SP_DialogResetButton
            )
        )
        controls.addWidget(self.clear_log_button)

        self.save_log_button = QPushButton("保存")
        self.save_log_button.setObjectName("CompactButton")
        self.save_log_button.setIcon(
            self.style().standardIcon(
                QStyle.StandardPixmap.SP_DialogSaveButton
            )
        )
        controls.addWidget(self.save_log_button)
        toolbar_layout.addLayout(controls)
        root.addWidget(toolbar)

        logs = QHBoxLayout()
        logs.setSpacing(12)
        car_panel, self.car_log_view, self.car_count_value = (
            self._create_log_panel("底盘巡线", "@CAR", "CarLogView")
        )
        ctrl_panel, self.ctrl_log_view, self.ctrl_count_value = (
            self._create_log_panel(
                "滚球控制", "@CTRL/@TEST", "CtrlLogView"
            )
        )
        event_panel, self.event_log_view, self.event_count_value = (
            self._create_log_panel("任务事件", "@TASK/@MODE", "EventLogView")
        )
        logs.addWidget(car_panel, 1)
        logs.addWidget(ctrl_panel, 1)
        logs.addWidget(event_panel, 1)
        root.addLayout(logs, 1)

        self.setCentralWidget(central)
        status_bar = QStatusBar()
        status_bar.showMessage("等待ESP32 Wi-Fi日志")
        self.setStatusBar(status_bar)

    def _create_log_panel(self, title, marker, object_name):
        panel = QFrame()
        panel.setObjectName("LogPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        name = QLabel(title)
        name.setObjectName("SectionTitle")
        header.addWidget(name)
        marker_label = QLabel(marker)
        marker_label.setObjectName("LogMarker")
        header.addWidget(marker_label)
        header.addStretch()
        count = QLabel("0 条")
        count.setObjectName("LogCount")
        header.addWidget(count)
        layout.addLayout(header)

        view = QPlainTextEdit()
        view.setObjectName(object_name)
        view.setReadOnly(True)
        view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        view.setPlaceholderText("等待%s日志" % marker)
        layout.addWidget(view, 1)
        return panel, view, count

    @staticmethod
    def _append_and_scroll(view, line):
        view.appendPlainText(line)
        scroll_bar = view.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def append_car(self, line):
        self.car_count += 1
        self._append_and_scroll(self.car_log_view, line)
        self.car_count_value.setText("%d 条" % self.car_count)

    def append_ctrl(self, line):
        self.ctrl_count += 1
        self._append_and_scroll(self.ctrl_log_view, line)
        self.ctrl_count_value.setText("%d 条" % self.ctrl_count)

    def append_event(self, line):
        self.event_count += 1
        self._append_and_scroll(self.event_log_view, line)
        self.event_count_value.setText("%d 条" % self.event_count)

    def clear_views(self):
        self.car_count = 0
        self.ctrl_count = 0
        self.event_count = 0
        self.car_log_view.clear()
        self.ctrl_log_view.clear()
        self.event_log_view.clear()
        self.car_count_value.setText("0 条")
        self.ctrl_count_value.setText("0 条")
        self.event_count_value.setText("0 条")


class MainWindow(QMainWindow):
    """实时图传和录像主窗口。"""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_UDP_PORT,
                 auto_start=True):
        super().__init__()
        self.receiver = None
        self.udp_log_reader = None
        self.latest_frame = None
        self.latest_image = None
        self.current_fps = 0.0
        self.current_resolution = (0, 0)
        self.video_connected = False
        self.recorder = VideoRecorder()
        self.car_log_lines = []
        self.ctrl_log_lines = []
        self.event_log_lines = []
        self.udp_log_clear_sequence = 0
        self.pending_auto_record = False
        self.pending_auto_record_task = None
        self.receiver_restart_pending = False
        self.closing = False
        self.last_prepare_event_key = None

        self.motor_log_window = MotorLogWindow(self)
        self.clear_log_button = self.motor_log_window.clear_log_button
        self.save_log_button = self.motor_log_window.save_log_button
        self.clear_log_button.clicked.connect(self.clear_logs)
        self.save_log_button.clicked.connect(self.save_logs)

        self.setWindowTitle("K230 钢珠检测上位机")
        self.setMinimumSize(1100, 720)
        self.resize(1440, 900)
        self._build_ui(host, port)
        self._apply_style()

        self.record_timer = QTimer(self)
        self.record_timer.setInterval(250)
        self.record_timer.timeout.connect(self._update_record_duration)
        self.record_timer.start()

        if auto_start:
            QTimer.singleShot(0, self.start_udp_log_receiver)
            QTimer.singleShot(0, self.start_receiver)

    def _build_ui(self, host, port):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header = QFrame()
        header.setObjectName("Header")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(22, 14, 22, 14)
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("K230 钢珠检测上位机")
        title.setObjectName("AppTitle")
        subtitle = QLabel("UDP JPEG 图传 · 任务录像 · 电机监控")
        subtitle.setObjectName("AppSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_layout.addLayout(title_box)
        header_layout.addStretch()
        self.header_state = QLabel("等待连接")
        self.header_state.setObjectName("HeaderState")
        header_layout.addWidget(self.header_state)
        root.addWidget(header)

        content = QHBoxLayout()
        content.setContentsMargins(18, 18, 18, 18)
        content.setSpacing(18)

        left_stack = QWidget()
        left_layout = QVBoxLayout(left_stack)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        video_panel = QFrame()
        video_panel.setObjectName("VideoPanel")
        video_layout = QVBoxLayout(video_panel)
        video_layout.setContentsMargins(1, 1, 1, 1)
        video_layout.setSpacing(0)
        self.video_label = QLabel("等待 K230 视频流")
        self.video_label.setObjectName("VideoLabel")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(640, 220)
        self.video_label.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        video_layout.addWidget(self.video_label)
        left_layout.addWidget(video_panel, 1)
        content.addWidget(left_stack, 1)

        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        sidebar.setMinimumWidth(292)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(18, 18, 18, 18)
        side.setSpacing(12)

        side.addWidget(self._section_title("连接"))
        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(10)
        self.host_input = QLineEdit(host)
        self.host_input.setPlaceholderText("K230 IP")
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(port)
        form.addRow("地址", self.host_input)
        form.addRow("端口", self.port_input)
        side.addLayout(form)

        self.receive_button = QPushButton("停止接收")
        self.receive_button.setObjectName("PrimaryButton")
        self.receive_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
        )
        self.receive_button.setIconSize(QSize(18, 18))
        self.receive_button.clicked.connect(self.toggle_receiver)
        side.addWidget(self.receive_button)

        side.addWidget(self._separator())
        side.addWidget(self._section_title("电机"))
        self.motor_log_button = QPushButton("打开电机监控")
        self.motor_log_button.setObjectName("MotorButton")
        self.motor_log_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_ComputerIcon)
        )
        self.motor_log_button.setIconSize(QSize(18, 18))
        self.motor_log_button.clicked.connect(self.show_motor_log_window)
        side.addWidget(self.motor_log_button)

        side.addWidget(self._separator())
        side.addWidget(self._section_title("链路状态"))
        self.connection_value = QLabel("初始化")
        self.local_port_value = QLabel("-")
        self.resolution_value = QLabel("-")
        self.fps_value = QLabel("0.0 FPS")
        self.bandwidth_value = QLabel("0.00 Mbps")
        self.drop_value = QLabel("0")
        side.addLayout(self._status_row("状态", self.connection_value))
        side.addLayout(self._status_row("本地端口", self.local_port_value))
        side.addLayout(self._status_row("分辨率", self.resolution_value))
        side.addLayout(self._status_row("接收帧率", self.fps_value))
        side.addLayout(self._status_row("带宽", self.bandwidth_value))
        side.addLayout(self._status_row("丢帧", self.drop_value))

        side.addWidget(self._separator())
        side.addWidget(self._section_title("录像"))
        self.record_button = QPushButton("开始录制")
        self.record_button.setObjectName("RecordButton")
        self.record_button.setEnabled(False)
        self.record_button.setIcon(
            self.style().standardIcon(
                QStyle.StandardPixmap.SP_DialogSaveButton
            )
        )
        self.record_button.setIconSize(QSize(18, 18))
        self.record_button.clicked.connect(self.toggle_recording)
        side.addWidget(self.record_button)

        self.record_state_value = QLabel("未录制")
        self.record_duration_value = QLabel("00:00:00")
        self.last_file_value = QLabel("-")
        self.last_file_value.setWordWrap(True)
        side.addLayout(
            self._status_row("录制状态", self.record_state_value)
        )
        side.addLayout(
            self._status_row("录制时长", self.record_duration_value)
        )
        side.addWidget(QLabel("最近文件"))
        side.addWidget(self.last_file_value)

        self.open_folder_button = QPushButton("打开 Save 文件夹")
        self.open_folder_button.setIcon(
            self.style().standardIcon(
                QStyle.StandardPixmap.SP_DirOpenIcon
            )
        )
        self.open_folder_button.setIconSize(QSize(18, 18))
        self.open_folder_button.clicked.connect(self.open_save_folder)
        side.addWidget(self.open_folder_button)
        side.addStretch()

        sidebar_scroll = QScrollArea()
        sidebar_scroll.setObjectName("SidebarScroll")
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        sidebar_scroll.setFrameShape(QFrame.Shape.NoFrame)
        sidebar_scroll.setFixedWidth(316)
        sidebar_scroll.setWidget(sidebar)
        content.addWidget(sidebar_scroll)
        root.addLayout(content, 1)
        self.setCentralWidget(central)

        status_bar = QStatusBar()
        status_bar.showMessage("就绪")
        self.setStatusBar(status_bar)

    @staticmethod
    def _section_title(text):
        label = QLabel(text)
        label.setObjectName("SectionTitle")
        return label

    @staticmethod
    def _separator():
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setObjectName("Separator")
        return line

    @staticmethod
    def _status_row(name, value):
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        name_label = QLabel(name)
        name_label.setObjectName("MetricName")
        value.setObjectName("MetricValue")
        value.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        layout.addWidget(name_label)
        layout.addStretch()
        layout.addWidget(value)
        return layout

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #14161a;
                color: #e8eaed;
                font-family: "Microsoft YaHei UI";
                font-size: 13px;
            }
            QLabel { background: transparent; }
            QFrame#Header {
                background: #1d2025;
                border-bottom: 1px solid #30343b;
            }
            QLabel#AppTitle { font-size: 20px; font-weight: 600; }
            QLabel#AppSubtitle { color: #9298a2; font-size: 12px; }
            QLabel#HeaderState {
                color: #d1d5db;
                background: #292d34;
                border: 1px solid #3a3f48;
                border-radius: 4px;
                padding: 6px 10px;
            }
            QFrame#VideoPanel {
                background: #070809;
                border: 1px solid #30343b;
                border-radius: 4px;
            }
            QLabel#VideoLabel { background: #070809; color: #737984; }
            QFrame#LogPanel {
                background: #111318;
                border: 1px solid #30343b;
                border-radius: 4px;
            }
            QFrame#LogToolbar {
                background: #1b1e23;
                border: 1px solid #30343b;
                border-radius: 4px;
            }
            QLabel#LogCount { color: #737984; font-size: 12px; }
            QLabel#LogMarker {
                color: #9ca3af;
                background: #252930;
                border: 1px solid #3a3f48;
                border-radius: 3px;
                padding: 2px 6px;
                font-family: "Cascadia Mono", "Consolas";
                font-size: 11px;
            }
            QPlainTextEdit#CarLogView,
            QPlainTextEdit#CtrlLogView,
            QPlainTextEdit#EventLogView {
                background: #080a0d;
                color: #d7dae0;
                border: 1px solid #292d34;
                border-radius: 3px;
                padding: 8px;
                font-family: "Cascadia Mono", "Consolas";
                font-size: 12px;
                selection-background-color: #1d4ed8;
            }
            QPlainTextEdit#CarLogView { color: #bbf7d0; }
            QPlainTextEdit#CtrlLogView { color: #bfdbfe; }
            QPlainTextEdit#EventLogView { color: #fde68a; }
            QFrame#Sidebar {
                background: #1b1e23;
                border: 1px solid #30343b;
                border-radius: 4px;
            }
            QScrollArea#SidebarScroll { background: transparent; border: none; }
            QLabel#SectionTitle { font-size: 14px; font-weight: 600; }
            QLabel#MetricName { color: #9298a2; }
            QLabel#MetricValue { color: #f3f4f6; font-weight: 500; }
            QFrame#Separator { color: #30343b; }
            QLineEdit, QSpinBox {
                background: #111318;
                border: 1px solid #3a3f48;
                border-radius: 4px;
                padding: 7px 8px;
                min-height: 20px;
            }
            QLineEdit:focus, QSpinBox:focus {
                border-color: #3b82f6;
            }
            QPushButton {
                background: #292d34;
                border: 1px solid #3a3f48;
                border-radius: 4px;
                padding: 8px 12px;
                min-height: 22px;
            }
            QPushButton:hover { background: #333842; }
            QPushButton:pressed { background: #22262c; }
            QPushButton:disabled { color: #686e78; background: #202329; }
            QPushButton#PrimaryButton {
                background: #2563eb;
                border-color: #3b82f6;
                color: white;
            }
            QPushButton#PrimaryButton:hover { background: #1d4ed8; }
            QPushButton#MotorButton {
                background: #24333f;
                border-color: #36566e;
                color: #dbeafe;
            }
            QPushButton#MotorButton:hover { background: #2c4253; }
            QPushButton#CompactButton {
                padding: 5px 9px;
                min-height: 18px;
            }
            QPushButton#RecordButton {
                background: #3a2225;
                border-color: #6f3037;
                color: #fecaca;
            }
            QPushButton#RecordButton:hover { background: #51282d; }
            QStatusBar {
                background: #1d2025;
                border-top: 1px solid #30343b;
                color: #9298a2;
            }
            QScrollBar:vertical {
                background: #101216;
                width: 10px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #3a3f48;
                min-height: 28px;
                border-radius: 4px;
            }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical { height: 0; }
            """
        )

    def show_motor_log_window(self):
        self.motor_log_window.show()
        self.motor_log_window.raise_()
        self.motor_log_window.activateWindow()

    def start_udp_log_receiver(self):
        if (self.udp_log_reader is not None and
                self.udp_log_reader.isRunning()):
            return
        self.udp_log_reader = UdpLogReceiverThread(UDP_LOG_PORT, self)
        self.udp_log_reader.line_ready.connect(self._on_udp_log_line)
        self.udp_log_reader.state_changed.connect(self._on_udp_log_state)
        self.udp_log_reader.finished.connect(self._on_udp_log_finished)
        self.udp_log_reader.start()

    def _on_udp_log_state(self, code, text):
        self.motor_log_window.udp_state_value.setText(text)
        colors = {
            "listening": "#f59e0b",
            "connected": "#22c55e",
            "error": "#ef4444",
            "stopped": "#9298a2",
        }
        self.motor_log_window.udp_state_value.setStyleSheet(
            "color: %s;" % colors.get(code, "#9298a2")
        )
        self.motor_log_window.statusBar().showMessage(text)
        self.statusBar().showMessage(text)

    def _on_udp_log_finished(self):
        if self.udp_log_reader is not None:
            self.udp_log_reader.deleteLater()
        self.udp_log_reader = None

    def _on_udp_log_line(self, sequence, line):
        if sequence <= self.udp_log_clear_sequence:
            return

        if "@CAR " in line:
            self.car_log_lines.append(line)
            self.motor_log_window.append_car(line)
        elif "@CTRL " in line or "@TEST " in line:
            self.ctrl_log_lines.append(line)
            self.motor_log_window.append_ctrl(line)
        elif "@" in line:
            self.event_log_lines.append(line)
            self.motor_log_window.append_event(line)

        task_event = parse_task_event(line)
        if task_event is not None:
            action, fields = task_event
            self._handle_task_event(action, fields)
        elif not self.recorder.active and not self.pending_auto_record:
            transport_start = parse_transport_record_start(line)
            if transport_start is not None:
                self._handle_task_event("start", transport_start)

        prepare_event = parse_prepare_event(line)
        if prepare_event is not None:
            self._handle_prepare_event(prepare_event)

    def clear_logs(self):
        if self.udp_log_reader is not None:
            self.udp_log_clear_sequence = max(
                self.udp_log_clear_sequence,
                self.udp_log_reader.last_sequence,
            )
        self.car_log_lines.clear()
        self.ctrl_log_lines.clear()
        self.event_log_lines.clear()
        self.motor_log_window.clear_views()
        self.motor_log_window.statusBar().showMessage("当前日志已清理")
        self.statusBar().showMessage("当前日志已清理")

    def save_logs(self):
        if not any((self.car_log_lines,
                    self.ctrl_log_lines,
                    self.event_log_lines)):
            QMessageBox.information(
                self.motor_log_window,
                "没有日志",
                "当前没有未清理的Wi-Fi日志可保存。",
            )
            return

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        task_tag = infer_log_task_tag(
            self.car_log_lines,
            self.ctrl_log_lines,
            self.event_log_lines,
        )
        groups = (
            ("CAR", self.car_log_lines),
            ("CTRL", self.ctrl_log_lines),
            ("EVENT", self.event_log_lines),
        )
        paths = [
            LOG_DIR / ("%s_%s_%s.log" % (timestamp, task_tag, name))
            for name, _ in groups
        ]
        try:
            for path, (_, lines) in zip(paths, groups):
                content = "\n".join(lines)
                if content:
                    content += "\n"
                path.write_text(content, encoding="utf-8")
        except OSError as error:
            QMessageBox.critical(
                self.motor_log_window, "保存日志失败", str(error)
            )
            return
        file_pattern = "%s_%s_{CAR,CTRL,EVENT}.log" % (
            timestamp, task_tag
        )
        self.motor_log_window.statusBar().showMessage(
            "三类日志已保存: %s" % file_pattern
        )
        self.statusBar().showMessage(
            "三类日志已保存到 Logs: %s" % file_pattern
        )

    def _handle_task_event(self, action, fields):
        run = fields.get("run", "-")
        task = fields.get("task", "UNKNOWN")
        if action == "start":
            if self.recorder.active:
                self.statusBar().showMessage(
                    "任务 %s run=%s 已开始，录像正在进行" % (task, run)
                )
                return
            self.pending_auto_record = True
            self.pending_auto_record_task = task
            if self.latest_frame is not None:
                self.start_recording(automatic=True, task=task)
            else:
                self.record_state_value.setText("等待画面自动录制")
                self.record_state_value.setStyleSheet("color: #f59e0b;")
                self.statusBar().showMessage(
                    "任务 %s run=%s 已开始，等待视频帧" % (task, run)
                )
            return

        self.pending_auto_record = False
        self.pending_auto_record_task = None
        if self.recorder.active:
            self.stop_recording()
        else:
            self.record_state_value.setText("未录制")
            self.record_state_value.setStyleSheet("")
            self.statusBar().showMessage(
                "任务 %s run=%s 已结束" % (task, run)
            )

    def _handle_prepare_event(self, fields):
        event_key = (
            fields.get("eventSeq", ""),
            fields.get("type", ""),
            fields.get("mode", ""),
            fields.get("task", ""),
            fields.get("t", ""),
        )
        if event_key == self.last_prepare_event_key:
            return
        if (self.latest_frame is None or self.receiver is None or
                not self.receiver.isRunning()):
            return

        self.last_prepare_event_key = event_key
        self.receiver_restart_pending = True
        self.receive_button.setEnabled(False)
        self.receive_button.setText("正在刷新")
        self.receiver.requestInterruption()

        task = fields.get("task", "UNKNOWN")
        message = "任务 %s 进入准备页，正在刷新UDP视频接收" % task
        self.motor_log_window.statusBar().showMessage(message)
        self.statusBar().showMessage(message)

    def start_receiver(self):
        if self.receiver is not None and self.receiver.isRunning():
            return
        host = self.host_input.text().strip()
        if not host:
            QMessageBox.warning(self, "地址无效", "请输入 K230 地址。")
            return
        port = self.port_input.value()
        self.host_input.setEnabled(False)
        self.port_input.setEnabled(False)
        self.record_button.setEnabled(False)
        self.local_port_value.setText("-")
        self.current_fps = 0.0
        self.fps_value.setText("0.0 FPS")
        self.bandwidth_value.setText("0.00 Mbps")
        self.receive_button.setEnabled(True)
        self.receive_button.setText("停止接收")
        self.receive_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
        )
        self.receiver = UdpReceiverThread(
            host,
            port,
            local_port=UDP_VIDEO_LOCAL_PORT,
            parent=self,
        )
        self.receiver.frame_ready.connect(self._on_frame)
        self.receiver.metrics_ready.connect(self._on_metrics)
        self.receiver.state_changed.connect(self._on_state)
        self.receiver.local_port_ready.connect(self._on_local_port)
        self.receiver.finished.connect(self._on_receiver_finished)
        self.receiver.start()
        self.statusBar().showMessage("正在启动 UDP 接收器")

    def stop_receiver(self):
        self.receiver_restart_pending = False
        if self.recorder.active:
            self.stop_recording()
        if self.receiver is None or not self.receiver.isRunning():
            return
        self.receive_button.setEnabled(False)
        self.receive_button.setText("正在停止")
        self.receiver.requestInterruption()

    def toggle_receiver(self):
        if self.receiver is not None and self.receiver.isRunning():
            self.stop_receiver()
        else:
            self.start_receiver()

    def _on_receiver_finished(self):
        restart_requested = self.receiver_restart_pending and not self.closing
        self.receiver_restart_pending = False
        if self.receiver is not None:
            self.receiver.deleteLater()
        self.receiver = None
        self.host_input.setEnabled(True)
        self.port_input.setEnabled(True)
        self.receive_button.setEnabled(True)
        self.record_button.setEnabled(False)
        if restart_requested:
            self.receive_button.setText("正在刷新")
            self.statusBar().showMessage("UDP视频接收器正在重新启动")
            QTimer.singleShot(0, self.start_receiver)
        else:
            self.receive_button.setText("开始接收")
            self.receive_button.setIcon(
                self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay)
            )
            self.statusBar().showMessage("接收器已停止")

    def _on_state(self, code, text):
        self.video_connected = code == "connected"
        self.connection_value.setText(text)
        self.header_state.setText(text)
        colors = {
            "connected": "#22c55e",
            "waiting": "#f59e0b",
            "error": "#ef4444",
            "stopped": "#9298a2",
        }
        color = colors.get(code, "#9298a2")
        self.header_state.setStyleSheet("color: %s;" % color)
        if code == "connected" and self.latest_frame is not None:
            self.record_button.setEnabled(True)
        elif code in ("waiting", "error", "stopped") and not self.recorder.active:
            self.record_button.setEnabled(False)
        self.statusBar().showMessage(text)

    def _on_local_port(self, port):
        self.local_port_value.setText(str(port))

    def _on_metrics(self, fps, dropped, mbps, width, height):
        self.current_fps = fps
        self.fps_value.setText("%.1f FPS" % fps)
        self.bandwidth_value.setText("%.2f Mbps" % mbps)
        self.drop_value.setText(str(dropped))
        if width > 0 and height > 0:
            self.current_resolution = (width, height)
            self.resolution_value.setText("%d x %d" % (width, height))

    def _on_frame(self, frame):
        self.latest_frame = frame
        self.video_connected = True
        if self.current_resolution == (0, 0):
            self.current_resolution = (frame.shape[1], frame.shape[0])
            self.resolution_value.setText(
                "%d x %d" % self.current_resolution
            )
        if self.pending_auto_record and not self.recorder.active:
            self.start_recording(
                automatic=True,
                task=self.pending_auto_record_task,
            )
        if self.recorder.active:
            try:
                self.recorder.write(frame)
            except Exception as error:
                self.stop_recording()
                QMessageBox.critical(self, "录像失败", str(error))

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = QImage(
            rgb.data,
            rgb.shape[1],
            rgb.shape[0],
            rgb.strides[0],
            QImage.Format.Format_RGB888,
        ).copy()
        self.latest_image = image
        self._render_latest_image()
        if not self.recorder.active:
            self.record_button.setEnabled(True)

    def _render_latest_image(self):
        if self.latest_image is None:
            return
        pixmap = QPixmap.fromImage(self.latest_image)
        display_size = self.video_label.size().boundedTo(
            MAX_VIDEO_DISPLAY_SIZE
        )
        pixmap = pixmap.scaled(
            display_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )
        self.video_label.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_latest_image()

    def toggle_recording(self):
        if self.recorder.active:
            self.stop_recording()
        else:
            self.pending_auto_record = False
            self.pending_auto_record_task = None
            self.start_recording()

    def start_recording(self, automatic=False, task=None):
        if self.latest_frame is None:
            if automatic:
                self.pending_auto_record = True
                self.pending_auto_record_task = task
                self.record_state_value.setText("等待画面自动录制")
                self.record_state_value.setStyleSheet("color: #f59e0b;")
            else:
                QMessageBox.information(
                    self, "暂无画面", "收到视频后才能录制。"
                )
            return
        try:
            self.recorder.start(
                self.latest_frame.shape[1],
                self.latest_frame.shape[0],
                DEFAULT_RECORD_FPS,
                file_prefix=task if automatic else None,
            )
            self.recorder.write(self.latest_frame)
        except Exception as error:
            QMessageBox.critical(self, "无法开始录制", str(error))
            return
        self.pending_auto_record = False
        self.pending_auto_record_task = None
        self.record_button.setText("停止录制")
        self.record_state_value.setText("正在录制")
        self.record_state_value.setStyleSheet("color: #ef4444;")
        self.statusBar().showMessage(
            "%s录制，%.1f FPS"
            % ("任务触发自动" if automatic else "正在",
               self.recorder.fps)
        )

    def stop_recording(self):
        self.pending_auto_record = False
        self.pending_auto_record_task = None
        save_failed = False
        try:
            path = self.recorder.stop()
        except Exception as error:
            QMessageBox.critical(self, "保存录像失败", str(error))
            path = None
            save_failed = True
        self.record_button.setText("开始录制")
        self.record_state_value.setText("未录制")
        self.record_state_value.setStyleSheet("")
        self.record_duration_value.setText("00:00:00")
        if path is not None:
            self.last_file_value.setText(path.name)
            self.last_file_value.setToolTip(str(path))
            self.statusBar().showMessage("录像已保存: %s" % path.name)
        elif save_failed:
            self.statusBar().showMessage("保存失败，临时录像文件已保留")
        else:
            self.statusBar().showMessage("录像已停止，没有可保存的帧")

    def _update_record_duration(self):
        seconds = int(self.recorder.elapsed_seconds())
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        self.record_duration_value.setText(
            "%02d:%02d:%02d" % (hours, minutes, secs)
        )

    def open_save_folder(self):
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(SAVE_DIR)))

    def closeEvent(self, event: QCloseEvent):
        self.closing = True
        self.receiver_restart_pending = False
        if self.recorder.active:
            self.stop_recording()
        if (self.udp_log_reader is not None and
                self.udp_log_reader.isRunning()):
            self.udp_log_reader.requestInterruption()
            if not self.udp_log_reader.wait(1500):
                self.closing = False
                event.ignore()
                QMessageBox.warning(
                    self,
                    "Wi-Fi日志仍在停止",
                    "UDP日志接收器尚未退出，请稍后再次关闭。",
                )
                return
        if self.receiver is not None and self.receiver.isRunning():
            self.receiver.requestInterruption()
            if not self.receiver.wait(3000):
                self.closing = False
                event.ignore()
                QMessageBox.warning(
                    self,
                    "接收器仍在停止",
                    "请稍后再次关闭窗口。",
                )
                return
        self.motor_log_window.hide()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(description="K230 UDP 图传上位机")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT)
    return parser.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv[:1])
    app.setApplicationName("K230VideoReceiver")
    app.setStyle("Fusion")

    lock_path = Path(tempfile.gettempdir()) / "K230VideoReceiverGUI.lock"
    lock = QLockFile(str(lock_path))
    lock.setStaleLockTime(10000)
    if not lock.tryLock(100):
        QMessageBox.warning(
            None,
            "程序已经运行",
            "只能运行一个 K230 图传上位机。",
        )
        return 1

    window = MainWindow(args.host, args.port)
    window.show()
    result = app.exec()
    lock.unlock()
    return result


if __name__ == "__main__":
    sys.exit(main())
