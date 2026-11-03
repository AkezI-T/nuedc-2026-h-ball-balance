"""K230 JPEG/TCP、JPEG/UDP 实时视频接收器。

只负责按帧接收、JPEG解码和显示，不包含录制功能。
"""

import argparse
import socket
import struct
import sys
import time


DEFAULT_HOST = "10.43.133.238"
DEFAULT_TCP_PORT = 9000
DEFAULT_UDP_PORT = 9001
WINDOW_NAME = "K230 实时视频"

FRAME_MAGIC = b"KJPG"
HEADER_STRUCT = struct.Struct("!4sIHHI")
UDP_HELLO_MAGIC = b"KHEL"
UDP_FRAME_MAGIC = b"KUJP"
UDP_HEADER_STRUCT = struct.Struct("!4sIHHHHI")
UDP_DATAGRAM_SIZE = 1200
UDP_PAYLOAD_SIZE = UDP_DATAGRAM_SIZE - UDP_HEADER_STRUCT.size
MAX_UDP_FRAGMENTS = 512
UDP_HELLO_INTERVAL_SECONDS = 1.0
UDP_FRAME_TIMEOUT_SECONDS = 0.5
UDP_TRANSIENT_ERROR_CODES = (10051, 10054, 10060, 10061, 10065)
MAX_JPEG_SIZE = 2 * 1024 * 1024
CONNECT_TIMEOUT_SECONDS = 3.0
READ_TIMEOUT_SECONDS = 5.0
RECONNECT_DELAY_SECONDS = 1.0


def parse_args():
    parser = argparse.ArgumentParser(description="接收K230 JPEG实时视频流")
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help="K230地址，默认值：%(default)s",
    )
    parser.add_argument(
        "--transport",
        choices=("tcp", "udp"),
        default="tcp",
        help="传输协议，默认值：%(default)s",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="端口；TCP默认9000，UDP默认9001",
    )
    return parser.parse_args()


def load_opencv():
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[receiver] 缺少OpenCV或NumPy，请运行start_receiver.bat")
        raise
    return cv2, np


def verify_opencv_gui(cv2):
    """确认当前OpenCV包含Windows窗口后端。"""
    gui_backend = "unknown"
    for line in cv2.getBuildInformation().splitlines():
        stripped = line.strip()
        if stripped.startswith("GUI:"):
            gui_backend = stripped.split(":", 1)[1].strip()
            break

    print("[receiver] Python:", sys.executable)
    print("[receiver] OpenCV %s, GUI=%s" % (cv2.__version__, gui_backend))
    if gui_backend.upper() in ("NONE", "NO"):
        raise RuntimeError("当前OpenCV不支持窗口显示，请运行start_receiver.bat")


def connect_stream(host, port):
    sock = socket.create_connection(
        (host, port),
        timeout=CONNECT_TIMEOUT_SECONDS,
    )
    sock.settimeout(READ_TIMEOUT_SECONDS)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return sock


def recv_exact(sock, size):
    """从TCP字节流中精确读取指定长度。"""
    data = bytearray(size)
    view = memoryview(data)
    offset = 0
    while offset < size:
        received = sock.recv_into(view[offset:])
        if received <= 0:
            raise ConnectionError("K230关闭了连接")
        offset += received
    return data


def decode_frame(cv2, np, jpeg_data):
    encoded = np.frombuffer(jpeg_data, dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def window_was_closed(cv2):
    try:
        return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


class FrameDisplay:
    """统一维护窗口、显示帧率和退出按键。"""

    def __init__(self, cv2):
        self.cv2 = cv2
        self.window_created = False
        self.frame_count = 0
        self.display_fps = 0.0
        self.fps_started_at = time.perf_counter()

    def show(self, frame, width, height, transport, dropped_frames=0):
        cv2 = self.cv2
        if not self.window_created:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 960, 720)
            self.window_created = True
            print("[receiver] 已连接，画面=%dx%d" % (width, height))

        self.frame_count += 1
        elapsed = time.perf_counter() - self.fps_started_at
        if elapsed >= 1.0:
            self.display_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_started_at = time.perf_counter()

        status = "%s LIVE  %.1f FPS" % (transport, self.display_fps)
        if transport == "UDP":
            status += "  DROP %d" % dropped_frames
        cv2.putText(
            frame,
            status,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(WINDOW_NAME, frame)
        return self.poll()

    def poll(self):
        if not self.window_created:
            return False
        key = self.cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return True
        return window_was_closed(self.cv2)

    def close(self):
        self.cv2.destroyAllWindows()


def run_tcp_receiver(cv2, np, host, port):
    print("[receiver] TCP地址: %s:%d" % (host, port))
    print("[receiver] 正在连接，按Q或Esc退出")

    display = FrameDisplay(cv2)
    stop_requested = False
    previous_sequence = None

    while not stop_requested:
        sock = None
        try:
            sock = connect_stream(host, port)
            print("[receiver] TCP连接成功")

            while True:
                header_data = recv_exact(sock, HEADER_STRUCT.size)
                magic, sequence, width, height, jpeg_size = (
                    HEADER_STRUCT.unpack(header_data)
                )
                if magic != FRAME_MAGIC:
                    raise ValueError("帧头标识错误")
                if jpeg_size <= 0 or jpeg_size > MAX_JPEG_SIZE:
                    raise ValueError("JPEG长度异常: %d" % jpeg_size)

                jpeg_data = recv_exact(sock, jpeg_size)
                frame = decode_frame(cv2, np, jpeg_data)
                if frame is None:
                    print("[receiver] 跳过一帧JPEG解码失败数据")
                    continue

                if previous_sequence is not None:
                    expected = (previous_sequence + 1) & 0xFFFFFFFF
                    if sequence != expected:
                        print(
                            "[receiver] 序号跳变: %d -> %d" % (
                                previous_sequence,
                                sequence,
                            )
                        )
                previous_sequence = sequence

                if display.show(frame, width, height, "TCP"):
                    stop_requested = True
                    break

        except (ConnectionError, OSError, ValueError) as error:
            print("[receiver] 连接中断:", error)
        finally:
            if sock is not None:
                sock.close()

        if not stop_requested:
            time.sleep(RECONNECT_DELAY_SECONDS)

    display.close()
    print("[receiver] 已退出")


def sequence_is_newer(sequence, reference):
    """按32位回绕序号判断 sequence 是否比 reference 新。"""
    difference = (sequence - reference) & 0xFFFFFFFF
    return 0 < difference < 0x80000000


def sequence_gap(previous, current):
    """返回两个已显示序号之间缺少的帧数。"""
    difference = (current - previous) & 0xFFFFFFFF
    if 0 < difference < 0x80000000:
        return max(0, difference - 1)
    return 0


def is_transient_udp_error(error):
    """Windows 会把 UDP 端口不可达报告为 10054，而不是普通超时。"""
    code = getattr(error, "winerror", None)
    if code is None:
        code = getattr(error, "errno", None)
    if code is None and error.args:
        code = error.args[0]
    return code in UDP_TRANSIENT_ERROR_CODES


def run_udp_receiver(cv2, np, host, port):
    target = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    except OSError:
        pass
    sock.bind(("", 0))
    sock.settimeout(0.2)

    print("[receiver] UDP地址: %s:%d" % target)
    print("[receiver] 本地端口: %d（应与K230日志 receiver registered 一致）" % sock.getsockname()[1])
    print("[receiver] 正在登记接收端，按Q或Esc退出")
    print("[receiver] 注意：同时只能运行一个接收程序，否则两端都组不出完整帧")

    display = FrameDisplay(cv2)
    assemblies = {}
    newest_sequence = None
    displayed_sequence = None
    dropped_frames = 0
    last_hello_at = 0.0
    stop_requested = False
    waiting_message_shown = False

    try:
        while not stop_requested:
            now = time.perf_counter()
            if now - last_hello_at >= UDP_HELLO_INTERVAL_SECONDS:
                try:
                    sock.sendto(UDP_HELLO_MAGIC, target)
                except OSError as error:
                    if not is_transient_udp_error(error):
                        raise
                    if not waiting_message_shown:
                        print("[receiver] K230 UDP端口尚未就绪，继续等待")
                        waiting_message_shown = True
                last_hello_at = now

            try:
                datagram, address = sock.recvfrom(UDP_DATAGRAM_SIZE + 64)
            except socket.timeout:
                if display.poll():
                    break
                if assemblies:
                    sequence, assembly = next(iter(assemblies.items()))
                    if now - assembly["created_at"] >= UDP_FRAME_TIMEOUT_SECONDS:
                        del assemblies[sequence]
                continue
            except OSError as error:
                if not is_transient_udp_error(error):
                    raise
                if not waiting_message_shown:
                    print("[receiver] K230 UDP端口尚未就绪，继续等待")
                    waiting_message_shown = True
                if display.poll():
                    break
                continue

            if waiting_message_shown:
                print("[receiver] K230 UDP端口已响应")
                waiting_message_shown = False

            if address[0] != target[0]:
                continue
            if len(datagram) < UDP_HEADER_STRUCT.size:
                continue

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
                continue
            if jpeg_size <= 0 or jpeg_size > MAX_JPEG_SIZE:
                continue

            expected_count = (
                jpeg_size + UDP_PAYLOAD_SIZE - 1
            ) // UDP_PAYLOAD_SIZE
            if (fragment_count != expected_count or
                    fragment_count <= 0 or
                    fragment_count > MAX_UDP_FRAGMENTS or
                    fragment_index >= fragment_count):
                continue

            payload = datagram[UDP_HEADER_STRUCT.size:]
            expected_payload_size = UDP_PAYLOAD_SIZE
            if fragment_index == fragment_count - 1:
                expected_payload_size = (
                    jpeg_size
                    - UDP_PAYLOAD_SIZE * (fragment_count - 1)
                )
            if len(payload) != expected_payload_size:
                continue

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

            frame = decode_frame(cv2, np, jpeg_data)
            if frame is None:
                continue

            if displayed_sequence is not None:
                dropped_frames += sequence_gap(
                    displayed_sequence,
                    sequence,
                )
            displayed_sequence = sequence
            if display.show(
                    frame,
                    assembly["width"],
                    assembly["height"],
                    "UDP",
                    dropped_frames):
                stop_requested = True

    finally:
        sock.close()
        display.close()
    print("[receiver] 已退出，UDP丢帧=%d" % dropped_frames)


def main():
    args = parse_args()
    cv2, np = load_opencv()
    verify_opencv_gui(cv2)
    if args.port is None:
        port = (
            DEFAULT_UDP_PORT
            if args.transport == "udp"
            else DEFAULT_TCP_PORT
        )
    else:
        port = args.port

    if args.transport == "udp":
        run_udp_receiver(cv2, np, args.host, port)
    else:
        run_tcp_receiver(cv2, np, args.host, port)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[receiver] 用户停止接收")
    except Exception as error:
        print("[receiver] 运行失败:", error)
        sys.exit(1)
