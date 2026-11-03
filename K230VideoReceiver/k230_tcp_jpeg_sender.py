"""K230 JPEG/TCP无线图传发送端。

协议头固定16字节：KJPG、序号、宽度、高度、JPEG数据长度，均使用网络字节序。
"""

import gc
import network
import os
import socket
import struct
import time
import uctypes

from media.media import *
from media.sensor import *
from media.vencoder import *


WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD"
WIFI_TIMEOUT_MS = 15000

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SENSOR_FPS = 30
STREAM_FPS = 15
JPEG_QUALITY = 70

TCP_PORT = 9000
TCP_BACKLOG = 1
FRAME_MAGIC = b"KJPG"
HEADER_FORMAT = ">4sIHHI"

VENC_CHANNEL = VENC_CHN_ID_0
VENC_BUFFER_NUM = 8
ENCODER_TIMEOUT_MS = 1000
STATS_PERIOD_MS = 5000
CLIENT_SOCKET_TIMEOUT_SECONDS = 1
SEND_STALL_TIMEOUT_MS = 5000


def connect_wifi():
    """连接热点并等待DHCP地址。"""
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
            raise RuntimeError("Wi-Fi连接超时")
        time.sleep_ms(100)


def create_jpeg_attributes(encoder, width, height):
    """创建JPEG编码参数，并兼容不同版本的ChnAttrStr。"""
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
    """创建非阻塞TCP监听套接字。"""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip_address, TCP_PORT))
    server.listen(TCP_BACKLOG)
    server.setblocking(False)
    print("[tcp] listening on %s:%d" % (ip_address, TCP_PORT))
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
    """处理TCP短写，确保一段数据完整发出。"""
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
            raise OSError("TCP连接已关闭")
        offset += sent
        last_progress_ms = time.ticks_ms()


def main():
    sensor = None
    encoder = None
    media_link = None
    server = None
    client = None
    media_initialized = False
    encoder_created = False
    encoder_started = False
    sensor_started = False

    try:
        _, ip_address = connect_wifi()
        width = ALIGN_UP(FRAME_WIDTH, 16)

        sensor = Sensor(fps=SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(
            width=width,
            height=FRAME_HEIGHT,
            chn=CAM_CHN_ID_0,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)

        encoder = Encoder()
        encoder.SetOutBufs(
            VENC_CHANNEL,
            VENC_BUFFER_NUM,
            width,
            FRAME_HEIGHT,
        )

        media_link = MediaManager.link(
            sensor.bind_info(chn=CAM_CHN_ID_0)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_CHANNEL),
        )
        MediaManager.init()
        media_initialized = True

        encoder.Create(
            VENC_CHANNEL,
            create_jpeg_attributes(encoder, width, FRAME_HEIGHT),
        )
        encoder_created = True
        encoder.Start(VENC_CHANNEL)
        encoder_started = True
        sensor.run()
        sensor_started = True

        server = create_tcp_server(ip_address)
        print(
            "[jpeg] frame=%dx%d sensor=%dfps send=%dfps quality=%d" % (
                width,
                FRAME_HEIGHT,
                SENSOR_FPS,
                STREAM_FPS,
                JPEG_QUALITY,
            )
        )

        stream_data = StreamData()
        encoded_frames = 0
        sent_frames = 0
        sent_bytes = 0
        sequence = 0
        send_divider = max(1, SENSOR_FPS // STREAM_FPS)
        stats_started_at = time.ticks_ms()

        while True:
            os.exitpoint()
            if client is None:
                client = try_accept_client(server)

            result = encoder.GetStream(
                VENC_CHANNEL,
                stream_data,
                timeout=ENCODER_TIMEOUT_MS,
            )
            if result not in (0, None):
                continue

            jpeg_data = None
            try:
                encoded_frames += 1
                should_send = (
                    client is not None
                    and encoded_frames % send_divider == 0
                )
                if should_send:
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
                    jpeg_data = b"".join(packets)
            finally:
                encoder.ReleaseStream(VENC_CHANNEL, stream_data)

            if jpeg_data:
                header = struct.pack(
                    HEADER_FORMAT,
                    FRAME_MAGIC,
                    sequence,
                    width,
                    FRAME_HEIGHT,
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

            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, stats_started_at)
            if elapsed_ms >= STATS_PERIOD_MS:
                elapsed_s = elapsed_ms / 1000.0
                print(
                    "[jpeg] encoded=%.1ffps sent=%.1ffps rate=%.2fMbps" % (
                        encoded_frames / elapsed_s,
                        sent_frames / elapsed_s,
                        sent_bytes * 8.0 / elapsed_s / 1000000.0,
                    )
                )
                encoded_frames = 0
                sent_frames = 0
                sent_bytes = 0
                stats_started_at = now_ms
                gc.collect()

    except KeyboardInterrupt:
        print("[jpeg] 用户停止图传")
    except Exception as error:
        print("[jpeg] 图传异常:", repr(error))
    finally:
        close_socket(client)
        close_socket(server)
        if sensor_started:
            sensor.stop()
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
        if media_initialized:
            MediaManager.deinit()
        gc.collect()
        print("[jpeg] 资源已释放")


if __name__ == "__main__":
    main()
