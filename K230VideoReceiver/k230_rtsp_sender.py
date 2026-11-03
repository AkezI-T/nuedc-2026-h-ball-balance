"""K230最小RTSP推流测试。

该脚本只运行摄像头和H.264推流，不运行钢珠识别模型。
"""

import gc
import network
import os
import time

import multimedia as mm
from media.media import *
from media.sensor import *
from media.vencoder import *


WIFI_SSID = "YOUR_HOTSPOT_SSID"
WIFI_PASSWORD = "YOUR_HOTSPOT_PASSWORD"
WIFI_TIMEOUT_MS = 15000

STREAM_WIDTH = 640
STREAM_HEIGHT = 480
SENSOR_FPS = 30
STREAM_FPS = 20
STREAM_BITRATE_KBPS = 1200
RTSP_PORT = 8554
RTSP_SESSION = "video"
ENCODER_TIMEOUT_MS = 1000
HEARTBEAT_PERIOD_MS = 5000


def connect_wifi():
    """等待热点连接和DHCP地址，成功时返回WLAN对象及IP。"""
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


def main():
    sensor = None
    encoder = None
    rtsp_server = None
    media_link = None
    media_initialized = False
    encoder_created = False
    encoder_started = False
    sensor_started = False
    rtsp_started = False

    try:
        _, ip_address = connect_wifi()
        width = ALIGN_UP(STREAM_WIDTH, 16)

        sensor = Sensor(fps=SENSOR_FPS)
        sensor.reset()
        sensor.set_framesize(
            width=width,
            height=STREAM_HEIGHT,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP)

        encoder = Encoder()
        encoder.SetOutBufs(
            VENC_CHN_ID_0,
            8,
            width,
            STREAM_HEIGHT,
        )
        MediaManager.init()
        media_initialized = True

        encoder_attributes = ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_BASELINE,
            width,
            STREAM_HEIGHT,
            bit_rate=STREAM_BITRATE_KBPS,
            gopLen=STREAM_FPS,
            dst_frame_rate=STREAM_FPS,
            src_frame_rate=SENSOR_FPS,
        )
        encoder.Create(VENC_CHN_ID_0, encoder_attributes)
        encoder_created = True

        # 编码器创建完成后再建立摄像头到VENC的硬件链路。
        media_link = MediaManager.link(
            sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_CHN_ID_0),
        )

        rtsp_server = mm.rtsp_server()
        rtsp_server.rtspserver_init(RTSP_PORT)
        rtsp_server.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        rtsp_server.rtspserver_start()
        rtsp_started = True

        encoder.Start(VENC_CHN_ID_0)
        encoder_started = True
        sensor.run()
        sensor_started = True

        stream_url = "rtsp://%s:%d/%s" % (
            ip_address,
            RTSP_PORT,
            RTSP_SESSION,
        )
        print("[rtsp] started:", stream_url)
        print(
            "[rtsp] video=%dx%d fps=%d bitrate=%dkbps" % (
                width,
                STREAM_HEIGHT,
                STREAM_FPS,
                STREAM_BITRATE_KBPS,
            )
        )
        print("[rtsp] input mode: sensor-venc hardware link")
        print("[rtsp] send mode: physical + encoder pts")
        print("[rtsp] waiting for first encoded frame...")

        stream_data = StreamData()
        frame_number = 0
        heartbeat_frames = 0
        heartbeat_started_at = time.ticks_ms()

        while True:
            stream_result = encoder.GetStream(
                VENC_CHN_ID_0,
                stream_data,
                timeout=ENCODER_TIMEOUT_MS,
            )
            if stream_result != 0:
                print("[rtsp] encoder output timeout:", stream_result)
                continue

            pack_count = stream_data.pack_cnt
            for pack_index in range(pack_count):
                packet_size = stream_data.data_size[pack_index]
                timestamp = stream_data.pts[pack_index]
                if timestamp <= 0:
                    timestamp = time.ticks_ms()
                rtsp_server.rtspserver_sendvideodata_byphyaddr(
                    RTSP_SESSION,
                    stream_data.phy_addr[pack_index],
                    packet_size,
                    timestamp,
                )

            encoder.ReleaseStream(VENC_CHN_ID_0, stream_data)
            frame_number += 1
            heartbeat_frames += 1
            if frame_number == 1:
                print(
                    "[rtsp] first frame sent, packs=%d pts=%d" % (
                        pack_count,
                        stream_data.pts[0],
                    )
                )

            now_ms = time.ticks_ms()
            heartbeat_elapsed = time.ticks_diff(
                now_ms,
                heartbeat_started_at,
            )
            if heartbeat_elapsed >= HEARTBEAT_PERIOD_MS:
                actual_fps = (
                    heartbeat_frames * 1000.0 / heartbeat_elapsed
                )
                print(
                    "[rtsp] running frames=%d fps=%.1f" % (
                        frame_number,
                        actual_fps,
                    )
                )
                heartbeat_frames = 0
                heartbeat_started_at = now_ms

            if frame_number % 120 == 0:
                gc.collect()
            os.exitpoint()

    except KeyboardInterrupt:
        print("[rtsp] 用户停止推流")
    except Exception as error:
        print("[rtsp] 推流异常:", error)
    finally:
        if sensor_started:
            sensor.stop()
        media_link = None
        if encoder_started:
            encoder.Stop(VENC_CHN_ID_0)
        if encoder_created:
            encoder.Destroy(VENC_CHN_ID_0)
        if rtsp_started:
            rtsp_server.rtspserver_stop()
            rtsp_server.rtspserver_deinit()
        if media_initialized:
            MediaManager.deinit()
        gc.collect()
        print("[rtsp] 资源已释放")


if __name__ == "__main__":
    main()
