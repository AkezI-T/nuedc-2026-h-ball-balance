# K230 独立 HTTP MJPEG 图传测试

本目录用于第二块 K230。它只负责采集和发送视频，不加入车辆控制链路。

## 网络结构

```text
手机热点（YOUR_HOTSPOT_SSID）
├── 原 K230：钢球识别与车辆控制
├── 新 K230：HTTP MJPEG 图传服务器
└── 电脑：Chrome/Edge 浏览器接收视频
```

新 K230 不发送 `@VIS`，不接收 `@TGT`，不使用 UDP `9001/9020`，也不需要连接 ESP32。

## 开源方案参数

- 协议：HTTP/TCP MJPEG
- 端口：`8080`
- 分辨率：`1280x720`
- 目标帧率：`25 FPS`
- JPEG质量：`50`
- 本地LCD：不启用
- 录像：由网页中的浏览器 `MediaRecorder` 完成

程序基于以下开源文件进行最小配置修改：

```text
H题/图传/k230/k230d版源码/image_trans_k230.py
```

上游仓库：

```text
https://gitee.com/linsen06/2026diansaikaiyuanqingdane
```

上游许可为 AFL-3.0，许可全文保存在本目录 `LICENSE`。

## 上板测试

1. 新 K230 建议使用与开源作者一致的 CanMV 1.6 固件。联网流程按
   CanMV 1.6官方WLAN例程处理：调用 `connect()` 后轮询
   `isconnected()`，不使用其返回值判断连接是否启动成功。
2. 在 CanMV IDE 中打开 `main.py`，确认热点账号和密码。
3. 运行程序，等待串口输出浏览器地址。
4. 电脑连接同一个手机热点。
5. 使用 Chrome 或 Edge 打开日志中的地址，例如：

```text
http://10.43.133.xxx:8080/
```

必须使用新 K230 日志打印的实际 IP，不能继续使用原 K230 的 `10.43.133.238`。

正常启动时会出现类似日志：

```text
Wi-Fi network information: (...)
JPEG encoder backend: media.vencoder JPEG (CanMV 1.6 compatibility)
First JPEG size: ... bytes
Standalone K230 MJPEG server started.
Open this URL with Chrome or Edge:
  http://10.43.133.xxx:8080/
```

浏览器打开后，串口应继续输出：

```text
HTTP client: (...) path: /
HTTP client: (...) path: /stream
MJPEG stream: ... fps
```

## 第一轮判断标准

- 能打印 `First JPEG size`：摄像头和硬件JPEG编码链路正常。
- 能访问首页但没有画面：重点检查 `/stream` 请求和JPEG发送日志。
- 运行时报 `dump_frame` 或 `SendFrame` 参数错误：实际固件接口低于开源代码要求，需要使用底层 `k_video_frame_info` 兼容层。
- 出现 `vb config failed(18)`：完整断电重启新 K230 后再测试，软重启不能清理残留VB池。
