# 2026 H题车载平衡滚球系统（电机连杆机构）

本仓库是 2026 年全国大学生电子设计竞赛 H 题“车载平衡滚球系统”的整车工程，覆盖底盘循迹、钢球视觉定位、车载闭环控制、无线图传和自动录像。

系统目标是让小车沿黑色环形赛道完成 T1-T6 任务，同时通过 D36A 步进电机控制摆杆，使钢球到达指定位置或在行驶中保持稳定。

> 本项目是竞赛工程，当前实物采用电机直接驱动连杆。电机编码器count闭合位置内环；源码未显式将电机轴角转换为摆杆倾角。GPIO、方向、编码器符号、视觉阈值和控制参数均需结合实物复核。

原题与三份分析报告见[文档说明](docs/README.md)；第三方来源见[第三方来源说明](THIRD_PARTY_NOTICES.md)。

## 系统架构

链路自底盘向上汇总：TMX 通过 UART0 上报任务事件，ESP32 通过 UART1 与 K230 交换目标与视觉反馈，ESP32 与 K230 各自通过 UDP 向上位机发送日志与视频。

| 子系统 | 平台与工具链 | 当前职责 |
|---|---|---|
| TMX 底盘主控 | 立创天猛星 MSPM0G3507、CCS Theia、DriverLib、SysConfig、FreeRTOS | 单按键菜单、T1-T6 生命周期、八路红外巡线、TB6612 双轮控制、编码器里程、A/B 点判定、任务事件单向上报 |
| 滚球与网络集中处理器 | ESP32-S3 N16R8、ESP-IDF 5.4.3、FreeRTOS | 任务解析与去重、D36A 控制、MS42CG 编码器采集、K230 视觉闭环、JY901S 前馈、UDP 网络遥测 |
| 主 K230 | CanMV MicroPython、传统 CV | 钢球检测、位置/速度估计、UART 视觉反馈、320x240 UDP JPEG 图传、本地 640x480 OSD |
| Windows 上位机 | Python、PySide6、OpenCV | UDP 视频重组、实时显示、MP4 自动录像、整车日志分类与保存 |
| 第二块 K230（独立可选） | CanMV MicroPython、HTTP MJPEG | 1280x720 独立图传和浏览器录像，不参与车辆控制 |

## T1-T6 任务

| 任务 | 底盘行为 | 滚球行为 | 完成/计时逻辑 |
|---|---|---|---|
| T1 | 静止 | 不启用 | 验证 `TASK_BEGIN/TASK_END` 能否自动开关上位机录像 |
| T2 | 快速循迹一圈 | 不启用 | 目标 20 s 内回到 A 点；终点后执行 1.5 s 线性软停 |
| T3 | 静止 | `+50 mm -> -50 mm` | TMX 提供 5 s 录像窗口；ESP32按视觉误差、速度和电机编码器count判定目标切换与最终稳定 |
| T4 | 降速从 A 到 B | 稳定在 O 点 | `2700 count` 锁存 B 点时间，约 `2850 count` 后软停；目标 8 s 内通过 B 点 |
| T5 | 降速循迹一圈 | 稳定在 O 点 | A 点横线确认后继续循迹 `300 count`（约 16.7 cm）摆正，再软停；目标 30 s 内完成 |
| T6 | 与 T5 相同 | 稳定在准备页记录的任意初始位置 | PREP 中采集并在启动时冻结 `referencePx`；无有效参考则进入故障状态 |

TMX 菜单统一采用以下操作流程：

```text
主菜单短按选择 -> 双击进入 PREP -> 短按启动
```

T1 运行页短按结束；T2/T4/T5/T6 自动结束并支持双击中止；T3 的 TMX 运行窗口固定为 5 s。`TEST` 是独立诊断页，不占用任务编号，当前源码只采集编码器、视觉和 IMU 遥测，D36A 保持取消使能。

## 当前通信与接线

### 有线控制链路

| 链路 | 接线 | 协议 |
|---|---|---|
| TMX -> ESP32 | `PA10/UART0_TX -> GPIO11/UART2_RX`，共地 | 115200-8N1，单向 ASCII 行协议；ESP32 的 GPIO12 已释放 |
| K230 -> ESP32 | `IO9/UART1_TX -> GPIO46/UART1_RX` | `@VIS,VALID,E_PX,LAST_E_PX,V_PX_S\r\n` |
| ESP32 -> K230 | `GPIO13/UART1_TX -> IO10/UART1_RX` | `@TGT,TARGET_MM\r\n` |
| D36A | ESP32 `GPIO18=STEP`、`GPIO8=DIR`、`GPIO3=EN` | EN 高有效；DIR 高时向车头转且 A/B 计数增加 |
| MS42CG | ESP32 `GPIO4=A`、`GPIO5=B`、`GPIO6=PWM`、`GPIO7=Z` | A/B 按 4096 count/rev 用于位置内环，PWM/Z 用于零点与诊断 |
| JY901S | ESP32 `GPIO16=SDA`、`GPIO15=SCL` | I2C0、100 kHz、7 位地址 `0x50` |
| WS2812 补光灯 | ESP32 `GPIO21 -> DIN` | 8 灯，默认白光亮度 `64/255`，外部 5 V 供电 |

所有 UART 均为 3.3 V TTL，所有控制板和驱动器必须共地。完整底盘接线见 [硬件接线与 SysConfig 配置](G3507%20TMX%20MainWork/docs/%E7%A1%AC%E4%BB%B6%E6%8E%A5%E7%BA%BF%E4%B8%8ESysConfig%E9%85%8D%E7%BD%AE.md)。

### 无线链路

| 端口 | 发送端 -> 接收端 | 用途 |
|---:|---|---|
| UDP 9001 | K230 -> 上位机 | 320x240 JPEG 分片；`KHEL` 登记接收端，`KUJP` 为 20 字节帧头 |
| UDP 9011 | ESP32 -> 上位机广播 | `@LINK/@CAR/@CTRL/@TEST/@TASK/@MODE/@PARAM/@TARGET` 日志 |
| TCP 9000 | K230 -> 命令行接收器 | 纯 JPEG 图传回退模式 |
| TCP 8080 | 第二块独立 K230 -> 浏览器 | HTTP MJPEG 页面和浏览器端录像 |
| UDP 9010 | 旧 ESP32 组件 | 旧版 JSON 录像通知；当前启动流程未使用 |
| UDP 9020 | 旧 K230 模型脚本 | 旧版视觉控制发现/反馈；当前传统 CV 主链路未使用 |

TMX 的任务和模式事件连续发送 3 次、间隔 150 ms。ESP32 对任务事件按 `source + eventSeq + type + run` 去重，对模式事件按 `source + eventSeq + type + mode` 去重，但会把原始重复行转发到上位机，便于判断链路质量。任务开始和结束只以 `@TASK` 为准，不能从 `@CAR state` 推断。

## 控制实现

### 底盘循迹

TMX 以 5 ms 周期运行控制任务。八路探头中 `0` 表示黑线，X1-X8 使用 `-7,-5,-3,-1,+1,+3,+5,+7` 权重求平均误差，再以 PD 差速控制左右轮。

- T2 使用快速档：基础 PWM 320，最大 600。
- T4 使用独立降速档：120 PWM 起步并在 1.2 s 内渐升到 250，最大 420。
- T5/T6 使用另一组降速档：120 PWM 起步并渐升到 220，最大 420。
- 丢线不足 400 ms 时按上次误差方向搜索；超时、巡线数据过期、运行超时或电机堵转均进入故障停车。
- 接近终点时渐近降速；T5/T6 通过 A 点后保留 300 count 摆正段；正常停车不使用 TB6612 短接抱刹。

TMX 使用四个静态分配 FreeRTOS 任务：Sensor 1 ms、Control 5 ms、UI 10 ms、Log 150 ms。巡线模块和 JY901S 共用非阻塞 I2C 事务队列。

### 滚球双闭环

ESP32 的控制任务固定在 Core 1，Wi-Fi、UART、UDP 和监控任务主要运行在 Core 0。

1. K230 将钢球位置相对目标位置的误差和 Alpha-Beta 速度估计发送给 ESP32。
2. ESP32 的视觉外环计算电机目标偏置 `encTarget - balance`，单位为count。
3. MS42CG A/B 编码器位置内环按位置误差生成 D36A 的方向和 LEDC STEP 频率。
4. 位置误差越大，STEP 频率越高；到达 1 count 容差后停脉冲并保持使能。

T3 使用电机计数到位驱动的分段轨迹：DRIVE、TURN、BRAKE、DAMP。正向与返程具有不同的零偏、驱动和制动幅值；返程终点附近低速停滞时，可执行最多3次逐级增大的起动/制动脉冲。

T4/T5/T6 使用行驶稳球控制器，而不是复用 T3 的大角冲刺：

- 位置 P + 速度阻尼 + 受限积分；
- 起步固定前馈及渐退；
- 静摩擦确认和短脉冲补偿；
- JY901S X 轴纵向加速度动态前馈；
- 视觉短时无效时冻结反馈项，前馈仍独立更新。

T6 的实际控制误差为：

```text
controlErrorPx = currentEpx - referencePx
```

### K230 传统视觉

当前主脚本是 `K230VideoReceiver/k230_classical_ball_test.py`：

- CH0：640x480 YUV420SP，硬件直连 LCD；
- CH1：320x240 YUV420SP，质量 40、目标 15 FPS 的 JPEG 图传；
- CH2：640x480 灰度图，固定 `y=55..135` ROI 做传统识别；
- 阈值、形态学和连通域筛选后，只保留符合尺寸、密度、圆度和导轨位置的候选；
- Alpha-Beta 跟踪器参数为 `alpha=0.70`、`beta=0.20`，并带动态门限、创新限幅、静止锁定和短时丢检处理；
- 单帧 blob 超过 64 个时直接判为过载，候选最多保留 16 个，防止强反光导致对象暴增。

目标毫米值通过已标定的 `0..100 mm` 像素表分段线性插值。机构协议范围为 `-110..+110 mm`，但正负 11 cm 尚未完成像素标定，当前 K230 脚本实际只接受绝对值不大于 100 mm 的目标。

### 上位机

`upper_computer.py` 提供：

- UDP 9001 JPEG 分片重组、丢帧统计和实时显示；
- `TASK_BEGIN` 自动开始、`TASK_END` 自动停止的 MP4 录像；
- T4/T5/T6 在 `TASK_BEGIN` 丢包时由 `@TARGET TRACK/FROZEN` 或 `@CTRL stage=TRACK` 兜底开始录像；
- 按真实墙钟时间补帧，避免低接收帧率把任务录像加速播放；
- 将日志分为底盘 `@CAR`、滚球 `@CTRL/@TEST`、任务/其他事件三列，并以同一时间戳保存三份日志。

录像写入 `K230VideoReceiver/Save/`，日志写入 `K230VideoReceiver/Logs/`；两者均已忽略，不提交到仓库。

## 目录结构

```text
GearRackMechanism/
|- G3507 TMX MainWork/         MSPM0G3507 底盘主控工程
|- Esp32 S3 MainWork/          ESP32-S3 滚球集中处理器
|- K230VideoReceiver/          主 K230 脚本、Windows 上位机与调试接收器
|- K230StandaloneMjpeg/        第二块 K230 的独立 HTTP MJPEG 服务
|- docs/                       原题、三份分析报告与文档说明
|- TASK_COMMUNICATION_FLOW.md  任务通信与运行流程
`- LICENSE                     根工程 MIT 许可证
```

各子工程的详细说明：

- [MSPM0G3507 工程说明](G3507%20TMX%20MainWork/README.md)
- [ESP32-S3 工程说明](Esp32%20S3%20MainWork/README.md)
- [K230 与上位机说明](K230VideoReceiver/README.md)
- [独立 K230 MJPEG 说明](K230StandaloneMjpeg/README.md)
- [任务通信与运行流程](TASK_COMMUNICATION_FLOW.md)

## 构建与运行

### 1. 构建 TMX 固件

1. 在 CCS Theia 中从纯 ASCII 路径导入 `G3507 TMX MainWork`，工程名应为 `G3507_TMX_MainWork`。
2. 当前工程记录的工具链为 MSPM0 SDK `2.10.00.04`、SysConfig `1.26.2`、TI Clang `4.0.4.LTS`。
3. 使用 `SEGGER J-Link Emulator + MSPM0G3507` 下载调试。
4. 首次通电先断开电机电池，验证 LCD、I2C、UART 和按键；随后架空车轮检查正转方向及编码器正负。

### 2. 构建 ESP32-S3 固件

在 ESP-IDF 5.4.3 环境中执行：

```powershell
Set-Location "Esp32 S3 MainWork"
Copy-Item sdkconfig.defaults.example sdkconfig.defaults
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p COMx flash monitor
```

在 `menuconfig -> 车载平衡滚球集中处理器` 中填写 Wi-Fi SSID/密码并复核 UART、I2C 和补光灯配置。`sdkconfig` 与本机 `sdkconfig.defaults` 已被忽略，不应提交热点凭据。

### 3. 启动 K230 与上位机

1. 让 ESP32、K230 和电脑连接同一个热点。
2. 在 `k230_classical_ball_test.py` 顶部填写 `WIFI_SSID` 和 `WIFI_PASSWORD`。
3. 用 CanMV IDE 运行该脚本，确认本地屏幕能够识别钢球，串口显示 UART1 和 UDP 9001 已就绪。
4. 电脑双击 `K230VideoReceiver/start_upper_computer.bat`；脚本会自动创建 `.venv` 并安装 PySide6/OpenCV。
5. K230 IP 不是默认值 `10.43.133.238` 时，使用：

```powershell
.\K230VideoReceiver\start_upper_computer.bat --host <K230-IP> --port 9001
```

同一台电脑不要同时运行 GUI 和命令行 UDP 接收器。K230 只向第一个有效 `KHEL` 心跳来源发送视频。

纯视频链路排查可使用：

```powershell
.\K230VideoReceiver\start_udp_receiver.bat --host <K230-IP>
.\K230VideoReceiver\start_receiver.bat --host <K230-IP> --port 9000
```

## 首次联调检查

1. ESP32 重启后首次进入 T3-T6 PREP 前，人工将摆杆置于真实水平位置；平衡零点只保存在本次开机期间。
2. 确认 K230 `@VIS` 持续到达 ESP32，ESP32 每秒重发 `@TGT`，视觉与视频链路互不依赖。
3. 确认上位机事件列出现 `@LINK type=UDP_LOG state=CONNECTED`，再测试 T1 自动录像。
4. 用 TEST 页核对 A/B、PWM、Z、视觉误差和 JY901S 数据；当前 TEST 不驱动 D36A。
5. 先架空底盘完成 T2 方向检查，再在赛道上分别标定 T2 和 T4/T5/T6 参数。
6. 任务通过与否以实车、录像和保存日志共同判断，不以单条 `@CTRL` 样本判断。

## 当前状态与已知边界

- `k230_classical_ball_test.py` 是与当前 ESP32 固件匹配的主视觉脚本，使用 UART1 控制链路。
- `k230_tcp_jpeg_detect.py` 是 KPU 模型识别版本，仍使用旧 UDP 9020 控制链路；直接替换传统 CV 脚本会与当前 ESP32 UART 控制方式不兼容。
- `TASK_COMMUNICATION_FLOW.md` 描述通信与流程；T3设计建议和当前运行行为已明确区分。
- T3的TMX `TASK_END`先经UDP 9011转发并结束5 s录像；ESP32随后忽略该结束事件，继续运行滚球状态机直到内部稳定。因此录像结束不等于滚球已在5 s内完成。
- 2026-08-01的一组本地历史日志出现`T3_NEG`，首次为任务elapsed 3.720 s，首次`COMPLETE`为7.090 s；原始日志没有固件版本绑定，不能视为当前源码的回归验证。详见第二份分析报告。
- T4较新的可见历史片段结束事件elapsed为6.540 s；去重有效视觉帧中36.26%位于±27 px内，最大绝对误差78 px。时间和稳球误差必须分别评估；这些历史统计不代表当前版本已通过验收。
- `K230VideoReceiver.zip` 是被忽略的本地打包文件，内部还包含独立 `.git` 历史；日常开发以目录中的当前源码为准，不把压缩包当作发布源。
- T3的13 px限制会在默认启动路径被active_params覆盖，默认运行门限为27 px；不能按13 px作为已落实精度。
- 仓库没有自动化硬件在环测试。控制参数、视觉阈值和像素标定均应在最终实车上重新整定。

相关测试记录：

- [T3 编码状态与实际运动不一致记录](Esp32%20S3%20MainWork/docs/T3%E7%BC%96%E7%A0%81%E7%8A%B6%E6%80%81%E4%B8%8E%E5%AE%9E%E9%99%85%E8%BF%90%E5%8A%A8%E4%B8%8D%E4%B8%80%E8%87%B4%E8%AE%B0%E5%BD%95.md)
- [T4 首次实车测试与抖动记录](Esp32%20S3%20MainWork/docs/T4%E9%A6%96%E6%AC%A1%E5%AE%9E%E8%BD%A6%E6%B5%8B%E8%AF%95%E4%B8%8E%E6%8A%96%E5%8A%A8%E8%AE%B0%E5%BD%95.md)

## 许可证

根工程原创部分沿用 [MIT License](LICENSE)。第三方文件头、子目录许可和来源说明继续有效，见[第三方来源说明](THIRD_PARTY_NOTICES.md)。题面和分析报告的材料属性在其目录中另行说明。

`K230StandaloneMjpeg` 基于 AFL-3.0 项目修改，适用其目录中的 [AFL-3.0 许可证](K230StandaloneMjpeg/LICENSE)。复用或分发该子项目时需同时遵守上游许可。
