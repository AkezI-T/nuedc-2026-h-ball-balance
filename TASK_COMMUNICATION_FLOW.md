# H题任务通信与运行流程

本文固定车载平衡滚球系统的任务编号、设备职责、通信链路和运行状态。文末记录任务编号与旧固件日志之间的对应关系。

## 1. 任务编号

| 任务 | 定义 | 主要执行设备 |
|---|---|---|
| `T1` | 测试任务开始、结束能否自动打开和关闭上位机录像 | TMX、ESP32、上位机 |
| `T2` | 小车从A点启动，顺时针循迹一圈并停回A点 | TMX |
| `T3` | 小车静止，小球从O点运行到`+5 cm`，折返到`-5 cm`并稳定 | ESP32、K230 |
| `T4` | 小车从A点降速循迹通过B点，同时将小球稳定在O点 | TMX、ESP32、K230 |
| `T5` | 小车从A点降速循迹一圈并通过A点，同时将小球稳定在O点 | TMX、ESP32、K230 |
| `T6` | 小车从A点降速循迹一圈并通过A点，同时将小球稳定在准备页记录的任意初始位置 | TMX、ESP32、K230 |

## 2. 固定通信链路

| 发送端 | 接收端 | 接口 | 内容 |
|---|---|---|---|
| TMX `PA10/TX` | ESP32 `GPIO11/RX` | UART，115200-8N1 | `@MODE`、`@TASK`、`@PARAM`、`@CHECKPOINT`、`@CAR` |
| K230 `IO9/TX` | ESP32 `GPIO46/RX` | UART1，115200-8N1 | `@VIS`视觉反馈 |
| ESP32 `GPIO13/TX` | K230 `IO10/RX` | UART1，115200-8N1 | `@TGT`目标位置 |
| K230 | 上位机 | UDP `9001` | JPEG视频流 |
| ESP32 | 上位机 | UDP `9011` | `@LINK/@CAR/@CTRL/@TASK/@MODE`日志 |

TMX到ESP32只使用单向UART。硬件只需连接 `PA10/TX -> GPIO11/RX`和GND；TMX `PA11/RX`与ESP32 `GPIO12/TX`不连接，GPIO12可释放给后续外设。

UDP `9010`是旧版JSON录像通知通道。当前上位机不监听该端口，任务录像以UDP `9011`收到的 `@TASK` 为准。

## 3. 通用事件规则

TMX是任务编号和人工按键事件的来源。任务开始和结束事件均连续发送3次，间隔150 ms；同一事件的 `eventSeq/type/run/task/t` 保持不变，仅 `repeat` 变化。

```text
@TASK eventSeq=10 repeat=1/3 type=TASK_BEGIN run=3 t=12000 task=T2
@TASK eventSeq=10 repeat=2/3 type=TASK_BEGIN run=3 t=12000 task=T2
@TASK eventSeq=10 repeat=3/3 type=TASK_BEGIN run=3 t=12000 task=T2
```

ESP32和上位机按以下键去重，只执行一次状态切换；旧TMX日志未携带 `source`时按 `source=TMX`处理：

```text
source + eventSeq + type + run
```

重复日志仍可显示和保存，用于确认UART及Wi-Fi链路质量。

任务准备页使用独立的模式事件：

```text
@MODE eventSeq=9 repeat=1/3 type=ENTER mode=PREP task=T2 t=10000
@MODE eventSeq=11 repeat=1/3 type=EXIT mode=PREP task=T2 t=12100
```

上位机收到有效的 `PREP ENTER` 后，只刷新一次UDP视频接收器。`TASK_BEGIN`开始录像，`TASK_END`停止并保存录像。

### 3.1 分任务参数

ESP32为T1至T6分别保存参数，参数必须显式携带 `task`，不得使用一个全局目标覆盖其他任务。支持独立参数帧，也允许把参数直接附加到准备或开始事件：

```text
@PARAM source=TMX task=T3 targetMm=... target2Mm=... timeoutMs=...
@MODE source=TMX eventSeq=... type=ENTER mode=PREP task=T3 targetMm=...
@TASK source=TMX eventSeq=... type=TASK_BEGIN run=... task=T3 targetMm=...
```

参数字段及职责如下：

| 字段 | 使用任务 | 含义 |
|---|---|---|
| `targetMm` | T3/T4/T5/T6 | 第一目标或固定目标 |
| `target2Mm` | T3 | 折返后的第二目标 |
| `timeoutMs` | T2至T6 | 任务侧时间窗口或诊断超时 |
| `reachPx/reachFrames` | T3 | 第一目标到达判定 |
| `settlePx/settleSpeed/settleFrames` | T3 | 第二目标稳定判定 |
| `captureFrames` | T6 | 准备页参考位置采样帧数 |

进入PREP时ESP32加载指定任务参数；收到 `TASK_BEGIN` 时再次形成该次运行的参数快照。任务运行中对其他任务的参数更新不会改变当前闭环。ESP32通过 `@PARAM source=ESP32` 回显完整生效快照，便于从UDP 9011核对。

## 4. 公共启动链路

1. K230、车载ESP32和上位机连接同一个手机热点。
2. ESP32的UDP日志socket就绪后，向UDP `9011`广播一次：

```text
@LINK type=UDP_LOG state=CONNECTED device=ESP32S3 port=9011
```

3. K230 通过 UART1 持续发送 `@VIS`，ESP32 不需要握手即按最新视觉帧更新状态。
4. ESP32 在目标变化时发送 `@TGT,TARGET_MM`，此后每秒重发同一目标。
5. K230 收到新目标后立即按新目标计算 `E_PX`，并继续按自身帧率发送 `@VIS`。
6. 上位机向K230发送视频心跳，K230登记上位机地址并通过UDP `9001`发送JPEG。

## 5. T1：自动录像开关测试

### 5.1 目的

只验证任务事件经过TMX、ESP32和Wi-Fi到达上位机后，录像能够自动开始和停止。T1不启动底盘，不启动滚球闭环。

### 5.2 开始流程

```text
TMX按键
  -> TMX发送 @TASK TASK_BEGIN task=T1
  -> ESP32接收、去重并广播到UDP 9011
  -> 上位机接收、去重
  -> 上位机开始录制K230视频
```

示例：

```text
@TASK eventSeq=1 repeat=1/3 type=TASK_BEGIN run=1 t=5000 task=T1
```

期望状态：

- TMX底盘电机保持停止。
- ESP32不进入视觉位置闭环。
- 上位机显示“正在录制”，并持续写入视频帧。

### 5.3 结束流程

```text
TMX再次按键
  -> TMX发送 @TASK TASK_END task=T1
  -> ESP32广播到UDP 9011
  -> 上位机停止录像并保存MP4
```

示例：

```text
@TASK eventSeq=2 repeat=1/3 type=TASK_END run=1 t=10000 task=T1 result=DONE fault=NONE elapsed=5000
```

T1通过条件：一次 `TASK_BEGIN`只启动一次录像，一次 `TASK_END`只保存一个可回放MP4文件。

## 6. T2：第一次循迹

### 6.1 设备职责

- TMX负责按键、八路巡线、左右轮控制、编码器里程、A点停车和计时显示。
- ESP32负责接收并转发任务日志和 `@CAR`，不参与底盘闭环。
- K230只提供视频。
- 上位机按任务事件自动录像。

### 6.2 准备与开始

进入T2准备页时：

```text
TMX -> ESP32: @MODE ... type=ENTER mode=PREP task=T2
ESP32 -> 上位机UDP 9011: 原样转发@MODE
上位机: 刷新一次UDP视频接收器
```

按键启动时：

```text
TMX本地启动循迹状态机
TMX -> ESP32: @TASK ... type=TASK_BEGIN task=T2
ESP32 -> 上位机UDP 9011: 原样转发@TASK
上位机: 开始录像
```

T2运行期间TMX周期发送：

```text
@CAR ... state=FOLLOW ... lapEnc=<里程>
```

ESP32只把 `@CAR`放入UDP日志队列，不根据 `@CAR state`推断任务开始或结束。

### 6.3 停车与结束

TMX识别A点横线或达到有效里程后锁存完成时间，随后从当前左右轮PWM开始用1.5 s线性降到0。软停本身约滑行半个车身，不再额外执行300 count的摆正段，PWM归零后普通断电停车并发送：

```text
@TASK eventSeq=22 repeat=1/3 type=TASK_END run=4 t=30000 task=T2 result=DONE fault=NONE elapsed=16160 lapEnc=10650
```

软停阶段的 `@CAR state=SOFT_STOP`仍属于T2运行中，ESP32不得据此推断任务结束。ESP32转发最终 `TASK_END`后，上位机停止录像并保存文件。TMX屏幕保持显示锁存的整圈时间。

T2通过条件：顺时针完成一圈、A点完成时间不超过20 s、随后平缓降速并摆正停车，且录像完整。

## 7. T3：静止滚球 `+5 cm -> -5 cm`

### 7.1 设备职责

- TMX确认底盘停止，发送T3准备、开始和最终结束事件。
- ESP32拥有T3滚球状态机、运行elapsed、目标切换、视觉外环和电机编码器count位置内环；当前不会按5 s超时停止T3。
- K230接收目标位置，检测小球并返回相对当前目标的位置误差和速度。
- 上位机只负责视频、录像和日志，不参与闭环。

### 7.2 准备阶段

```text
TMX -> ESP32: @MODE ... type=ENTER mode=PREP task=T3
ESP32: 确认底盘任务未运行，使能D36A并回到已记录的平衡位置
ESP32 -> K230: @TGT,0\r\n
ESP32 -> 上位机: 转发@MODE并刷新一次视频接收
```

首次上电尚无平衡零点时，摆杆必须人工放平，再由ESP32等待编码器稳定并记录平衡位置。

### 7.3 开始和正向目标

```text
TMX -> ESP32: @TASK ... type=TASK_BEGIN task=T3
ESP32 -> 上位机: 转发@TASK，上位机开始录像
ESP32: 记录任务开始时刻，进入T3_POS（不以5 s到时结束）
ESP32 -> K230: @TGT,50\r\n
```

K230把 `+50 mm`换算为图像中心右侧的目标像素，并持续发送：

```text
@VIS,VALID,E_PX,LAST_E_PX,V_PX_S\r\n
```

其中：

```text
E_PX = BALL_X - TARGET_X
```

ESP32根据 `E_PX`和 `V_PX_S`运行 `HOLD -> DRIVE -> BRAKE -> DAMP -> SETTLE`，再通过编码器位置内环输出D36A的DIR和STEP。

### 7.4 到达 `+5 cm`并折返

ESP32必须使用视觉误差判断小球确实到达 `+5 cm`附近，不能只按固定时间折返。到达门槛应严于题目允许的 `1 cm`误差，并保留检测波动余量。

满足正向到达条件后：

```text
ESP32: MOVE_POS -> MOVE_NEG
ESP32 -> K230: @TGT,-50\r\n
```

K230立即把目标切换到图像中心左侧，并用新目标计算后续 `E_PX`。ESP32在目标变化时丢弃旧目标对应的视觉帧并重置本轮外环状态，但从TASK_BEGIN起的任务elapsed不重置。

### 7.5 稳定在 `-5 cm`：设计目标与当前实现

以下5 s约束与超时状态机是任务设计目标；当前源码实际使用T3_POS/T3_NEG/COMPLETE，未以5 s超时退出。

最终完成必须同时满足：

- 小球相对 `-5 cm`目标的位置误差处于允许范围内。
- 小球速度低于稳定门槛。
- 位置和速度连续稳定若干视觉帧，不能单帧命中即完成。
- 从 `TASK_BEGIN`开始的总时间不超过5 s。

建议状态机：

```text
PREP
  -> MOVE_POS
  -> POS_REACHED
  -> MOVE_NEG
  -> NEG_STABLE
  -> DONE
  -> FAULT_TIMEOUT（总时间超过5 s）
```

### 7.6 T3完成回报

T3采用固定5 s任务窗口。TMX从 `TASK_BEGIN`开始本地计时，达到5000 ms后直接发送三次结束事件：

```text
@TASK source=TMX eventSeq=2 repeat=1/3 type=TASK_END run=5 task=T3 result=DONE fault=NONE elapsed=5000 lapEnc=0
```

上位机按 `source/eventSeq/type/run`去重，只停止并保存一次录像。当前ESP32先转发TMX结束事件，随后在解析分支忽略T3的TASK_END，让滚球状态机继续运行到内部稳定判定；它不会因此在5 s到时停止外环，也不自行生成T3的TASK_END。比赛要求的5 s完成与当前行为必须分别检查。

若要加入5 s超时退出及TIMEOUT回报，属于后续代码改动；当前发布副本未实现该改动。录像窗口仍以TMX固定5 s结束事件为准。

T1至T6的最终 `TASK_END`现在均由TMX通过单向UART发送；ESP32只转发TMX任务事件并执行对应滚球控制。

## 8. 底盘速度策略

不同任务必须使用独立的底盘速度档：

| 任务 | 底盘状态 | 速度策略 |
|---|---|---|
| `T1` | 静止 | 不使能底盘 |
| `T2` | 行驶 | 保留当前快速循迹参数 |
| `T3` | 静止 | 不使能底盘 |
| `T4` | 行驶 | 使用独立降速循迹参数 |
| `T5` | 行驶 | 使用独立降速循迹参数 |
| `T6` | 行驶 | 使用独立降速循迹参数 |

除T2外，后续所有“行驶中稳球”任务默认使用降速档。T4、T5和T6可以共用一套初始降速参数，但必须与T2参数分开保存，避免稳球调参影响已经完成的快速循迹任务。

降速不仅调整基础PWM，还应保留输出斜坡，降低按键起步、巡线修正和进入弯道时的横向加速度突变。T4应把AB时间调到8 s以内并保留余量；T5应把整圈时间调到30 s以内并保留余量。

## 9. T4：A点到B点降速循迹并稳球

### 9.1 线路和B点基准

题目线路中AB是上方长度1.5 m的直线，B位于第一次右侧半圆弯道入口。两次完整循迹日志中，第一次持续转弯分别出现在：

| 完整圈编码器 | B前最后直行 | 首个持续转弯 |
|---:|---:|---:|
| `10176` | `2592` | `2674` |
| `10244` | `2568` | `2708` |

实车再次测得B点约为 `2.7k`，因此T4先使用独立参数：

```text
T4_B_PASS_ENCODER_COUNT = 2700
```

该参数只用于T4的B点通过判定，不替代T2/T5的A点横线和整圈里程判据。

### 9.2 准备阶段

```text
TMX -> ESP32: @MODE ... type=ENTER mode=PREP task=T4
TMX: 底盘保持停止并清零本圈编码器
ESP32: 使能D36A并回到平衡位置
ESP32 -> K230: @TGT,0\r\n
K230 -> ESP32: 持续发送相对O点的@VIS
```

只有K230控制链路有效、摆杆已回平且钢球位于O点允许范围内时，T4才进入可启动状态。

### 9.3 并行运行

按键启动后，TMX和ESP32同时开始各自控制：

```text
TMX -> ESP32: @TASK ... type=TASK_BEGIN task=T4
TMX: 使用降速档运行A到B循迹
ESP32: 保持targetMm=0，运行滚球位置外环和摆杆角度内环
ESP32 -> 上位机: 转发@TASK并开始录像
```

运行期间两条闭环互不互相等待：

- TMX根据八路巡线反馈控制左右轮。
- ESP32根据K230的 `E_PX/V_PX_S`控制D36A。
- ESP32周期广播 `@CTRL`，TMX周期发送 `@CAR`。

### 9.4 通过B点

当 `lapEnc >= 2700`时，TMX锁存AB行驶时间。B点要求“通过”而不是“停车”，因此不能在阈值处立即急刹；应先确认车体通过B基准，再按测试流程安全停车。

建议增加一次过点事件：

```text
@CHECKPOINT eventSeq=40 type=PASS run=6 task=T4 point=B elapsed=6500 lapEnc=2700
```

安全停车后发送最终 `TASK_END`，其中 `elapsed`使用已经锁存的AB时间：

```text
@TASK eventSeq=41 repeat=1/3 type=TASK_END run=6 task=T4 result=DONE fault=NONE elapsed=6500 lapEnc=2700
```

T4通过条件：AB锁存时间不超过8 s，且从启动到通过B的全过程中小球相对O点误差绝对值不超过1 cm。

## 10. T5：降速循迹一圈并稳球

T5是T2底盘整圈循迹与T4小球居中控制的组合，但使用T4/T5降速档，不使用T2快速档。

### 10.1 准备和开始

```text
TMX -> ESP32: @MODE ... type=ENTER mode=PREP task=T5
ESP32 -> K230: @TGT,0\r\n
TMX -> ESP32: @TASK ... type=TASK_BEGIN task=T5
TMX: 启动降速整圈循迹
ESP32: 启动O点滚球闭环
上位机: 开始录像
```

### 10.2 整圈运行

- TMX复用T2的巡线、丢线保护、编码器和A点横线识别。
- ESP32复用T4的 `targetMm=0`滚球闭环，并在直线和两个半圆弯道中持续控制。
- 当前整圈里程约为 `10.2k count`，该值可作为A点横线识别的备用里程窗口，不能单独替代横线确认。
- 任务总计时从按键启动开始，到小车完成一圈并通过A点时锁存。

T5要求“通过A点”，因此通过A点时先锁存整圈时间和A点里程，再继续巡线约16 cm摆正车身，最后执行安全停车。

```text
@CHECKPOINT eventSeq=50 type=PASS run=7 task=T5 point=A elapsed=25000 lapEnc=10200
@TASK eventSeq=51 repeat=1/3 type=TASK_END run=7 task=T5 result=DONE fault=NONE elapsed=25000 lapEnc=10500
```

T5通过条件：整圈锁存时间不超过30 s，且从启动到通过A点的全过程中小球相对O点误差绝对值不超过1 cm。

## 11. T6：降速循迹一圈并稳定在任意初始位置

T6复用T5的降速整圈循迹，区别是滚球目标不固定为O点，而是在T6准备页中从K230视觉反馈记录。

### 11.1 准备页使能和记录

进入T6准备页前，钢球由测试人员放在摆杆任意指定位置。TMX进入准备页后发送：

```text
TMX -> ESP32: @MODE ... type=ENTER mode=PREP task=T6
```

ESP32在准备页窗口内依次执行：

1. 保持底盘停止。
2. 使能D36A并让摆杆回到平衡位置，但不执行小球归中。
3. 向K230发送 `@TGT,0`，使K230以O点为固定像素参考。
4. 接收准备页期间的新鲜有效 `@VIS`。
5. 对最近少量稳定帧的 `E_PX`做轻量滤波，保存为候选参考 `referencePx`。

K230此时仍按以下公式发送误差：

```text
E_PX = BALL_X - O_X
```

因此准备页记录的 `referencePx`就是钢球任意初始位置相对O点的像素偏移，不需要把该位置换算成毫米，也不需要扩展K230协议。

准备页允许刷新候选参考值，便于放置钢球后等待其静止；一旦收到T6的 `TASK_BEGIN`，ESP32立即冻结最后一个有效 `referencePx`，任务运行期间禁止再次更新。

建议输出一次记录日志：

```text
@TARGET task=T6 state=CAPTURED referencePx=72 frame=1234
```

### 11.2 按键启动和控制误差

```text
TMX -> ESP32: @TASK ... type=TASK_BEGIN task=T6
TMX: 使用降速档启动整圈循迹
ESP32: 冻结referencePx并启动任意位置闭环
上位机: 开始录像
```

T6不直接使用K230相对O点的 `E_PX`作为闭环误差，而是计算：

```text
controlErrorPx = currentEpx - referencePx
```

例如准备页记录 `referencePx=+72`：

```text
currentEpx=+72 -> controlErrorPx=0   -> 钢球位于指定位置
currentEpx=+90 -> controlErrorPx=+18 -> 钢球偏向指定位置右侧
currentEpx=+50 -> controlErrorPx=-22 -> 钢球偏向指定位置左侧
```

后续仍使用现有的 `HOLD -> DRIVE -> BRAKE -> DAMP -> SETTLE`滚球外环和编码器位置内环，只把输入误差替换为 `controlErrorPx`。K230提供的速度 `V_PX_S`可以直接继续使用。

### 11.3 无有效参考时的行为

若准备页内始终没有获得有效、稳定的钢球位置，ESP32必须将T6标记为未准备好。此时不能用默认0点冒充任意目标，也不能在任务开始后持续改写参考值。应保持摆杆水平并输出明确故障日志：

```text
@TARGET task=T6 state=INVALID fault=NO_REFERENCE
```

### 11.4 整圈完成

TMX复用T5的降速循迹、A点通过判定和整圈计时。通过A点时先锁存时间，再继续巡线约16 cm摆正车身，随后安全停车并发送最终任务事件：

```text
@CHECKPOINT eventSeq=60 type=PASS run=8 task=T6 point=A elapsed=26000 lapEnc=10200
@TASK eventSeq=61 repeat=1/3 type=TASK_END run=8 task=T6 result=DONE fault=NONE elapsed=26000 lapEnc=10500
```

T6通过条件：整圈锁存时间不超过30 s，且行驶全过程中 `currentEpx-referencePx`对应的实际位置误差绝对值不超过1 cm。

## 12. 日志归类

上位机电机监控窗口固定分为三列：

| 列 | 日志 |
|---|---|
| 底盘 | `@CAR` |
| 滚球控制 | `@CTRL` |
| 任务事件 | `@LINK/@TASK/@MODE/@CHECKPOINT/@TARGET/@ENC`及其他 `@`协议 |

T3至T6运行期间，ESP32应周期发送紧凑 `@CTRL`，至少包含任务、阶段、目标、误差、速度、摆杆目标角、实际角和任务已用时间。T4至T6还应统计本次任务的最大位置误差；T6额外记录冻结的 `referencePx`，便于检查行驶全过程是否满足1 cm限制。

## 13. 任务编号迁移记录

新TMX固件中的 `task=T1`已经表示录像开关测试；旧固件日志仍需按旧编号解释。

| 项目 | 当前实现 |
|---|---|
| TMX任务编号 | T1录像测试、T2快速循迹、T3静止滚球、T4/T5/T6降速循迹 |
| TMX任务页面 | T1至T6均有准备页，TEST保持独立调试页面 |
| TMX循迹参数 | T2快速档与T4/T5/T6独立降速档 |
| ESP32视觉闭环范围 | T3运行目标序列；T4/T5固定目标0 mm；T6使用准备页冻结的像素参考 |
| ESP32 TEST页面 | 独立诊断页，不建立平衡零点、不输出STEP、不执行小球归中 |
| ESP32 UDP `9010`录像JSON通知 | 旧组件，当前构建不引用，上位机不监听 |
| ESP32到TMX的UART | 不实现回报，GPIO12已释放 |
| 上位机PREP事件去重键 | `eventSeq + type + mode + task + t` |
