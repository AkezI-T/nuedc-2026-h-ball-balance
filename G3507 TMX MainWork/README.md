# G3507 TMX MainWork

2026年电赛H题车载平衡滚球系统的MSPM0G3507主控工程。任务编号、通信链路和设备职责遵循仓库根目录的 `TASK_COMMUNICATION_FLOW.md`。

## CCS导入

工程必须从纯ASCII路径导入：

工程所在目录（路径必须为纯 ASCII，且不含中文字符）

1. CCS中已有同名项目时先移除，但不要删除磁盘内容。
2. 选择 `File -> Import Project(s)` / `Import CCS Projects`。
3. 项目名应为 `G3507_TMX_MainWork`。
4. 当前工具链为MSPM0 SDK `2.10.00.04`、SysConfig `1.26.2`、TI Clang `4.0.4.LTS`。
5. 调试使用 `SEGGER J-Link Emulator + MSPM0G3507`。

## TMX任务

| 任务 | TMX界面和底盘行为 |
|---|---|
| T1 | 自动录像开关测试；底盘始终停止 |
| T2 | 第一次快速循迹一圈；到达终点后直接用1.5 s软停滑行摆正 |
| T3 | 静止滚球 `+5 cm -> -5 cm`；底盘停止，TMX运行5 s后上报结束 |
| T4 | 独立降速档从A到B，B点约 `2700 count` 锁存时间后安全停车 |
| T5 | 独立降速档循迹一圈，通过A点后继续循迹约16 cm摆正车身 |
| T6 | 独立降速档循迹一圈，通过A点后继续循迹约16 cm摆正车身；滚球目标由ESP32在准备阶段记录 |
| TEST | 传感器和手动滚球调试，不占用T1～T6编号 |

T1～T6均使用相同页面流程：

```text
主菜单短按选择
  -> 双击进入准备页
  -> 短按正式启动
```

进入准备页会发送三次：

```text
@MODE source=TMX eventSeq=1 repeat=1/3 type=ENTER mode=PREP task=T2 t=10000
```

正式启动时先发送 `TASK_BEGIN`，随后发送 `PREP EXIT`。T1运行页短按结束录像测试；T2/T4/T5/T6由线路或里程自动结束，也可双击中止；T3固定运行5 s后自动进入DONE并由TMX上报结束。

T5/T6确认完成点后进入 `ALIGN`，继续按巡线误差闭环行驶 `300 count`，并在摆正段先降到100 PWM。T2不再额外执行这300 count，终点确认后直接进入软停，用软停滑行距离完成摆正。T2/T4/T5/T6正常结束时，均从进入软停时的左右轮实际PWM出发，用1.5 s分别线性降到0；PWM归零后执行普通Stop并发送 `TASK_END`，全程禁止TB6612短接抱刹。按实车A到B的 `1.5 m ≈ 2700 count`换算，`300 count ≈ 16.7 cm`，接近半个32 cm车身。完成时间在进入摆正或软停时锁存，不包含后续停车过程；`lapEnc`记录最终停车里程。

## 分任务速度

参数位于 `Components/APP/app_config.h`，不同任务独立保存：

| 任务 | 基础PWM | 最大PWM | 斜坡/5 ms | 说明 |
|---|---:|---:|---:|---|
| T1 | 0 | 0 | 0 | 底盘停止 |
| T2 | 320 | 600 | 30 | 当前快速档，实测约16.845 s/圈 |
| T3 | 0 | 0 | 0 | 底盘停止 |
| T4 | 250 | 420 | 18 | A到B降速档 |
| T5 | 220 | 420 | 18 | 整圈降速档，宏与T4独立 |
| T6 | 220 | 420 | 18 | 与T5共用全部底盘参数 |

T2使用 `T2_*`，T4使用 `T4_*`，T5/T6共用 `T5_*`。后续调整稳球任务不会改变已经验证的T2快速参数。

## 串口协议

TMX通过UART0 `PA10/TX`向车载ESP32 `GPIO11/RX`单向发送，格式为115200-8N1。硬件只连接PA10、GPIO11和共地，PA11不连接。

任务开始和结束各重复三次，间隔150 ms。同一事件除 `repeat` 外的快照字段不变：

```text
@TASK source=TMX eventSeq=2 repeat=1/3 type=TASK_BEGIN run=1 t=12100 task=T2 state=FOLLOW
@TASK source=TMX eventSeq=3 repeat=1/3 type=TASK_END run=1 t=28260 task=T2 result=DONE fault=NONE elapsed=16160 lapEnc=10650
```

T3由TMX发送 `TASK_BEGIN`，底盘保持停止；达到5000 ms后TMX发送三次 `TASK_END result=DONE elapsed=5000`。ESP32在该窗口内执行滚球闭环，但不再生成T3任务结束事件。

T4通过B点以及T5/T6通过A点时发送一次过点事件：

```text
@CHECKPOINT source=TMX eventSeq=4 type=PASS run=2 t=30000 task=T4 point=B elapsed=6500 lapEnc=2700
```

只有底盘运行中的T2/T4/T5/T6每150 ms发送 `@CAR`：

```text
@CAR schema=2 source=GRAY seq=1 t=12300 task=T2 ... state=FOLLOW ... lapEnc=100
```

接收端不得根据 `@CAR state`推断任务类型。任务事件按 `source + eventSeq + type + run`去重，模式事件按 `source + eventSeq + type + mode`去重，重复原始日志仍可显示和保存。

## 软件结构

```text
Components/
|- APP/   通用任务运行器、分任务巡线配置、菜单和四任务调度
|- BSP/   电机、编码器、I2C、八路探头、JY901S、UART日志和LCD
`- OS/    FreeRTOS V11.2 Cortex-M0+轻量内核
```

FreeRTOS任务周期为：`Control` 5 ms、`Sensor` 1 ms调度、`UI` 10 ms、`Log` 150 ms。八路探头每5 ms采样，JY901S每20 ms采样。所有任务均静态分配。

若系统进入永久停机钩子，可在CCS表达式窗口查看：`g_app_fatal_reason=1`表示断言失败，`2`表示任务栈溢出，`3`表示调度器启动失败；栈溢出时 `g_app_stack_overflow_task` 指向对应任务名。UART格式化缓冲位于静态区，避免长 `@CAR` 日志占用LogTask栈。

## SysConfig和接线

- ST7735S 1.8寸SPI屏幕
- K1单键菜单
- TB6612双路底盘电机
- 左右轮正交编码器
- I2C0：亚博八路巡线 + JY901S
- UART0：PA10向ESP32发送协议日志
- J-Link SWD：PA20/PA19

完整接线见 `docs/硬件接线与SysConfig配置.md`。首次接电机前必须架空车轮确认正转方向和编码器正负值。
