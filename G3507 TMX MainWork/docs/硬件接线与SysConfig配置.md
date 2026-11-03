# 硬件接线与 SysConfig 配置

## 1. 本阶段范围

T1 对应 H 题基本要求第 2 项：小车从 A 点按键启动，沿黑线顺时针行驶一圈，停回 A 点并显示总时间。指标是总时间不超过 20 s，停车偏差不超过 2 cm。赛题还要求巡迹只使用红外光电模块，并配置启动按键和不大于 2 英寸的显示屏。

当前已接入 FreeRTOS 四任务、巡线 PD、A 点启停线识别、一圈状态机、计时显示和 ESP32 日志。首次烧录前仍应断开电机电池，先验证低压外设和方向配置。

## 2. 总体连接图

```text
                       +----------------------+
 J-Link SWD ---------->|                      |---- SPI ----> ST7735S 1.8寸屏
 PA20/PA19             |   天猛星 MSPM0G3507   |---- GPIO ---> K1启动键
                       |                      |---- PWM/GPIO -> TB6612 -> 左/右电机
 八路巡线 0x12 ----+-->| I2C0 PA28/PA31     |<--- GPIO ---- 左/右编码器
 JY901S  0x50 ------+   |                      |
                       | UART0 PA10/PA11     |---- TTL ----> ESP32日志
                       +----------------------+

 电机电池 ----> TB6612 VM
 3.3V/5V逻辑供电 -> 各低压模块
 所有模块 GND ---------------------------- 必须共地
```

## 3. SysConfig 资源表

| SysConfig 名称 | 外设 | 引脚 | 配置 |
|---|---|---|---|
| `LED` | 状态 LED | PB22 | GPIO 输出 |
| `LCD` | 屏幕控制 | PB10=RES, PB11=DC, PB14=CS, PB26=BLK | GPIO 输出 |
| `SPI_LCD` | 屏幕 SPI | PB9=SCLK, PB8=MOSI, PB7=MISO保留 | SPI1, MOTO3, 8 MHz |
| `KEY` | T1 启动/菜单 | PB21=K1 | 上拉输入，低有效 |
| `TB6612` | 电机方向 | PA7=AIN1, PA14=AIN2, PA22=BIN1, PA15=BIN2 | GPIO 输出 |
| `STBY` | 电机总使能 | PB23 | GPIO 输出，上电默认低 |
| `PWM_TB6612` | 双路电机 PWM | PB1=PWMA, PB0=PWMB | TIMA1 CCP1/CCP0 |
| `ENC1` | 左轮编码器 | PB6=E1A, PB27=E1B | GPIO 输入 |
| `ENC2` | 右轮编码器 | PA12=E2A, PA13=E2B | GPIO 输入 |
| `I2C_GRAY` | 巡线 + JY901S | PA28=SDA, PA31=SCL | I2C0, 100 kHz |
| `UART_DEBUG` | ESP32 日志 | PA10=TX, PA11=RX | UART0, 115200-8N1 |
| `TIMER_CONTROL` | 裸机时基预留 | 无外部引脚 | TIMG8, 1 ms；FreeRTOS 版本不启动 |
| `DEBUGSS` | J-Link SWD | PA20=SWCLK, PA19=SWDIO | 调试烧录 |

## 4. 分模块接线图

### 4.1 J-Link 与天猛星

```text
J-Link               天猛星 MSPM0G3507
VTref 3.3V  --------> 3V3
GND         --------> GND
SWDIO       --------> PA19 / DIO
SWCLK       --------> PA20 / CLK
RESET       --------> RST        (建议接)
```

只把 VTref 用作电平参考，不用 J-Link 给整车供电。PA19/PA20 不分配给其他外设。

### 4.2 ST7735S 1.8 寸屏

```text
ST7735S             MSPM0G3507
VCC       --------> 3V3
GND       --------> GND
SCL/SCK   --------> PB9  (SPI1 SCLK)
SDA/MOSI  --------> PB8  (SPI1 MOSI)
RES/RST   --------> PB10
DC/A0     --------> PB11
CS        --------> PB14
BL/BLK    --------> PB26
```

屏幕是只写接口，不需要连 MISO；SysConfig 为 SPI1 保留 PB7，便于沿用现有驱动定义。参考工程对屏幕引脚的定义见 `TMX MSPM0G3507 Test/TianMengXinTest_v2/Components/BSP/LCD/lcd.h:9`。

### 4.3 K1 启动键

使用天猛星板载 K1（PB21）时无需外接。如使用外置按键：

```text
PB21 ---- 按键 ---- GND
```

PB21 已开启内部上拉，按下为低电平。

### 4.4 亚博八路巡线模块

```text
八路巡线模块       MSPM0G3507
5V/VCC       --------> 5V
GND          --------> GND
SDA          --------> PA28 (I2C0 SDA)
SCL          --------> PA31 (I2C0 SCL)
```

- 使用 I2C 7 位地址 `0x12`，数字状态寄存器 `0x30`；亚博代码的地址和位顺定义见 `【Yahboom】MSPM0G3507 扩展板 电机驱动板/八路巡线红外传感器/2.小车案例/6.MSPM0-轮式小车底盘V2/八路巡线模块/八路巡线模块代码/CCS/EightWayTrack/BSP/Eight_Tracking/app_irtracking.h:13` 和 `app_irtracking.c:73`。
- 亚博文档的 PA0/PA1 只是原例程接线，本工程不使用；必须按上表接 PA28/PA31。
- 首次使用按模块 KEY 完成黑/白场标定；标定值可断电保存。模块上电后需先稳定，再允许 T1 发车。

### 4.5 JY901S I2C

```text
JY901S               MSPM0G3507
VCC         --------> 3V3 (推荐)
GND         --------> GND
SDA         --------> PA28 (I2C0 SDA)
SCL         --------> PA31 (I2C0 SCL)
```

JY901S 与巡线模块并联在同一 I2C 总线，地址不冲突。参考工程使用 `0x50` 和角度寄存器 `0x3D`，见 `TMX MSPM0G3507 Test/TianMengXinTest_v2/Components/BSP/GyroSensor/bsp_gyrosensor.c:9`。

不建议为 JY901S 再占用一组 I2C 引脚；两个设备应由同一个非阻塞总线调度器串行访问。

### 4.6 ESP32 日志串口

```text
MSPM0G3507                 ESP32
PA10 / UART0_TX  --------> RX
PA11 / UART0_RX  <-------- TX   (需要双向命令时再接)
GND              --------> GND
```

串口电平必须是 3.3 V TTL，不得接 RS-232 电平。如果只发日志，只接 PA10、ESP32 RX 和共地即可。

### 4.7 TB6612 与双电机

```text
MSPM0G3507           TB6612              电机
3V3       ---------> VCC
GND       ---------> GND
PB23      ---------> STBY
PB1       ---------> PWMA ---- A01/A02 -> 左电机
PA7       ---------> AIN1
PA14      ---------> AIN2
PB0       ---------> PWMB ---- B01/B02 -> 右电机
PA22      ---------> BIN1
PA15      ---------> BIN2
电机电池 + ---------> VM
电机电池 - ---------> GND
```

TB6612 引脚与参考工程一致，见 `TMX MSPM0G3507 Test/TianMengXinTest_v2/Components/BSP/TB6612/bsp_tb6612.h:5`。左右电机命名是软件约定；首次运动前必须架空车轮，确认正转方向与编码器计数符号。

### 4.8 左右轮编码器

```text
左轮编码器              MSPM0G3507
VCC            --------> 3V3
GND            --------> GND
A              --------> PB6
B              --------> PB27

右轮编码器              MSPM0G3507
VCC            --------> 3V3
GND            --------> GND
A              --------> PA12
B              --------> PA13
```

参考工程对编码器引脚的定义见 `TMX MSPM0G3507 Test/TianMengXinTest_v2/Components/BSP/Encoder/bsp_encoder.h:5`。如编码器只能 5 V 供电且 A/B 输出高电平为 5 V，必须加电平转换，不能直接 MSPM0G3507。

## 5. I2C 电平与上拉检查

亚博模块文档按 5 V 供电，JY901S 推荐 3.3 V 供电。两个模块并联前，先单独给亚博模块上电，用万用表测 SDA/SCL 空闲高电平：

- 约 3.3 V：可与 MSPM0G3507/JY901S 直接共享总线。
- 高于 3.6 V：先加双向 I2C 电平转换，不得直接 PA28/PA31。
- 并联后如出现上升沿过快、无法拉低或总线占用，检查两块板的上拉电阻是否并联过小。

## 6. 供电与首次上电顺序

1. 先断开电机电池，只给 MSPM0G3507、屏幕、巡线模块和 JY901S 上电。
2. 检查所有 GND 共地，检查 3.3 V/5 V 没有反接。
3. 检查 I2C 空闲高电平，确认不超过 3.6 V。
4. 接上 J-Link，编译、下载后确认屏幕进入 T1 `READY` 页面。
5. 用逻辑分析仪确认 PB9 SPI、PA28/PA31 I2C 和 PA10 UART 波形；检查屏幕 `LINE` 数据会随黑线位置变化。
6. 架空车轮后再接电机电池。未按 K1 时 PB23/STBY 必须为低，按 K1 后确认两轮都向车头方向转。
7. 如方向相反，编码器符号修改 `Components/APP/app_config.h` 中的 `APP_ENCODER_LEFT_INVERT` 与 `APP_ENCODER_RIGHT_INVERT`；电机转向由 `Components/BSP/bsp_motor.c` 中 `set_motor_a` 与 `set_motor_b` 输出的 IN1/IN2 电平组合决定。

## 7. 首次 CCS 验收清单

- 从本工程目录导入后，CCS 项目名是 `G3507_TMX_MainWork`。
- SysConfig 界面无红色引脚冲突。
- Build 后 `Debug/ti_msp_dl_config.h` 包含 `I2C_GRAY`、`UART_DEBUG`、`PWM_TB6612` 和 `TIMER_CONTROL`。
- Target Configuration 显示 `SEGGER J-Link Emulator -> MSPM0G3507`。
- 下载后屏幕显示 T1 页面，系统不进入 HardFault 或 FreeRTOS 断言。
- READY 状态电机保持停止，TB6612 STBY 为低。
- PA10 日志持续输出，`raw` 随巡线探头变化，JY901S 在线时 `yaw10` 会变化。
