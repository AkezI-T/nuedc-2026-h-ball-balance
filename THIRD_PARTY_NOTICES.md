# 第三方来源与许可范围

- 根目录`LICENSE`沿用原工程MIT文本，适用于该文本覆盖的原创代码部分。
- `K230StandaloneMjpeg/`的上游来源和AFL-3.0全文分别保留在该目录的README和LICENSE中。该子工程为可选独立图传，不能改写为根目录MIT许可。
- `G3507 TMX MainWork/Components/OS/`含FreeRTOS Kernel V11.2.0源码，原文件MIT版权与许可头完整保留。其余内核头文件和DriverLib依赖本地TI SDK，不在本包内复制SDK安装目录。
- MSPM0工程使用TI SDK的DriverLib、CMSIS及示例工程配置；保留已有文件声明，构建时自行安装项目记录的SDK版本。
- `G3507 TMX MainWork/Components/BSP/LCD/lcd.c`和`lcd.h`文件头写明“源方电子”来源，`lcdfont.h`说明从TianMengXinTest_v2复用。仓库内未找到这些LCD/字库原始分发包的完整授权说明；已保留文件头，公开发布前仍需补充可追溯的来源及授权文件，不据此声称这些第三方文件全部为原创MIT代码。
- 原题与两份分析报告的材料属性见`docs/README.md`。

这是本次基于仓库文件的来源盘点，不是对全部第三方权利的完备审查。
