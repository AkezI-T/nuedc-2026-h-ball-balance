/**
 * @file    lcd.h
 * @brief   ST7735S 1.8寸 TFT-LCD 驱动头文件（MSPM0G3507 移植版）
 * @author  源方电子 → MSPM0G3507 移植
 * @date    2024.9.13（原版）→ 2026.7.4（MSPM0 移植）
 * @platform MSPM0G3507 + TI DriverLib + SysConfig
 * @interface 4线 SPI（硬件 SPI1）
 *
 * ==== SysConfig 配置说明 ====
 * 1. SPI 实例：name="SPI_LCD"，peripheral=SPI1，PB9=SCLK、PB8=MOSI、PB7=MISO
 *    帧格式=MOTO3，速率=20MHz
 * 2. GPIO 组：name="LCD"，port=PORTB，包含 4 个子引脚：
 *    RES(PB10)、DC(PB11)、CS(PB14)、BLK(PB26)
 *    ioStructure=SD（标准驱动），初始电平=HIGH
 *
 * SysConfig 会自动生成以下宏（在 ti_msp_dl_config.h 中）：
 *   SPI_LCD_INST        — SPI1 外设实例
 *   LCD_PORT            — 控制引脚端口（GPIOB）
 *   LCD_RES_PIN / LCD_DC_PIN / LCD_CS_PIN / LCD_BLK_PIN — 引脚位掩码
 */

#ifndef __LCD_H
#define __LCD_H

#include "ti_msp_dl_config.h"
#include "stdlib.h"

/* ==================== 类型定义 ==================== */
/* 原 STM32 驱动使用 u8/u16/u32，在此做兼容映射 */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

/* ==================== 引脚操作宏 ==================== */
/* SCL/SDA 由硬件 SPI1 管理，无需 GPIO 宏 */
/* RES/DC/CS/BLK 由 SysConfig LCD GPIO 组生成 */

#define LCD_RES_Clr()   DL_GPIO_clearPins(LCD_PORT, LCD_RES_PIN)
#define LCD_RES_Set()   DL_GPIO_setPins(LCD_PORT, LCD_RES_PIN)

#define LCD_DC_Clr()    DL_GPIO_clearPins(LCD_PORT, LCD_DC_PIN)
#define LCD_DC_Set()    DL_GPIO_setPins(LCD_PORT, LCD_DC_PIN)

#define LCD_CS_Clr()    DL_GPIO_clearPins(LCD_PORT, LCD_CS_PIN)
#define LCD_CS_Set()    DL_GPIO_setPins(LCD_PORT, LCD_CS_PIN)

#define LCD_BLK_Clr()   DL_GPIO_clearPins(LCD_PORT, LCD_BLK_PIN)
#define LCD_BLK_Set()   DL_GPIO_setPins(LCD_PORT, LCD_BLK_PIN)

/* ==================== 显示方向 ==================== */
#define USE_HORIZONTAL 2   /* 0/1=竖屏(128×160)  2/3=横屏(160×128) */

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif

/* ==================== RGB565 颜色宏 ==================== */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0xF81F
#define GRED           0xFFE0
#define GBLUE          0x07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0xBC40
#define BRRED          0xFC07
#define GRAY           0x8430
#define DARKBLUE       0x01CF
#define LIGHTBLUE      0x7D7C
#define GRAYBLUE       0x5458
#define LIGHTGREEN     0x841F
#define LGRAY          0xC618
#define LGRAYBLUE      0xA651
#define LBBLUE         0x2B12

/* 延时函数 LCD_DelayMs() 实现在 lcd.c 中，使用 DL_Common_delayCycles */

/* ==================== 函数声明 ==================== */
void LCD_GPIO_Init(void);                                  /* GPIO 初始化（SysConfig 已处理，通常为空） */
void LCD_Writ_Bus(u8 dat);                                 /* 硬件 SPI 写入一个字节 */
void LCD_WR_DATA8(u8 dat);                                 /* 写 8 位数据 */
void LCD_WR_DATA(u16 dat);                                 /* 写 16 位数据 */
void LCD_WR_REG(u8 dat);                                   /* 写命令寄存器 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);     /* 设置显示窗口 */
void LCD_Init(void);                                       /* LCD 初始化 */

/* 绘图 */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
void LCD_DrawPoint(u16 x, u16 y, u16 color);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color);

/* 6x12 / 8x16 ASCII 文字 */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);
u32 mypow(u8 m, u8 n);
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey);
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey);

/* 图片 */
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[]);

#endif /* __LCD_H */
