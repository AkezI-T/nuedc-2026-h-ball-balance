/**
 * @file    lcd.c
 * @brief   ST7735S 1.8寸 TFT-LCD 驱动实现（MSPM0G3507 移植版）
 * @author  源方电子 → MSPM0G3507 移植
 * @date    2024.9.13（原版）→ 2026.7.4（MSPM0 移植）
 * @platform MSPM0G3507 + TI DriverLib
 * @interface 4线 SPI（硬件 SPI1 + GPIO 控制引脚）
 *
 * ==== 与 STM32 原版的差异 ====
 * 1. GPIO 操作：GPIO_ResetBits/SetBits → DL_GPIO_clearPins/setPins
 * 2. SPI 通信：软件 GPIO bit-bang → 硬件 SPI1（DL_SPI_transmitData8，20MHz）
 * 3. GPIO 初始化：SysConfig 完成，LCD_GPIO_Init() 为空
 * 4. 数据类型：在 lcd.h 中 typedef uint8_t → u8 等
 * 5. 延时函数：LCD_DelayMs() 使用 DL_Common_delayCycles 忙等（不依赖 SysTick）
 */

#include "lcd.h"
#include "lcdfont.h"
#include "stdlib.h"

/* ==================== 延时 ==================== */
/**
 * @brief  毫秒延时（忙等待，不依赖 SysTick / FreeRTOS）
 * @note   在 LCD_Init 硬件初始化阶段使用（调度器尚未启动）。
 *         调度器启动后，上层应用应使用 vTaskDelay()。
 */
static void LCD_DelayMs(u32 ms)
{
    DL_Common_delayCycles(CPUCLK_FREQ / 1000 * ms);
}

/* ==================== GPIO 初始化 ==================== */
void LCD_GPIO_Init(void)
{
    /* SysConfig 已处理，无额外操作 */
}

/* ==================== 硬件 SPI 底层通信 ==================== */

/**
 * @brief  硬件 SPI1 写入一个字节
 * @param  dat 要写入的字节
 * @note   CS 手动控制（拉低→发送→拉高）
 *         SCL/MOSI 由 SPI1 外设自动管理
 */
void LCD_Writ_Bus(u8 dat)
{
    uint8_t dummy;

    LCD_CS_Clr();

    DL_SPI_transmitData8(SPI_LCD_INST, dat);  /* SPI1 发送 */
    while (DL_SPI_isBusy(SPI_LCD_INST));      /* 等待发送完成 */

    dummy = DL_SPI_receiveData8(SPI_LCD_INST); /* 清空接收缓冲 */
    while (DL_SPI_isBusy(SPI_LCD_INST));

    LCD_CS_Set();
}

/**
 * @brief  LCD 写 8 位数据
 */
void LCD_WR_DATA8(u8 dat)
{
    LCD_Writ_Bus(dat);
}

/**
 * @brief  LCD 写 16 位数据（高字节先发）
 */
void LCD_WR_DATA(u16 dat)
{
    LCD_Writ_Bus(dat >> 8);
    LCD_Writ_Bus(dat);
}

/**
 * @brief  LCD 写命令寄存器
 * @note   DC=0 表示写命令，DC=1 表示写数据
 */
void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();  /* 命令模式 */
    LCD_Writ_Bus(dat);
    LCD_DC_Set();  /* 切回数据模式 */
}

/* ==================== 显示窗口设置 ==================== */

/**
 * @brief  设置 ST7735S 的列/行地址窗口
 * @note   0x2A=列地址(CASET), 0x2B=行地址(RASET), 0x2C=内存写(RAMWR)
 *         偏移量 (+1/+2) 由屏幕面板的物理偏移决定
 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    if (USE_HORIZONTAL == 0)
    {
        LCD_WR_REG(0x2A);  /* 列地址设置 */
        LCD_WR_DATA(x1 + 2);
        LCD_WR_DATA(x2 + 2);
        LCD_WR_REG(0x2B);  /* 行地址设置 */
        LCD_WR_DATA(y1 + 1);
        LCD_WR_DATA(y2 + 1);
        LCD_WR_REG(0x2C);  /* 内存写 */
    }
    else if (USE_HORIZONTAL == 1)
    {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1 + 2);
        LCD_WR_DATA(x2 + 2);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1 + 1);
        LCD_WR_DATA(y2 + 1);
        LCD_WR_REG(0x2C);
    }
    else if (USE_HORIZONTAL == 2)
    {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1 + 1);
        LCD_WR_DATA(x2 + 1);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1 + 2);
        LCD_WR_DATA(y2 + 2);
        LCD_WR_REG(0x2C);
    }
    else
    {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1 + 1);
        LCD_WR_DATA(x2 + 1);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1 + 2);
        LCD_WR_DATA(y2 + 2);
        LCD_WR_REG(0x2C);
    }
}

/* ==================== ST7735S 初始化序列 ==================== */

void LCD_Init(void)
{
    LCD_GPIO_Init();  /* 初始化 GPIO（若已由 SysConfig 初始化，可跳过） */

    LCD_RES_Clr();    /* 硬件复位 */
    LCD_DelayMs(100);
    LCD_RES_Set();
    LCD_DelayMs(100);

    LCD_BLK_Set();    /* 打开背光 */
    LCD_DelayMs(100);

    /* ---------- Start Initial Sequence ---------- */
    LCD_WR_REG(0x11);           /* Sleep Out */
    LCD_DelayMs(120);              /* 等待 120ms */

    /* ---------- ST7735S Frame Rate ---------- */
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    /* ---------- End ST7735S Frame Rate ---------- */

    LCD_WR_REG(0xB4);           /* Dot Inversion */
    LCD_WR_DATA8(0x03);

    /* ---------- ST7735S Power Sequence ---------- */
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1);
    LCD_WR_DATA8(0xC0);
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0xEE);
    /* ---------- End ST7735S Power Sequence ---------- */

    LCD_WR_REG(0xC5);           /* VCOM Control */
    LCD_WR_DATA8(0x1A);

    LCD_WR_REG(0x36);           /* MADCTL: 内存数据访问控制（旋转/镜像） */
    if (USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);
    else if (USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if (USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else                          LCD_WR_DATA8(0xA0);

    /* ---------- ST7735S Gamma Sequence (正极性) ---------- */
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x22);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x30);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x13);

    /* ---------- ST7735S Gamma Sequence (负极性) ---------- */
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x13);
    /* ---------- End ST7735S Gamma Sequence ---------- */

    LCD_WR_REG(0x3A);           /* COLMOD: 像素格式 */
    LCD_WR_DATA8(0x05);         /* 16-bit/pixel (RGB565, 65K 色) */
    LCD_WR_REG(0x29);           /* Display On */
}

/* ==================== 基础绘图 ==================== */

/**
 * @brief  指定区域填充颜色
 */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    u16 i, j;
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);  /* 设置显示窗口 */
    for (i = ysta; i < yend; i++)
    {
        for (j = xsta; j < xend; j++)
        {
            LCD_WR_DATA(color);
        }
    }
}

/**
 * @brief  在指定位置画一个点
 */
void LCD_DrawPoint(u16 x, u16 y, u16 color)
{
    LCD_Address_Set(x, y, x, y);  /* 设置光标位置 */
    LCD_WR_DATA(color);
}

/**
 * @brief  画线（Bresenham 算法）
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;  /* 计算坐标增量 */
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)      incx = 1;      /* 设置单步方向 */
    else if (delta_x == 0) incx = 0;     /* 垂直线 */
    else { incx = -1; delta_x = -delta_x; }
    if (delta_y > 0)      incy = 1;
    else if (delta_y == 0) incy = 0;     /* 水平线 */
    else { incy = -1; delta_y = -delta_y; }
    if (delta_x > delta_y) distance = delta_x;  /* 选取基本增量坐标轴 */
    else                   distance = delta_y;
    for (t = 0; t < distance + 1; t++)
    {
        LCD_DrawPoint(uRow, uCol, color);  /* 画点 */
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/**
 * @brief  画矩形边框
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

/**
 * @brief  画圆（中点画圆算法）
 */
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color)
{
    int a, b;
    a = 0;
    b = r;
    while (a <= b)
    {
        LCD_DrawPoint(x0 - b, y0 - a, color);   /* 第 3 象限 */
        LCD_DrawPoint(x0 + b, y0 - a, color);   /* 第 0 象限 */
        LCD_DrawPoint(x0 - a, y0 + b, color);   /* 第 1 象限 */
        LCD_DrawPoint(x0 - a, y0 - b, color);   /* 第 2 象限 */
        LCD_DrawPoint(x0 + b, y0 + a, color);   /* 第 4 象限 */
        LCD_DrawPoint(x0 + a, y0 - b, color);   /* 第 5 象限 */
        LCD_DrawPoint(x0 + a, y0 + b, color);   /* 第 6 象限 */
        LCD_DrawPoint(x0 - b, y0 + a, color);   /* 第 7 象限 */
        a++;
        if ((a * a + b * b) > (r * r))  /* 判断要画的点是否过远 */
        {
            b--;
        }
    }
}

/* ==================== 6x12 / 8x16 ASCII 文字显示 ==================== */

/**
 * @brief  显示一个 ASCII 字符
 * @param  sizey 当前轻量版本支持 12/16
 * @param  mode  0=非叠加模式  1=叠加模式
 */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 temp, sizex, t, m = 0;
    u16 i, TypefaceNum;
    u16 x0 = x;

    if ((sizey != 12U) && (sizey != 16U)) return;
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';    /* 得到偏移后的值 */
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);  /* 设置光标位置 */
    for (i = 0; i < TypefaceNum; i++)
    {
        temp = (sizey == 12U) ? ascii_1206[num][i] : ascii_1608[num][i];
        for (t = 0; t < 8; t++)
        {
            if (!mode)  /* 非叠加模式 */
            {
                if (temp & (0x01 << t))
                    LCD_WR_DATA(fc);
                else
                    LCD_WR_DATA(bc);
                m++;
                if (m % sizex == 0)
                {
                    m = 0;
                    break;
                }
            }
            else  /* 叠加模式 */
            {
                if (temp & (0x01 << t))
                    LCD_DrawPoint(x, y, fc);  /* 画一个点 */
                x++;
                if ((x - x0) == sizex)
                {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

/**
 * @brief  显示字符串
 */
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    while (*p != '\0')
    {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/**
 * @brief  求 m 的 n 次方
 */
u32 mypow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief  显示整数
 * @param  len 要显示的位数
 */
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t, temp;
    u8 enshow = 0;
    u8 sizex = sizey / 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            }
            else enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/**
 * @brief  显示两位小数的浮点数
 * @param  len 要显示的总位数（含小数点）
 * @note   固定保留两位小数
 */
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t, temp, sizex;
    u16 num1;
    sizex = sizey / 2;
    num1 = num * 100;
    for (t = 0; t < len; t++)
    {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if (t == (len - 2))
        {
            LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/* ==================== 图片显示 ==================== */

/**
 * @brief  显示图片（从 FLASH 数组读取）
 * @param  length 图片宽度（像素）
 * @param  width  图片高度（像素）
 * @param  pic[]  图片数据数组（RGB565 格式，每像素 2 字节）
 */
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[])
{
    u16 i, j;
    u32 k = 0;
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    for (i = 0; i < length; i++)
    {
        for (j = 0; j < width; j++)
        {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}

