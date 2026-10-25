#ifndef _oled_H
#define _oled_H

#include "system.h"

/* ============================ 硬件连接（见原理图 U4） ============================
 * 0.96 寸 OLED 模块：VCC-5V  GND  SCL  SDA（4 线 IIC）
 * 本工程：SCL -> PB8，SDA -> PB9（核心板 28/27 脚，见原理图 U1），地址 0x78（7 位 0x3C 左移）
 * 注意：PB6/PB7 是 MPU6050 的 I2C，别和 OLED 混用
 * 说明：这里用软件 I2C（GPIO 模拟），不依赖 stm32f10x_i2c.c，换引脚只改下面两行
 * ============================================================================ */
#define OLED_SCL_PORT		GPIOB
#define OLED_SCL_PIN		GPIO_Pin_8
#define OLED_SDA_PORT		GPIOB
#define OLED_SDA_PIN		GPIO_Pin_9

#define OLED_ADDR			0x78		/* 模块地址（含读写位，写=0x78） */

/* 屏幕参数：128x64，8 页，每页 8 行 */
#define OLED_WIDTH			128
#define OLED_HEIGHT			64
#define OLED_PAGES			8

/* 字体高度选择 */
#define OLED_FONT_8			8			/* 6x8 小字 */
#define OLED_FONT_16		16			/* 8x16 大字 */

/* 画笔颜色 */
#define OLED_BLACK			0
#define OLED_WHITE			1


void OLED_Init(void);
void OLED_ClearBuffer(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_RefreshPages(u8 from, u8 to);		/* 只刷指定页范围，用于局部动态区域 */

/* 2 倍放大的数字（16x32）：用 8x16 字模逐像素放大，不占额外字库空间 */
void OLED_ShowChar2x(u8 x, u8 y, char ch);
void OLED_ShowNum2x(u8 x, u8 y, s32 num);

/* 8x8 单色图标，icon[0] 是最上面一行，bit7 在左 */
void OLED_DrawIcon8x8(u8 x, u8 y, const u8 *icon);

/* 页码指示器：total 个 4x4 小方块，第 cur 个实心 */
void OLED_DrawPageDots(u8 x, u8 y, u8 total, u8 cur);

void OLED_SetPos(u8 x, u8 y);
void OLED_ShowChar(u8 x, u8 y, char ch, u8 size);
void OLED_ShowString(u8 x, u8 y, char *str, u8 size);
void OLED_ShowNum(u8 x, u8 y, s32 num, u8 size);
void OLED_Print(char *str);

void OLED_DrawPoint(u8 x, u8 y, u8 color);
void OLED_DrawLine(u8 x0, u8 y0, u8 x1, u8 y1, u8 color);
void OLED_DrawRect(u8 x0, u8 y0, u8 x1, u8 y1, u8 color);
void OLED_FillRect(u8 x0, u8 y0, u8 x1, u8 y1, u8 color);
void OLED_DrawCircle(u8 x0, u8 y0, u8 r, u8 color);

void OLED_SetContrast(u8 contrast);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_Inverse(u8 mode);


#endif
