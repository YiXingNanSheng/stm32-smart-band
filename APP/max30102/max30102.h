#ifndef __MAX30102_H
#define __MAX30102_H

#include "system.h"

/* ============================ 硬件连接（原理图 U12） ============================
 * 板载 MAX30102 座子，插模块即可，无需飞线
 *   1 VIN -> VCC_5V      2 SCL -> PB10      3 SDA -> PB11
 *   4 INT -> PB2         7 GND -> GND
 *
 * 通信方式：硬件 I2C2（PB10/PB11 是 I2C2 默认引脚，无需重映射），400kHz
 * 注意 1：I2C2 的时钟在 APB1，不是 APB2
 * 注意 2：INT 接的是 PB2，而 PB2 是 BOOT1，上电为高会进 ISP 模式导致程序不跑
 *         所以第一版不接 INT，全部用轮询读 FIFO
 * 地址：0xAE 写 / 0xAF 读（7 位 0x57）
 * ============================================================================ */
#define MAX30102_I2C			I2C2
#define MAX30102_SCL_PORT		GPIOB
#define MAX30102_SCL_PIN		GPIO_Pin_10
#define MAX30102_SDA_PORT		GPIOB
#define MAX30102_SDA_PIN		GPIO_Pin_11

#define MAX30102_ADDRESS		0xAE		/* 8 位写地址：0x57 << 1 */
#define MAX30102_I2C_SPEED		400000		/* 400kHz */

/* ============================ 寄存器地址 ============================ */
#define REG_INTR_STATUS_1		0x00
#define REG_INTR_STATUS_2		0x01
#define REG_INTR_ENABLE_1		0x02
#define REG_INTR_ENABLE_2		0x03
#define REG_FIFO_WR_PTR			0x04
#define REG_OVF_COUNTER			0x05
#define REG_FIFO_RD_PTR			0x06
#define REG_FIFO_DATA			0x07
#define REG_FIFO_CONFIG			0x08
#define REG_MODE_CONFIG			0x09
#define REG_SPO2_CONFIG			0x0A
#define REG_LED1_PA				0x0C
#define REG_LED2_PA				0x0D
#define REG_PILOT_PA			0x10
#define REG_MULTI_LED_CTRL1		0x11
#define REG_MULTI_LED_CTRL2		0x12
#define REG_TEMP_INTR			0x1F
#define REG_TEMP_FRAC			0x20
#define REG_TEMP_CONFIG			0x21
#define REG_PROX_INT_THRESH		0x30
#define REG_REV_ID				0xFE
#define REG_PART_ID				0xFF

#define MAX30102_PART_ID_VALUE	0x15		/* 读 0xFF 应该得到这个值 */

/* ============================ 寄存器读写 ============================ */
u8 MAX30102_WriteReg(u8 reg, u8 data);
u8 MAX30102_ReadReg(u8 reg, u8 *data);
u8 MAX30102_ReadRegs(u8 reg, u8 *buf, u8 len);

/* ============================ 初始化与自检 ============================ */
void MAX30102_Init(void);
u8   MAX30102_GetPartID(void);
u8   MAX30102_IsOnline(void);

/* ============================ FIFO ============================ */
void MAX30102_ClearFIFO(void);
u8   MAX30102_GetFIFOWritePtr(void);
u8   MAX30102_GetFIFOReadPtr(void);
u8   MAX30102_ReadFIFO(u32 *red, u32 *ir);

/* ============================ 片内温度（可选） ============================ */
u8   MAX30102_ReadTemperature(s16 *temp10);		/* 单位 0.1℃ */

/* ============================ 采样缓冲 ============================
 * 硬件 FIFO 只有 32 格，25Hz 采样下 1.3 秒就写满。
 * 主循环每轮调用 MAX30102_Poll() 把样本搬进软件环形缓冲，
 * 保证读到的永远是新鲜数据，而不是十几秒前的旧值。
 * ================================================================ */
#define MAX30102_BUF_SIZE	128

typedef struct
{
	u32 red;
	u32 ir;
} MAX30102_Sample_t;

void MAX30102_ResetBuffer(void);
u8   MAX30102_Poll(void);						/* 主循环每轮调用，返回本次搬了几组 */
u16  MAX30102_Buffered(void);					/* 缓冲里还有几组没取 */
u8   MAX30102_Pop(MAX30102_Sample_t *s);		/* 取一组，0=成功 */
u16  MAX30102_GetOverflow(void);				/* 软件缓冲覆盖次数 */

#endif
