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

/* ============================ LED 电流自动增益（AGC） ============================
 * 反射式 PPG 对贴合压力很敏感：
 *   压太紧 -> 皮肤表面反射增强、毛细血管被压闭，DC 高但 AC 几乎消失，
 *             且 DC 逼近 18 位满量程（262143）后进入非线性区
 *   太松   -> 光进不去组织，信号弱、噪声占比大
 * 所以不固定电流，而是按实测红外 DC 把它拉回量程中段（约 50%）。
 *
 * 目标区间 9 万~18 万（满量程的 34%~69%），中心取 13.5 万。
 * 每次按比例一步到位，收敛后落在死区内不再调整。
 * ============================================================================== */
#define MAX30102_LED_PA_RED		0x1E		/* 初始红光 6.0mA（与红外同档，避免 DC 失衡放大 R） */
#define MAX30102_LED_PA_IR		0x1E		/* 初始红外 6.0mA */
#define MAX30102_AGC_LO			90000		/* 低于此值：信号太暗 */
#define MAX30102_AGC_HI			180000		/* 高于此值：接近饱和 */
#define MAX30102_AGC_TARGET		135000		/* 目标直流 */
#define MAX30102_AGC_MIN		0x08		/* 1.6mA */
#define MAX30102_AGC_MAX		0x3F		/* 12.6mA */

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

/* LED 电流自动增益：传入当前红外直流，返回 1 表示电流已改变 */
u8   MAX30102_AgcStep(u32 dc_ir);
u8   MAX30102_GetLedPa(void);

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
