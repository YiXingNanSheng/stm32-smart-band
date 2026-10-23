#ifndef _ds18b20_H
#define _ds18b20_H

#include "system.h"

/* ============================ 硬件连接（原理图 U8） ============================
 * DS18B20：VDD -> VCC_5V，GND -> GND，DQ -> PA8，DQ 经 R11(10K) 上拉到 VCC_5V
 * 注意：PA8 是 5V 容忍(FT)引脚，才能直接挂 5V 上拉
 * ============================================================================ */
#define DS18B20_PORT			GPIOA
#define DS18B20_PIN				GPIO_Pin_8

/* ============================ 命令字（DS18B20 手册） ============================ */
#define DS18B20_CMD_SKIP_ROM		0xCC	/* 跳过 ROM：总线上只有一颗时用 */
#define DS18B20_CMD_CONVERT_T		0x44	/* 启动温度转换 */
#define DS18B20_CMD_READ_SCRATCH	0xBE	/* 读暂存器（9 字节） */
#define DS18B20_CMD_WRITE_SCRATCH	0x4E	/* 写暂存器（TH/TL/配置） */

/* ============================ 返回码 ============================ */
#define DS18B20_OK					0		/* 成功 */
#define DS18B20_ERR_NODEV			1		/* 无器件应答（接线/供电） */
#define DS18B20_ERR_CRC				2		/* CRC 校验失败 */
#define DS18B20_ERR_RANGE			3		/* 数据超出 -55~125℃ */
#define DS18B20_ERR_NOTCONV			4		/* 读到 85℃，转换尚未完成 */


u8  DS18B20_Init(void);
u8  DS18B20_Reset(void);
u8  DS18B20_StartConvert(void);
u8  DS18B20_ReadTemp(s16 *temp_x10);
u8  DS18B20_ReadTempNow(s16 *temp_x10);
u8  DS18B20_SetResolution(u8 bits);


#endif
