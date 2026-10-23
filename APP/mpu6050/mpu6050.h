#ifndef __MPU6050_H
#define __MPU6050_H

#include "system.h"

/* ============================ 硬件连接（原理图 U11） ============================
 * GY-521：VCC->VCC_5V（模块自带 3.3V 稳压），GND->GND
 *         SCL->PB6，SDA->PB7，AD0 悬空 -> 7 位地址 0x68（写 0xD0）
 *
 * 通信方式：硬件 I2C1（PB6/PB7 是 I2C1 的默认引脚，无需重映射）
 * 注意：工程必须包含 Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
 * ============================================================================ */
#define MPU6050_I2C				I2C1
#define MPU6050_SCL_PORT		GPIOB
#define MPU6050_SCL_PIN			GPIO_Pin_6
#define MPU6050_SDA_PORT		GPIOB
#define MPU6050_SDA_PIN			GPIO_Pin_7

#define MPU6050_ADDRESS			0xD0		/* 8 位写地址：0x68 << 1 */
#define MPU6050_I2C_SPEED		400000		/* 400kHz，手册上限 */

/* ============================ 量程与灵敏度（对应 Init 里的配置） ============================
 * GYRO_CONFIG = 0x18 -> FS_SEL=3  -> ±2000 dps，16.4 LSB/(°/s)
 * ACCEL_CONFIG= 0x18 -> AFS_SEL=3 -> ±16 g，2048 LSB/g
 * ============================================================================ */
#define MPU6050_ACCEL_LSB_PER_G		2048
#define MPU6050_GYRO_LSB_PER_DPS10	164			/* LSB/(°/s) × 10，便于整数运算 */


void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
uint8_t MPU6050_IsOnline(void);

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 数据加工（扩展接口，原有调用方式不受影响） */
void MPU6050_GetAccelMG(int16_t *AccX, int16_t *AccY, int16_t *AccZ);
void MPU6050_GetGyroDPS10(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
int16_t MPU6050_GetAccelMagnitudeMG(void);

#endif
