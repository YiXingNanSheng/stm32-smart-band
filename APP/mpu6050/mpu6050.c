#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "SysTick.h"

/* ============================ 硬件 I2C1 底层 ============================
 * PB6=SCL、PB7=SDA，复用开漏，400kHz
 * 所有等待事件都带超时，避免 F103 硬件 I2C 偶发卡死在 BUSY 状态
 * ==================================================================== */
#define I2C_TIMEOUT		2000

/* 等待某个事件，返回 0=成功 1=超时 */
static uint8_t MPU6050_WaitEvent(uint32_t Event)
{
	uint16_t t = I2C_TIMEOUT;

	while(!I2C_CheckEvent(MPU6050_I2C, Event))
	{
		if(--t == 0)
			return 1;
	}

	return 0;
}

/* 超时后兜底：产生停止条件并复位外设，避免总线被占死 */
static void MPU6050_I2C_Recover(void)
{
	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
	I2C_SoftwareResetCmd(MPU6050_I2C, ENABLE);
	I2C_SoftwareResetCmd(MPU6050_I2C, DISABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
}


/* 写一个寄存器：START -> 地址W -> 寄存器号 -> 数据 -> STOP */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))          { MPU6050_I2C_Recover(); return; }

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { MPU6050_I2C_Recover(); return; }

	I2C_SendData(MPU6050_I2C, RegAddress);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING))     { MPU6050_I2C_Recover(); return; }

	I2C_SendData(MPU6050_I2C, Data);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))      { MPU6050_I2C_Recover(); return; }

	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
}

/* 连续读多个寄存器：START -> 地址W -> 寄存器号 -> 重复START -> 地址R -> 连续读 -> STOP */
static uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))          { MPU6050_I2C_Recover(); return 1; }

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { MPU6050_I2C_Recover(); return 1; }

	I2C_SendData(MPU6050_I2C, RegAddress);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING))     { MPU6050_I2C_Recover(); return 1; }

	/* 重复起始，切换为接收 */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))          { MPU6050_I2C_Recover(); return 1; }

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDRESS, I2C_Direction_Receiver);
	if(MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) { MPU6050_I2C_Recover(); return 1; }

	for(i = 0; i < len; i++)
	{
		if(i == (uint8_t)(len - 1))
		{
			/* 最后一个字节：先关 ACK 并置 STOP，再读数据 */
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		}

		if(MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
			MPU6050_I2C_Recover();
			return 1;
		}

		buf[i] = I2C_ReceiveData(MPU6050_I2C);
	}

	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);

	return 0;
}

/* 读一个寄存器 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t data = 0xFF;

	if(MPU6050_ReadRegs(RegAddress, &data, 1))
		return 0xFF;

	return data;
}


/* ============================ 初始化 ============================ */
void MPU6050_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* PB6/PB7：I2C1 默认引脚，复用开漏 */
	GPIO_InitStructure.GPIO_Pin   = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 先 DeInit，清掉上一次可能残留的 BUSY 状态 */
	I2C_DeInit(MPU6050_I2C);

	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0x30;		/* 主机地址，任意值 */
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = MPU6050_I2C_SPEED;
	I2C_Init(MPU6050_I2C, &I2C_InitStructure);

	I2C_Cmd(MPU6050_I2C, ENABLE);
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);

	delay_ms(50);						/* 模块上电稳定 */

	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		/* 解除休眠，时钟源=PLL X 轴陀螺 */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		/* 六轴全部使能 */
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		/* 1kHz/(1+9)=100Hz 采样 */
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			/* DLPF 5Hz */
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	/* 陀螺 ±2000 dps */
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	/* 加速度 ±16 g */
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_IsOnline(void)
{
	return (MPU6050_GetID() == 0x68) ? 1 : 0;
}


/* ============================ 采集：六轴原始数据 ============================ */
/* 一次突发读 0x3B 起的 14 字节（加速度 6 + 温度 2 + 陀螺 6），
 * 温度字节跳过不用；相比 12 次单字节读，速度快 5 倍以上 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t buf[14];

	if(MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14))
		return;							/* 读失败保持原值，不写野数据 */

	if(AccX)  *AccX  = (int16_t)((buf[0]  << 8) | buf[1]);
	if(AccY)  *AccY  = (int16_t)((buf[2]  << 8) | buf[3]);
	if(AccZ)  *AccZ  = (int16_t)((buf[4]  << 8) | buf[5]);

	/* buf[6]、buf[7] 是温度，按需求丢弃 */

	if(GyroX) *GyroX = (int16_t)((buf[8]  << 8) | buf[9]);
	if(GyroY) *GyroY = (int16_t)((buf[10] << 8) | buf[11]);
	if(GyroZ) *GyroZ = (int16_t)((buf[12] << 8) | buf[13]);
}


/* ============================ 数据处理（单位换算） ============================ */

/* 加速度换算成 mg：mg = raw * 1000 / 2048 */
void MPU6050_GetAccelMG(int16_t *AccX, int16_t *AccY, int16_t *AccZ)
{
	int16_t ax, ay, az;

	MPU6050_GetData(&ax, &ay, &az, 0, 0, 0);

	if(AccX) *AccX = (int16_t)((int32_t)ax * 1000 / MPU6050_ACCEL_LSB_PER_G);
	if(AccY) *AccY = (int16_t)((int32_t)ay * 1000 / MPU6050_ACCEL_LSB_PER_G);
	if(AccZ) *AccZ = (int16_t)((int32_t)az * 1000 / MPU6050_ACCEL_LSB_PER_G);
}

/* 陀螺换算成 0.1°/s：dps10 = raw * 100 / 164 */
void MPU6050_GetGyroDPS10(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	int16_t gx, gy, gz;

	MPU6050_GetData(0, 0, 0, &gx, &gy, &gz);

	if(GyroX) *GyroX = (int16_t)((int32_t)gx * 100 / MPU6050_GYRO_LSB_PER_DPS10);
	if(GyroY) *GyroY = (int16_t)((int32_t)gy * 100 / MPU6050_GYRO_LSB_PER_DPS10);
	if(GyroZ) *GyroZ = (int16_t)((int32_t)gz * 100 / MPU6050_GYRO_LSB_PER_DPS10);
}

/* 整数开方，避免引入 math.h */
static uint32_t MPU6050_ISqrt(uint32_t n)
{
	uint32_t r = 0, bit = 1UL << 30;

	while(bit > n)
		bit >>= 2;

	while(bit)
	{
		if(n >= r + bit)
		{
			n -= r + bit;
			r = (r >> 1) + bit;
		}
		else
		{
			r >>= 1;
		}
		bit >>= 2;
	}

	return r;
}

/* 合加速度（mg）：静止约 1000，走路 1000~2000，跌落会突然变小 */
int16_t MPU6050_GetAccelMagnitudeMG(void)
{
	int16_t x, y, z;
	uint32_t sum;

	MPU6050_GetAccelMG(&x, &y, &z);

	sum = (uint32_t)((int32_t)x * x / 4) +
	      (uint32_t)((int32_t)y * y / 4) +
	      (uint32_t)((int32_t)z * z / 4);

	return (int16_t)(MPU6050_ISqrt(sum) * 2);
}
