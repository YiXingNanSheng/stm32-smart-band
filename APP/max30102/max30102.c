#include "max30102.h"
#include "SysTick.h"

/* ============================ 硬件 I2C2 底层 ============================
 * PB10=SCL、PB11=SDA，复用开漏，400kHz
 * 所有等待都带超时，避免 F103 硬件 I2C 偶发卡在 BUSY
 * ==================================================================== */
#define I2C_TIMEOUT		2000

static u8 MAX30102_WaitEvent(u32 event)
{
	u16 t = I2C_TIMEOUT;

	while(!I2C_CheckEvent(MAX30102_I2C, event))
	{
		if(--t == 0)
			return 1;
	}

	return 0;
}

/* 超时兜底：发停止条件并软复位外设，避免总线被占死
 * 注意：SoftReset 会清掉 CR1/CR2，复位后必须重新 I2C_Init，
 *       否则时钟配置丢失，之后所有通信都会失败（只 I2C_Cmd 不够） */
static void MAX30102_I2C_Recover(void)
{
	I2C_InitTypeDef I2C_InitStructure;

	I2C_GenerateSTOP(MAX30102_I2C, ENABLE);
	I2C_SoftwareResetCmd(MAX30102_I2C, ENABLE);
	I2C_SoftwareResetCmd(MAX30102_I2C, DISABLE);

	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0x30;
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = MAX30102_I2C_SPEED;
	I2C_Init(MAX30102_I2C, &I2C_InitStructure);

	I2C_Cmd(MAX30102_I2C, ENABLE);
	I2C_AcknowledgeConfig(MAX30102_I2C, ENABLE);
}


/* ============================ 寄存器读写 ============================ */

/* 写一个寄存器：START -> 地址W -> 寄存器号 -> 数据 -> STOP */
u8 MAX30102_WriteReg(u8 reg, u8 data)
{
	I2C_GenerateSTART(MAX30102_I2C, ENABLE);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))              { MAX30102_I2C_Recover(); return 1; }

	I2C_Send7bitAddress(MAX30102_I2C, MAX30102_ADDRESS, I2C_Direction_Transmitter);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { MAX30102_I2C_Recover(); return 1; }

	I2C_SendData(MAX30102_I2C, reg);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING))         { MAX30102_I2C_Recover(); return 1; }

	I2C_SendData(MAX30102_I2C, data);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))          { MAX30102_I2C_Recover(); return 1; }

	I2C_GenerateSTOP(MAX30102_I2C, ENABLE);
	return 0;
}

/* 连续读多个寄存器：START -> 地址W -> 寄存器号 -> 重复START -> 地址R -> 连读 -> STOP */
u8 MAX30102_ReadRegs(u8 reg, u8 *buf, u8 len)
{
	u8 i;

	if(len == 0 || buf == 0)
		return 1;

	I2C_GenerateSTART(MAX30102_I2C, ENABLE);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))              { MAX30102_I2C_Recover(); return 1; }

	I2C_Send7bitAddress(MAX30102_I2C, MAX30102_ADDRESS, I2C_Direction_Transmitter);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { MAX30102_I2C_Recover(); return 1; }

	I2C_SendData(MAX30102_I2C, reg);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING))         { MAX30102_I2C_Recover(); return 1; }

	/* 重复起始，切换为接收 */
	I2C_GenerateSTART(MAX30102_I2C, ENABLE);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))              { MAX30102_I2C_Recover(); return 1; }

	I2C_Send7bitAddress(MAX30102_I2C, MAX30102_ADDRESS, I2C_Direction_Receiver);
	if(MAX30102_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))    { MAX30102_I2C_Recover(); return 1; }

	for(i = 0; i < len; i++)
	{
		if(i == (u8)(len - 1))
		{
			/* 最后一个字节：先关 ACK 并置 STOP，再读数据 */
			I2C_AcknowledgeConfig(MAX30102_I2C, DISABLE);
			I2C_GenerateSTOP(MAX30102_I2C, ENABLE);
		}

		if(MAX30102_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			I2C_AcknowledgeConfig(MAX30102_I2C, ENABLE);
			MAX30102_I2C_Recover();
			return 1;
		}

		buf[i] = I2C_ReceiveData(MAX30102_I2C);
	}

	I2C_AcknowledgeConfig(MAX30102_I2C, ENABLE);
	return 0;
}

/* 读一个寄存器，失败返回 1，data 保持原值 */
u8 MAX30102_ReadReg(u8 reg, u8 *data)
{
	if(data == 0)
		return 1;

	return MAX30102_ReadRegs(reg, data, 1);
}


/* ============================ 初始化 ============================ */
void MAX30102_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);	/* I2C2 在 APB1 */

	/* PB10/PB11：I2C2 默认引脚，复用开漏 */
	GPIO_InitStructure.GPIO_Pin   = MAX30102_SCL_PIN | MAX30102_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 先 DeInit，清掉上一次可能残留的 BUSY */
	I2C_DeInit(MAX30102_I2C);

	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0x30;		/* 主机地址，任意值 */
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = MAX30102_I2C_SPEED;
	I2C_Init(MAX30102_I2C, &I2C_InitStructure);

	I2C_Cmd(MAX30102_I2C, ENABLE);
	I2C_AcknowledgeConfig(MAX30102_I2C, ENABLE);

	delay_ms(50);											/* 模块上电稳定 */

	/* 软复位：MODE_CONFIG 的 bit6 */
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x40);
	delay_ms(20);

	/* FIFO：4 次平均、满则覆盖旧数据（ROLLOVER=1，绝不能关，否则满后停采）、剩 15 空位 */
	MAX30102_WriteReg(REG_FIFO_CONFIG, 0x5F);
	/* SpO2 模式：红光 + 红外两路 */
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);
	/* ADC 量程 4096nA、采样率 100Hz、脉宽 411us（18 bit） */
	MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);
	/* LED 电流 0x24 ≈ 7.2mA，信号不够再往 0x3F 调 */
	MAX30102_WriteReg(REG_LED1_PA, 0x24);
	MAX30102_WriteReg(REG_LED2_PA, 0x24);

	MAX30102_ClearFIFO();
}

u8 MAX30102_GetPartID(void)
{
	u8 id = 0xFF;

	MAX30102_ReadReg(REG_PART_ID, &id);
	return id;
}

u8 MAX30102_IsOnline(void)
{
	return (MAX30102_GetPartID() == MAX30102_PART_ID_VALUE) ? 1 : 0;
}


/* ============================ FIFO ============================ */

void MAX30102_ClearFIFO(void)
{
	MAX30102_WriteReg(REG_FIFO_WR_PTR, 0x00);
	MAX30102_WriteReg(REG_OVF_COUNTER, 0x00);
	MAX30102_WriteReg(REG_FIFO_RD_PTR, 0x00);
}

u8 MAX30102_GetFIFOWritePtr(void)
{
	u8 v = 0xFF;
	MAX30102_ReadReg(REG_FIFO_WR_PTR, &v);
	return v;
}

u8 MAX30102_GetFIFOReadPtr(void)
{
	u8 v = 0xFF;
	MAX30102_ReadReg(REG_FIFO_RD_PTR, &v);
	return v;
}

/* 读一组 FIFO 样本：红光 3 字节 + 红外 3 字节，18 位有效 */
u8 MAX30102_ReadFIFO(u32 *red, u32 *ir)
{
	u8 buf[6];

	if(MAX30102_ReadRegs(REG_FIFO_DATA, buf, 6))
		return 1;

	if(red) *red = ((u32)(buf[0] & 0x03) << 16) | ((u32)buf[1] << 8) | (u32)buf[2];
	if(ir)  *ir  = ((u32)(buf[3] & 0x03) << 16) | ((u32)buf[4] << 8) | (u32)buf[5];

	return 0;
}


/* ============================ 采样缓冲 ============================ */
static MAX30102_Sample_t s_buf[MAX30102_BUF_SIZE];
static u16 s_head = 0;
static u16 s_tail = 0;
static u16 s_ovf  = 0;

void MAX30102_ResetBuffer(void)
{
	s_head = 0;
	s_tail = 0;
	s_ovf  = 0;
}

u16 MAX30102_Buffered(void)
{
	return (u16)((s_head + MAX30102_BUF_SIZE - s_tail) % MAX30102_BUF_SIZE);
}

u16 MAX30102_GetOverflow(void)
{
	return s_ovf;
}

u8 MAX30102_Pop(MAX30102_Sample_t *s)
{
	if(s == 0)
		return 1;

	if(s_head == s_tail)
		return 1;

	*s = s_buf[s_tail];
	s_tail = (u16)((s_tail + 1) % MAX30102_BUF_SIZE);
	return 0;
}

/* 主循环每轮调一次：按 FIFO 写/读指针的差值，把积压的样本全部搬走
 * 单次最多搬 8 组，避免一次占总线太久拖慢主循环 */
u8 MAX30102_Poll(void)
{
	u8 wp, rp, cnt, i, n = 0;
	u16 next;
	MAX30102_Sample_t s;

	wp = MAX30102_GetFIFOWritePtr();
	rp = MAX30102_GetFIFOReadPtr();

	if(wp > 31 || rp > 31)						/* 读失败 */
		return 0;

	cnt = (u8)((wp + 32 - rp) % 32);
	if(cnt == 0)
		return 0;
	if(cnt > 8)
		cnt = 8;

	for(i = 0; i < cnt; i++)
	{
		if(MAX30102_ReadFIFO(&s.red, &s.ir))
			break;

		next = (u16)((s_head + 1) % MAX30102_BUF_SIZE);
		if(next == s_tail)
		{
			/* 缓冲满：丢掉最老的一组，保证永远保留最新数据 */
			s_tail = (u16)((s_tail + 1) % MAX30102_BUF_SIZE);
			s_ovf++;
		}

		s_buf[s_head] = s;
		s_head = next;
		n++;
	}

	return n;
}


/* ============================ 片内温度（可选） ============================ */
u8 MAX30102_ReadTemperature(s16 *temp10)
{
	u8 int_part = 0, frac = 0;

	if(temp10 == 0)
		return 1;

	if(MAX30102_WriteReg(REG_TEMP_CONFIG, 0x01))	return 1;	/* 启动一次转换 */
	delay_ms(40);

	if(MAX30102_ReadReg(REG_TEMP_INTR, &int_part))	return 1;
	if(MAX30102_ReadReg(REG_TEMP_FRAC, &frac))		return 1;

	/* 整数部分为有符号补码，小数部分 0.0625℃/LSB */
	*temp10 = (s16)((s8)int_part * 10 + (s16)(((frac & 0x0F) * 10) / 16));

	return 0;
}
