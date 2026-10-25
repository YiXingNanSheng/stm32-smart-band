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


/* ============================ LED 电流自动增益 ============================ */
static u8 s_pa_ir = MAX30102_LED_PA_IR;		/* 当前红外 LED 电流档位 */

/* 按实测红外直流把 LED 电流拉回量程中段：
 * 压太紧时 DC 冲高（表面反射 + 非线性），压太松时 DC 偏低（信号弱），
 * 两种情况都会让 AC 提取失败。按比例一次调整到位，线性关系下收敛很快。 */
u8 MAX30102_AgcStep(u32 dc_ir)
{
	u8  pa = s_pa_ir;
	u32 np;

	if(dc_ir < 50000)						/* 手指还没放上去，等稳定了再调 */
		return 0;

	if(dc_ir > MAX30102_AGC_LO && dc_ir < MAX30102_AGC_HI)
		return 0;							/* 已在目标区间，保持不动 */

	np = (u32)pa * MAX30102_AGC_TARGET / dc_ir;

	if(np < MAX30102_AGC_MIN) np = MAX30102_AGC_MIN;
	if(np > MAX30102_AGC_MAX) np = MAX30102_AGC_MAX;
	if((u8)np == pa)
		return 0;

	s_pa_ir = (u8)np;

	/* 红光与红外用同一档电流。
	 * 曾试过红光取 85%，想减少红光受环境光干扰，但 R 的计算是
	 *   R = (ACred/DCred) / (ACir/DCir)
	 * 一旦两路 DC 失衡，DC 比值会直接放大 R（实测 DCr/DCi 掉到 0.82，
	 * 相当于把 R 整体乘了 1.21）。保持两路电流一致，让 DC 尽量相等，
	 * R 才只取决于 AC 之比。 */
	MAX30102_WriteReg(REG_LED2_PA, s_pa_ir);
	MAX30102_WriteReg(REG_LED1_PA, s_pa_ir);

	return 1;
}

u8 MAX30102_GetLedPa(void)
{
	return s_pa_ir;
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

	/* FIFO 配置 = 0x1F：bit7:5=000 不做硬件平均，bit4=1 FIFO回绕覆盖，bit3:0=15 剩15组时拉INT
	 * 注意 1：血氧算法（spo2.c）按官方参数需要 100Hz 原始样本，所以关掉硬件平均；
	 *         心率仍保持 25Hz 有效速率，由 HR_Feed100Hz() 在软件里做 4 点平均后抽取。
	 * 注意 2：有些资料写 0x0F，但 0x0F 的 bit4=0 会关闭回绕 —— 100Hz 下 32 组只要 0.32 秒
	 *         就写满，一旦主循环有停顿就直接停采，表现为心率/血氧永久无数据，切勿照抄。 */
	MAX30102_WriteReg(REG_FIFO_CONFIG, 0x1F);
	/* SpO2 模式：红光 + 红外两路 */
	MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);
	/* ADC 量程 4096nA、采样率 100Hz、脉宽 411us（18 bit） */
	MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);
	/* LED 电流先给一个偏小的初始值，随后由 MAX30102_AgcStep() 按实测 DC 自动调整。
	 * 起始定得小一点，避免手指一贴上去就冲到接近饱和。 */
	s_pa_ir = MAX30102_LED_PA_IR;
	MAX30102_WriteReg(REG_LED1_PA, MAX30102_LED_PA_RED);
	MAX30102_WriteReg(REG_LED2_PA, MAX30102_LED_PA_IR);

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
	if(cnt > 16)								/* 100Hz 下一个主循环可能积压多组，一次多搬一些 */
		cnt = 16;

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
