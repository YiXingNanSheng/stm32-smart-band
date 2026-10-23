#include "ds18b20.h"
#include "SysTick.h"

/* 单总线只有一根线：开漏输出，写 1 = 释放总线（由 10K 上拉拉高）
 * 因此读的时候不用切换方向，直接读 IDR 即可 */
#define DQ_H()		GPIO_SetBits(DS18B20_PORT, DS18B20_PIN)
#define DQ_L()		GPIO_ResetBits(DS18B20_PORT, DS18B20_PIN)
#define DQ_READ()	GPIO_ReadInputDataBit(DS18B20_PORT, DS18B20_PIN)


/* ============================ 单总线底层时序 ============================ */

/* 复位脉冲 + 检测存在脉冲：返回 0 = 有器件应答 */
u8 DS18B20_Reset(void)
{
	u8 presence;

	DQ_L();
	delay_us(750);					/* 手册：拉低 480~960us */
	DQ_H();
	delay_us(70);					/* 释放后等 15~60us 再采样 */
	presence = DQ_READ();			/* 0 = 器件把总线拉低（存在脉冲） */
	delay_us(400);					/* 等存在脉冲结束（60~240us） */

	return presence;
}

/* 写 1 位：写 1 在 15us 内释放；写 0 拉低满 60us */
static void DS18B20_WriteBit(u8 bit)
{
	DQ_L();
	if(bit)
	{
		delay_us(2);
		DQ_H();
		delay_us(60);
	}
	else
	{
		delay_us(60);
		DQ_H();
		delay_us(2);
	}
}

/* 读 1 位：自己拉低 ≥1us 作为起始，释放后在 15us 内采样 */
static u8 DS18B20_ReadBit(void)
{
	u8 bit;

	DQ_L();
	delay_us(2);
	DQ_H();
	delay_us(10);
	bit = DQ_READ();
	delay_us(50);					/* 补足 60us 时隙 */

	return bit;
}

/* 写一个字节，低位在前 */
static void DS18B20_WriteByte(u8 dat)
{
	u8 i;

	for(i = 0; i < 8; i++)
	{
		DS18B20_WriteBit(dat & 0x01);
		dat >>= 1;
	}
}

/* 读一个字节，低位在前 */
static u8 DS18B20_ReadByte(void)
{
	u8 i, dat = 0;

	for(i = 0; i < 8; i++)
	{
		dat >>= 1;
		if(DS18B20_ReadBit())
			dat |= 0x80;
	}

	return dat;
}

/* Dallas/Maxim CRC8，多项式 x^8+x^5+x^4+1（0x31 反射为 0x8C） */
static u8 DS18B20_CRC8(u8 *data, u8 len)
{
	u8 i, j, crc = 0;

	for(i = 0; i < len; i++)
	{
		u8 b = data[i];
		for(j = 0; j < 8; j++)
		{
			u8 mix = (u8)((crc ^ b) & 0x01);
			crc >>= 1;
			if(mix)
				crc ^= 0x8C;
			b >>= 1;
		}
	}

	return crc;
}


/* ============================ 对外接口 ============================ */

/* 初始化：PA8 配成开漏输出并释放总线；返回 0 表示检测到器件 */
u8 DS18B20_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = DS18B20_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;	/* 关键：开漏 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);

	DQ_H();							/* 释放总线，靠 10K 上拉维持高电平 */
	delay_ms(1);

	return DS18B20_Reset();			/* 顺带做一次在线检测 */
}

/* 启动一次温度转换；12 位分辨率需要最多 750ms，之后才能读 */
u8 DS18B20_StartConvert(void)
{
	if(DS18B20_Reset())
		return DS18B20_ERR_NODEV;

	DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
	DS18B20_WriteByte(DS18B20_CMD_CONVERT_T);

	return DS18B20_OK;
}

/* 读上一次转换结果，单位 0.1℃（例如 253 表示 25.3℃） */
u8 DS18B20_ReadTemp(s16 *temp_x10)
{
	u8 buf[9], i;
	s16 raw;

	if(temp_x10 == 0)
		return DS18B20_ERR_RANGE;

	if(DS18B20_Reset())
		return DS18B20_ERR_NODEV;

	DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
	DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCH);

	for(i = 0; i < 9; i++)
		buf[i] = DS18B20_ReadByte();

	/* 第 9 字节是前 8 字节的 CRC（手册 2.7 暂存存储器） */
	if(DS18B20_CRC8(buf, 8) != buf[8])
		return DS18B20_ERR_CRC;

	raw = (s16)(((u16)buf[1] << 8) | buf[0]);

	/* 上电默认值是 85℃，读到这里说明还没转换过 */
	if(raw == 0x0550)
		return DS18B20_ERR_NOTCONV;

	/* 量程 -55~125℃，对应原始值 -880~2000 */
	if(raw < -880 || raw > 2000)
		return DS18B20_ERR_RANGE;

	/* 分辨率 0.0625℃，乘 10 换算成 0.1℃：raw * 10 / 16 */
	*temp_x10 = (s16)((raw * 10) / 16);

	return DS18B20_OK;
}

/* 转换 + 等待 + 读取（阻塞约 800ms，调试时最省事） */
u8 DS18B20_ReadTempNow(s16 *temp_x10)
{
	u8 ret;

	ret = DS18B20_StartConvert();
	if(ret != DS18B20_OK)
		return ret;

	delay_ms(750);					/* 手册：12 位最大转换时间 750ms */

	return DS18B20_ReadTemp(temp_x10);
}

/* 设置分辨率：9~12 位（写入配置寄存器 bit6:5） */
u8 DS18B20_SetResolution(u8 bits)
{
	u8 cfg;

	if(bits < 9 || bits > 12)
		return DS18B20_ERR_RANGE;

	cfg = (u8)(0x1F | ((bits - 9) << 5));	/* 9位=0x1F 10位=0x3F 11位=0x5F 12位=0x7F */

	if(DS18B20_Reset())
		return DS18B20_ERR_NODEV;

	DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
	DS18B20_WriteByte(DS18B20_CMD_WRITE_SCRATCH);
	DS18B20_WriteByte(0x4B);		/* TH，默认值 */
	DS18B20_WriteByte(0x46);		/* TL，默认值 */
	DS18B20_WriteByte(cfg);

	return DS18B20_OK;
}
