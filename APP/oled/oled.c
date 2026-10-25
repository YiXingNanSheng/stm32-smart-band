#include "oled.h"
#include "oledfont.h"
#include "SysTick.h"

/* 显存：8 页 x 128 列，每字节 8 个像素（bit0 在上）
 * 所有绘图都先写显存，再统一 OLED_Refresh() 送屏，这样画线/画圆不会闪烁 */
static u8 s_gram[OLED_PAGES][OLED_WIDTH];

/* 文本光标（像素坐标），OLED_Print 从这里继续输出 */
static u8 s_cx = 0;
static u8 s_cy = 0;


/* ============================ 软件 I2C 底层 ============================ */
#define SCL_H()		GPIO_SetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define SCL_L()		GPIO_ResetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define SDA_H()		GPIO_SetBits(OLED_SDA_PORT, OLED_SDA_PIN)
#define SDA_L()		GPIO_ResetBits(OLED_SDA_PORT, OLED_SDA_PIN)
#define SDA_READ()	GPIO_ReadInputDataBit(OLED_SDA_PORT, OLED_SDA_PIN)

#define I2C_DELAY()	delay_us(2)			/* 半周期 2us，约 250kHz */


static void I2C_Start(void)
{
	SDA_H();
	SCL_H();
	I2C_DELAY();
	SDA_L();							/* SCL 高时 SDA 下降沿 = 起始 */
	I2C_DELAY();
	SCL_L();
}

static void I2C_Stop(void)
{
	SCL_L();
	SDA_L();
	I2C_DELAY();
	SCL_H();
	I2C_DELAY();
	SDA_H();							/* SCL 高时 SDA 上升沿 = 停止 */
	I2C_DELAY();
}

/* 写一个字节，忽略从机应答（OLED 不会 NACK，简单处理） */
static void I2C_WriteByte(u8 dat)
{
	u8 i;

	for(i = 0; i < 8; i++)
	{
		if(dat & 0x80)
			SDA_H();
		else
			SDA_L();
		dat <<= 1;
		I2C_DELAY();
		SCL_H();
		I2C_DELAY();
		SCL_L();
	}

	/* 第 9 个时钟：释放 SDA，读应答位 */
	SDA_H();
	I2C_DELAY();
	SCL_H();
	I2C_DELAY();
	SCL_L();
}


/* ============================ SSD1306 命令/数据 ============================ */

/* 写命令：地址 + 0x00 控制字节 + 命令（对应手册“IIC 电路连接”一节的时序） */
static void OLED_WriteCmd(u8 cmd)
{
	I2C_Start();
	I2C_WriteByte(OLED_ADDR);
	I2C_WriteByte(0x00);				/* Co=0 D/C#=0：后面是命令 */
	I2C_WriteByte(cmd);
	I2C_Stop();
}

/* 连续写一页 128 字节显存：地址 + 0x40 控制字节 + 数据流 */
static void OLED_WritePage(u8 page, u8 *buf)
{
	u8 i;

	OLED_WriteCmd(0xB0 + page);			/* 页地址 0~7 */
	OLED_WriteCmd(0x00);				/* 列地址低 4 位 */
	OLED_WriteCmd(0x10);				/* 列地址高 4 位 */

	I2C_Start();
	I2C_WriteByte(OLED_ADDR);
	I2C_WriteByte(0x40);				/* Co=0 D/C#=1：后面是数据 */
	for(i = 0; i < OLED_WIDTH; i++)
		I2C_WriteByte(buf[i]);
	I2C_Stop();
}


/* ============================ 初始化 ============================ */
/* 命令序列与资料包例程 OLED_Init() 一致，括号内为 SSD1306 手册命令名 */
void OLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* 两根线都配成开漏输出：开漏才能线与，读 IDR 也能反映真实电平 */
	GPIO_InitStructure.GPIO_Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SCL_H();
	SDA_H();
	delay_ms(100);						/* 上电稳定，手册提示 OLED 上电无显示，需程序驱动 */

	OLED_WriteCmd(0xAE);				/* Display OFF */
	OLED_WriteCmd(0x20);				/* Set Memory Addressing Mode */
	OLED_WriteCmd(0x10);				/* 10 = Page Addressing Mode（页寻址） */
	OLED_WriteCmd(0xB0);				/* Page Start Address = 0 */
	OLED_WriteCmd(0xC8);				/* COM Output Scan Direction 反向 */
	OLED_WriteCmd(0x00);				/* 低列地址 */
	OLED_WriteCmd(0x10);				/* 高列地址 */
	OLED_WriteCmd(0x40);				/* Display Start Line = 0 */
	OLED_WriteCmd(0x81);				/* Set Contrast Control */
	OLED_WriteCmd(0xCF);				/* 对比度 0x00~0xFF，默认给 0xCF */
	OLED_WriteCmd(0xA1);				/* Segment Re-map（左右镜像修正） */
	OLED_WriteCmd(0xA6);				/* Normal Display（0xA7 为反显） */
	OLED_WriteCmd(0xA8);				/* Set Multiplex Ratio */
	OLED_WriteCmd(0x3F);				/* 1/64 duty */
	OLED_WriteCmd(0xA4);				/* Output follows RAM content */
	OLED_WriteCmd(0xD3);				/* Set Display Offset */
	OLED_WriteCmd(0x00);
	OLED_WriteCmd(0xD5);				/* Set Display Clock Divide Ratio */
	OLED_WriteCmd(0xF0);
	OLED_WriteCmd(0xD9);				/* Set Pre-charge Period */
	OLED_WriteCmd(0x22);
	OLED_WriteCmd(0xDA);				/* Set COM Pins Hardware Config */
	OLED_WriteCmd(0x12);
	OLED_WriteCmd(0xDB);				/* Set VCOMH Deselect Level */
	OLED_WriteCmd(0x20);
	OLED_WriteCmd(0x8D);				/* Charge Pump Setting */
	OLED_WriteCmd(0x14);				/* 0x14 = 开启电荷泵（模块必需） */
	OLED_WriteCmd(0xAF);				/* Display ON */

	OLED_ClearBuffer();
	OLED_Refresh();
}


/* ============================ 显存与刷屏 ============================ */
void OLED_ClearBuffer(void)
{
	u8 page, col;

	for(page = 0; page < OLED_PAGES; page++)
		for(col = 0; col < OLED_WIDTH; col++)
			s_gram[page][col] = 0x00;
}

/* 清屏：清显存并立即送屏 */
void OLED_Clear(void)
{
	OLED_ClearBuffer();
	OLED_Refresh();
}

/* 把显存整体送到屏幕（8 页 x 128 列，约 20~40ms） */
void OLED_Refresh(void)
{
	u8 page;

	for(page = 0; page < OLED_PAGES; page++)
		OLED_WritePage(page, s_gram[page]);
}

/* 只刷指定页范围：整屏 8 页约 40ms，只刷 2 页约 10ms。
 * 波形这类高频刷新的区域用它，避免把主循环节拍（MAX30102 100Hz 采集）拖垮 */
void OLED_RefreshPages(u8 from, u8 to)
{
	u8 page;

	if(from > to)
		return;
	if(to >= OLED_PAGES)
		to = OLED_PAGES - 1;

	for(page = from; page <= to; page++)
		OLED_WritePage(page, s_gram[page]);
}


/* ============================ 光标与字符 ============================ */
void OLED_SetPos(u8 x, u8 y)
{
	if(x >= OLED_WIDTH)  x = 0;
	if(y >= OLED_HEIGHT) y = 0;
	s_cx = x;
	s_cy = y;
}

void OLED_DrawPoint(u8 x, u8 y, u8 color)
{
	if(x >= OLED_WIDTH || y >= OLED_HEIGHT)
		return;

	if(color)
		s_gram[y / 8][x] |= (u8)(1 << (y % 8));
	else
		s_gram[y / 8][x] &= (u8)~(1 << (y % 8));
}

/* 显示一个 ASCII 字符；size=8 用 6x8 字模，size=16 用 8x16 字模 */
void OLED_ShowChar(u8 x, u8 y, char ch, u8 size)
{
	u8 i, j, data, code, max_ch;

	/* 6x8 字模表只收录了 ' '~'z'（92 条，最后一条是图形符号），越界会读到乱码 */
	max_ch = (size == OLED_FONT_8) ? 'z' : '~';
	if(ch < ' ' || ch > max_ch)
		ch = '?';

	code = (u8)(ch - ' ');

	if(size == OLED_FONT_8)
	{
		for(i = 0; i < 6; i++)					/* 6 列 */
		{
			data = F6x8[code][i];
			for(j = 0; j < 8; j++)				/* 每列 8 位，bit0 在上 */
				OLED_DrawPoint(x + i, y + j, (data >> j) & 0x01);
		}
	}
	else
	{
		for(i = 0; i < 8; i++)					/* 上半页 8 列 */
		{
			data = F8X16[code * 16 + i];
			for(j = 0; j < 8; j++)
				OLED_DrawPoint(x + i, y + j, (data >> j) & 0x01);
		}
		for(i = 0; i < 8; i++)					/* 下半页 8 列 */
		{
			data = F8X16[code * 16 + 8 + i];
			for(j = 0; j < 8; j++)
				OLED_DrawPoint(x + i, y + 8 + j, (data >> j) & 0x01);
		}
	}
}

/* 显示字符串，支持 '\n' 换行；每写一个字符自动更新光标 */
void OLED_ShowString(u8 x, u8 y, char *str, u8 size)
{
	u8 w = (size == OLED_FONT_8) ? 6 : 8;

	while(*str != 0)
	{
		if(*str == '\n')
		{
			x  = 0;
			y += size;
		}
		else
		{
			if(x + w > OLED_WIDTH)			/* 行末自动换行 */
			{
				x  = 0;
				y += size;
			}
			OLED_ShowChar(x, y, *str, size);
			x += w;
		}
		str++;
	}

	s_cx = x;
	s_cy = y;
}

/* 显示整数（-2147483648 ~ 2147483647） */
void OLED_ShowNum(u8 x, u8 y, s32 num, u8 size)
{
	char buf[12];
	u8 i = 0, j;
	u32 u;

	if(num < 0)
	{
		buf[i++] = '-';
		u = (u32)(-num);
	}
	else
	{
		u = (u32)num;
	}

	if(u == 0)
	{
		buf[i++] = '0';
	}
	else
	{
		char tmp[10];
		u8 n = 0;
		while(u > 0)
		{
			tmp[n++] = (char)('0' + (u % 10));
			u /= 10;
		}
		for(j = 0; j < n; j++)
			buf[i++] = tmp[n - 1 - j];
	}
	buf[i] = 0;

	OLED_ShowString(x, y, buf, size);
}

/* 从当前光标继续输出（OLED_SetPos 定位后用它） */
void OLED_Print(char *str)
{
	OLED_ShowString(s_cx, s_cy, str, OLED_FONT_8);
}


/* ============================ 放大数字与图标 ============================ */

/* 单字符放大 2 倍（16x32）：直接复用 8x16 字模，每个像素画成 2x2 方块。
 * 好处是不用新增字库，风格和原有 8x16 完全一致；代价是边缘呈方块状，
 * 但在单色点阵屏上反而更清晰。 */
void OLED_ShowChar2x(u8 x, u8 y, char ch)
{
	u8 i, j, data, code;

	if(ch < ' ' || ch > '~')
		ch = '?';

	code = (u8)(ch - ' ');

	for(i = 0; i < 8; i++)					/* 字模上半 8 列 */
	{
		data = F8X16[code * 16 + i];
		for(j = 0; j < 8; j++)
			if(data & (1 << j))
				OLED_FillRect((u8)(x + i * 2), (u8)(y + j * 2),
				              (u8)(x + i * 2 + 1), (u8)(y + j * 2 + 1), OLED_WHITE);
	}

	for(i = 0; i < 8; i++)					/* 字模下半 8 列 */
	{
		data = F8X16[code * 16 + 8 + i];
		for(j = 0; j < 8; j++)
			if(data & (1 << j))
				OLED_FillRect((u8)(x + i * 2), (u8)(y + 16 + j * 2),
				              (u8)(x + i * 2 + 1), (u8)(y + 16 + j * 2 + 1), OLED_WHITE);
	}
}

/* 放大数字：每个字符占 16 像素宽 */
void OLED_ShowNum2x(u8 x, u8 y, s32 num)
{
	char buf[12];
	u8 i = 0, j;
	u32 u;

	if(num < 0)
	{
		buf[i++] = '-';
		u = (u32)(-num);
	}
	else
	{
		u = (u32)num;
	}

	if(u == 0)
	{
		buf[i++] = '0';
	}
	else
	{
		char tmp[10];
		u8 n = 0;

		while(u > 0)
		{
			tmp[n++] = (char)('0' + (u % 10));
			u /= 10;
		}
		for(j = 0; j < n; j++)
			buf[i++] = tmp[n - 1 - j];
	}
	buf[i] = 0;

	for(j = 0; j < i; j++)
		OLED_ShowChar2x((u8)(x + j * 16), y, buf[j]);
}

/* 8x8 图标：icon[0] 是最上面一行，bit7 在最左 */
void OLED_DrawIcon8x8(u8 x, u8 y, const u8 *icon)
{
	u8 i, j;

	if(icon == 0)
		return;

	for(j = 0; j < 8; j++)
		for(i = 0; i < 8; i++)
			OLED_DrawPoint((u8)(x + i), (u8)(y + j), (u8)((icon[j] >> (7 - i)) & 0x01));
}

/* 页码指示器：total 个 4x4 方块，间隔 6 像素，当前页实心 */
void OLED_DrawPageDots(u8 x, u8 y, u8 total, u8 cur)
{
	u8 i, dx;

	for(i = 0; i < total; i++)
	{
		dx = (u8)(x + i * 6);

		if(i == cur)
			OLED_FillRect(dx, y, (u8)(dx + 3), (u8)(y + 3), OLED_WHITE);
		else
			OLED_DrawRect(dx, y, (u8)(dx + 3), (u8)(y + 3), OLED_WHITE);
	}
}


/* ============================ 基础图形 ============================ */
void OLED_DrawLine(u8 x0, u8 y0, u8 x1, u8 y1, u8 color)
{
	s16 dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
	s16 dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
	s16 sx = (x0 < x1) ? 1 : -1;
	s16 sy = (y0 < y1) ? 1 : -1;
	s16 err = dx - dy;
	s16 e2;

	while(1)
	{
		OLED_DrawPoint((u8)x0, (u8)y0, color);
		if(x0 == x1 && y0 == y1)
			break;
		e2 = err * 2;
		if(e2 > -dy) { err -= dy; x0 += sx; }
		if(e2 <  dx) { err += dx; y0 += sy; }
	}
}

/* 空心矩形，(x0,y0) 左上、(x1,y1) 右下 */
void OLED_DrawRect(u8 x0, u8 y0, u8 x1, u8 y1, u8 color)
{
	OLED_DrawLine(x0, y0, x1, y0, color);
	OLED_DrawLine(x0, y1, x1, y1, color);
	OLED_DrawLine(x0, y0, x0, y1, color);
	OLED_DrawLine(x1, y0, x1, y1, color);
}

/* 实心矩形 */
void OLED_FillRect(u8 x0, u8 y0, u8 x1, u8 y1, u8 color)
{
	u8 x, y;

	if(x0 > x1) { x = x0; x0 = x1; x1 = x; }
	if(y0 > y1) { y = y0; y0 = y1; y1 = y; }

	for(y = y0; y <= y1; y++)
		for(x = x0; x <= x1; x++)
			OLED_DrawPoint(x, y, color);
}

/* 空心圆：中点画圆法 */
void OLED_DrawCircle(u8 x0, u8 y0, u8 r, u8 color)
{
	s16 a = 0, b = r;
	s16 d = 3 - 2 * r;

	while(a <= b)
	{
		OLED_DrawPoint(x0 + a, y0 + b, color);
		OLED_DrawPoint(x0 - a, y0 + b, color);
		OLED_DrawPoint(x0 + a, y0 - b, color);
		OLED_DrawPoint(x0 - a, y0 - b, color);
		OLED_DrawPoint(x0 + b, y0 + a, color);
		OLED_DrawPoint(x0 - b, y0 + a, color);
		OLED_DrawPoint(x0 + b, y0 - a, color);
		OLED_DrawPoint(x0 - b, y0 - a, color);

		if(d < 0)
			d += 4 * a + 6;
		else
		{
			d += 4 * (a - b) + 10;
			b--;
		}
		a++;
	}
}


/* ============================ 亮度与电源 ============================ */
/* 对比度/亮度：0x00 最暗，0xFF 最亮（SSD1306 Set Contrast Control, 0x81） */
void OLED_SetContrast(u8 contrast)
{
	OLED_WriteCmd(0x81);
	OLED_WriteCmd(contrast);
}

void OLED_DisplayOn(void)
{
	OLED_WriteCmd(0x8D);				/* 电荷泵 */
	OLED_WriteCmd(0x14);				/* 开 */
	OLED_WriteCmd(0xAF);				/* 开显示 */
}

/* 休眠：功耗可降到 10uA 以下，适合手环待机 */
void OLED_DisplayOff(void)
{
	OLED_WriteCmd(0x8D);
	OLED_WriteCmd(0x10);				/* 关电荷泵 */
	OLED_WriteCmd(0xAE);				/* 关显示 */
}

/* mode=1 反显，mode=0 正常 */
void OLED_Inverse(u8 mode)
{
	OLED_WriteCmd(mode ? 0xA7 : 0xA6);
}
