#include "usart2.h"

/* 蓝牙串口接收环形缓冲区：中断里写，主循环里读 */
static u8  s_rx_buf[BT_RX_BUF_SIZE];
static volatile u16 s_rx_head = 0;
static volatile u16 s_rx_tail = 0;
static volatile u16 s_rx_cnt  = 0;			/* 累计收到的字节数，便于自测 */
static volatile u8  s_rx_err  = 0;			/* 溢出/帧错误次数 */


/*******************************************************************************
* 函 数 名 : USART2_Init
* 函数功能 : 蓝牙模块串口初始化
* 输    入 : bound-波特率（HC-05 / HC-06 出厂默认 9600）
* 说    明 : PA2 = USART2_TX，PA3 = USART2_RX
*            8 位数据位、1 位停止位、无校验、无硬件流控
*            注意 USART2 的时钟在 APB1，不是 APB2
*            板上 J1 需短接 3-5、4-6，蓝牙才与单片机相连
*******************************************************************************/
void USART2_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* PA2：TX，复用推挽输出 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA3：RX，浮空输入 */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate            = bound;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	/* 只开接收中断：每收到一个字节进一次 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}


/*******************************************************************************
* 函 数 名 : USART2_SendByte
* 函数功能 : 发送一个字节，等待 TXE 置位后再返回
*******************************************************************************/
void USART2_SendByte(u8 dat)
{
	USART_SendData(USART2, dat);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}


/*******************************************************************************
* 函 数 名 : USART2_SendString
* 函数功能 : 发送以 0 结尾的字符串
*******************************************************************************/
void USART2_SendString(const char *str)
{
	while(*str != 0)
	{
		USART2_SendByte((u8)*str);
		str++;
	}
}


/*******************************************************************************
* 函 数 名 : USART2_SendData
* 函数功能 : 发送指定长度的数据块（二进制协议帧用）
*******************************************************************************/
void USART2_SendData(const u8 *buf, u16 len)
{
	while(len--)
	{
		USART2_SendByte(*buf);
		buf++;
	}
}


/*******************************************************************************
* 函 数 名 : USART2_ReadByte
* 函数功能 : 从环形缓冲区取一个字节，不等待
* 返 回 值 : 0=取到数据，1=缓冲区为空
*******************************************************************************/
u8 USART2_ReadByte(u8 *dat)
{
	if(s_rx_head == s_rx_tail)
		return 1;

	*dat = s_rx_buf[s_rx_tail];
	s_rx_tail = (u16)((s_rx_tail + 1) % BT_RX_BUF_SIZE);
	return 0;
}


/*******************************************************************************
* 函 数 名 : USART2_Available
* 函数功能 : 返回缓冲区里还有多少字节没读
*******************************************************************************/
u16 USART2_Available(void)
{
	return (u16)((s_rx_head + BT_RX_BUF_SIZE - s_rx_tail) % BT_RX_BUF_SIZE);
}


/*******************************************************************************
* 函 数 名 : USART2_GetLine
* 函数功能 : 按行取一条命令，遇到 \r 或 \n 认为一行结束
* 输    入 : buf-输出缓冲区，max-缓冲区总大小
* 返 回 值 : 0=还没收完一整行，1=收到一行（buf 内已补 0）
* 说    明 : 支持退格 0x08/0x7F，超长字符自动丢弃，不会越界
*******************************************************************************/
u8 USART2_GetLine(char *buf, u8 max)
{
	static u8  idx = 0;
	static u16 idle = 0;
	u8 ch;
	u8 guard = 32;					/* 单次最多处理 32 字节，防止连续数据流把主循环卡死 */

	while(guard-- && USART2_ReadByte(&ch) == 0)
	{
		idle = 0;						/* 收到新字节，空闲计时清零 */

		if(ch == '\r' || ch == '\n')
		{
			if(idx > 0)
			{
				buf[idx] = 0;
				idx = 0;
				return 1;
			}
		}
		else if(ch == 0x08 || ch == 0x7F)
		{
			if(idx > 0) idx--;
		}
		else if(idx < (u8)(max - 1))
		{
			buf[idx] = (char)ch;
			idx++;
		}
	}

	/* 兼容"发送不带换行"的手机 App：50ms 内没有新字节就认为一帧结束
	   本函数由主循环每 10ms 调用一次，累加到 5 即 50ms */
	if(idx > 0)
	{
		idle++;
		if(idle >= 5)
		{
			buf[idx] = 0;
			idx = 0;
			idle = 0;
			return 1;
		}
	}

	return 0;
}


/*******************************************************************************
* 函 数 名 : USART2_ClearBuf
* 函数功能 : 清空接收缓冲区
*******************************************************************************/
void USART2_ClearBuf(void)
{
	s_rx_tail = s_rx_head;
}


/*******************************************************************************
* 函 数 名 : USART2_GetRxCount
* 函数功能 : 返回上电以来收到的字节总数，用于判断"到底通没通"
*******************************************************************************/
u16 USART2_GetRxCount(void)
{
	return s_rx_cnt;
}


/*******************************************************************************
* 函 数 名 : USART2_GetErrCount
* 函数功能 : 返回溢出/帧错误次数
*******************************************************************************/
u8 USART2_GetErrCount(void)
{
	return s_rx_err;
}


/*******************************************************************************
* 函 数 名 : USART2_IRQHandler
* 函数功能 : 接收中断，把收到的字节塞进环形缓冲区
* 说    明 : 缓冲区满时丢弃新数据，绝不覆盖未读数据
*******************************************************************************/
void USART2_IRQHandler(void)
{
	u8  r;
	u16 next;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		r = (u8)USART_ReceiveData(USART2);		/* 读 DR 同时清 RXNE 和 ORE */

		next = (u16)((s_rx_head + 1) % BT_RX_BUF_SIZE);
		if(next != s_rx_tail)
		{
			s_rx_buf[s_rx_head] = r;
			s_rx_head = next;
		}
		s_rx_cnt++;
	}
	else if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		/* 溢出错误：读一次 DR 清标志，否则接收会彻底卡死 */
		(void)USART_ReceiveData(USART2);
		s_rx_err++;
	}
}
