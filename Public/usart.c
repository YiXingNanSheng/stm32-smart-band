#include "usart.h"
#include "SysTick.h"

/*  串口接收环形缓冲区：中断只负责存，主循环负责取 */
static u8 s_rx_buf[USART_RX_BUF_SIZE];
static volatile u16 s_rx_head = 0;
static volatile u16 s_rx_tail = 0;


/*******************************************************************************
* 函 数 名 : fputc
* 函数功能 : printf重定向，使用printf时自动调用，把字符从USART1发出去
*******************************************************************************/
int fputc(int ch, FILE *p)
{
    USART_SendData(USART1, (u8)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}


/*******************************************************************************
* 函 数 名 : USART1_Init
* 函数功能 : USART1初始化，PA9=TX，PA10=RX
* 输    入 : bound-波特率（本工程用115200）
* 参数配置 : 8位数据位、1位停止位、无校验、无硬件流控、收发模式
*******************************************************************************/
void USART1_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9：TX，复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10：RX，浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 串口参数：8-N-1 */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 开启接收中断，配合环形缓冲区使用 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}


/*******************************************************************************
* 函 数 名 : USART_SendByte
* 函数功能 : 发送一个字节（阻塞，等TXE置位表示数据已进移位寄存器）
*******************************************************************************/
void USART_SendByte(USART_TypeDef *USARTx, u8 dat)
{
    USART_SendData(USARTx, dat);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}


/*******************************************************************************
* 函 数 名 : USART_SendString
* 函数功能 : 发送字符串，遇到结束符停止
*******************************************************************************/
void USART_SendString(USART_TypeDef *USARTx, char *str)
{
    while(*str != 0)
    {
        USART_SendByte(USARTx, (u8)*str);
        str++;
    }
}


/*******************************************************************************
* 函 数 名 : USART_ReadByte
* 函数功能 : 从环形缓冲区取一个字节（非阻塞）
* 输    出 : 0=取到数据，1=缓冲区为空
*******************************************************************************/
u8 USART_ReadByte(u8 *dat)
{
    if(s_rx_head == s_rx_tail)
        return 1;

    *dat = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % USART_RX_BUF_SIZE;
    return 0;
}


/*******************************************************************************
* 函 数 名 : USART_ReceiveByte
* 函数功能 : 阻塞等待接收一个字节，最多等约100ms
* 输    出 : 0=收到数据，1=超时
*******************************************************************************/
u8 USART_ReceiveByte(u8 *dat)
{
    u16 timeout = 100;

    while(USART_ReadByte(dat))
    {
        if(timeout-- == 0)
            return 1;
        delay_ms(1);
    }
    return 0;
}


/*******************************************************************************
* 函 数 名 : USART1_IRQHandler
* 函数功能 : USART1中断服务程序，收到字节后存入环形缓冲区
*******************************************************************************/
void USART1_IRQHandler(void)
{
    u8 r;
    u16 next;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        r = USART_ReceiveData(USART1);

        next = (s_rx_head + 1) % USART_RX_BUF_SIZE;
        if(next != s_rx_tail)                  /* 缓冲区未满才存，满了丢弃 */
        {
            s_rx_buf[s_rx_head] = r;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
