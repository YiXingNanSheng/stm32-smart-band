#ifndef __usart2_H
#define __usart2_H

#include "system.h"

/* 蓝牙串口接收环形缓冲区大小（字节） */
#define BT_RX_BUF_SIZE		128
/* 单条命令行最大长度（含结束符） */
#define BT_LINE_MAX			32

void USART2_Init(u32 bound);
void USART2_SendByte(u8 dat);
void USART2_SendString(const char *str);
void USART2_SendData(const u8 *buf, u16 len);
u8   USART2_ReadByte(u8 *dat);
u16  USART2_Available(void);
u8   USART2_GetLine(char *buf, u8 max);
void USART2_ClearBuf(void);
u16  USART2_GetRxCount(void);
u8   USART2_GetErrCount(void);

#endif
