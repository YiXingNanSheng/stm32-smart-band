#ifndef __usart_H
#define __usart_H

#include "system.h"
#include "stdio.h"

/*  串口接收缓冲区大小（字节） */
#define USART_RX_BUF_SIZE	64

void USART1_Init(u32 bound);
void USART_SendByte(USART_TypeDef *USARTx, u8 dat);
void USART_SendString(USART_TypeDef *USARTx, char *str);
u8   USART_ReadByte(u8 *dat);
u8   USART_ReceiveByte(u8 *dat);


#endif
