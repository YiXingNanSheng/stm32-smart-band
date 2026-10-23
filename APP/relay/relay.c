#include "relay.h"


void RELAY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;						 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin=RELAY_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	 
	GPIO_Init(RELAY_PORT,&GPIO_InitStructure); 	 	 	 

	GPIO_ResetBits(RELAY_PORT,RELAY_PIN);					 //上电默认断开继电器
}

void RELAY_Ctrl(unsigned int relay_status)
{
	if(relay_status == RELAY_ON)
		GPIO_SetBits(RELAY_PORT,RELAY_PIN);				 //PA5输出高电平，三极管导通，继电器吸合
	else if(relay_status == RELAY_OFF)
		GPIO_ResetBits(RELAY_PORT,RELAY_PIN);			 //PA5输出低电平，三极管截止，继电器断开
}
