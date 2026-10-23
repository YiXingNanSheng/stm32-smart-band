#include "buzzer.h"


void BUZZER_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;						 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin=BUZZER_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	 
	GPIO_Init(BUZZER_PORT,&GPIO_InitStructure); 	 	 	 

	GPIO_ResetBits(BUZZER_PORT,BUZZER_PIN);					 //上电默认关闭蜂鸣器
}

void BUZZER_Ctrl(unsigned int buzzer_status)
{
	if(buzzer_status == BUZZER_ON)
		GPIO_SetBits(BUZZER_PORT,BUZZER_PIN);				 //PA4输出高电平，三极管导通，蜂鸣器响
	else if(buzzer_status == BUZZER_OFF)
		GPIO_ResetBits(BUZZER_PORT,BUZZER_PIN);			 //PA4输出低电平，三极管截止，蜂鸣器停
}
