#include "led.h"


void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;						 
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	   		 
	
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);   	
	//GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);
}

void LED_Ctrl(unsigned int led_num, unsigned int led_status)
{
	switch(led_num){
		case LED3:
				if(led_status == LED_ON)
					GPIO_ResetBits(GPIOB,GPIO_Pin_3);
				else if(led_status == LED_OFF)
					GPIO_SetBits(GPIOB,GPIO_Pin_3);
			  break;
			
		case LED4:
				if(led_status == LED_ON)
					GPIO_ResetBits(GPIOB,GPIO_Pin_4);
				else if(led_status == LED_OFF)
					GPIO_SetBits(GPIOB,GPIO_Pin_4);
				break;
		default:
			break;
	}
	
}

