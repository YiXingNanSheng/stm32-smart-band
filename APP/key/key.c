#include "key.h"
#include "SysTick.h"


void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;						 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin=KEY_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;			 //上拉输入，按键另一端接GND
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	 
	GPIO_Init(KEY_PORT,&GPIO_InitStructure); 	 	 	 
}

u8 KEY_Scan(void)
{
	if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_12) == KEY_DOWN)		//KEY1
	{
		delay_ms(10);												 //软件消抖
		if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_12) == KEY_DOWN)
		{
			while(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_12) == KEY_DOWN);	//等待松手，防止一次按下多次触发
			return KEY1;
		}
	}

	if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_13) == KEY_DOWN)		//KEY2
	{
		delay_ms(10);
		if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_13) == KEY_DOWN)
		{
			while(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_13) == KEY_DOWN);
			return KEY2;
		}
	}

	if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_14) == KEY_DOWN)		//KEY3
	{
		delay_ms(10);
		if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_14) == KEY_DOWN)
		{
			while(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_14) == KEY_DOWN);
			return KEY3;
		}
	}

	if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_15) == KEY_DOWN)		//KEY4
	{
		delay_ms(10);
		if(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_15) == KEY_DOWN)
		{
			while(GPIO_ReadInputDataBit(KEY_PORT,GPIO_Pin_15) == KEY_DOWN);
			return KEY4;
		}
	}

	return KEY_NONE;
}
