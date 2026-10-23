#ifndef _led_H
#define _led_H

#include "system.h"

/*  LED时钟端口、引脚定义 */
#define LED_PORT 			GPIOB   
#define LED_PIN 			(GPIO_Pin_3|GPIO_Pin_4)


#define LED_ON	0
#define LED_OFF 1

#define LED3 			3   
#define LED4 			4


void LED_Init(void);
void LED_Ctrl(unsigned int led_num, unsigned int led_status);


#endif
