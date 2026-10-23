#ifndef _key_H
#define _key_H

#include "system.h"

/*  按键时钟端口、引脚定义 */
#define KEY_PORT 			GPIOB
#define KEY_PIN 			(GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15)


#define KEY_DOWN	0
#define KEY_UP		1

#define KEY_NONE	0
#define KEY1 			1   
#define KEY2 			2
#define KEY3 			3
#define KEY4 			4


void KEY_Init(void);
u8 KEY_Scan(void);


#endif
