#ifndef _relay_H
#define _relay_H

#include "system.h"

/*  继电器时钟端口、引脚定义 */
#define RELAY_PORT 			GPIOA
#define RELAY_PIN 			GPIO_Pin_5


#define RELAY_ON	1
#define RELAY_OFF 0


void RELAY_Init(void);
void RELAY_Ctrl(unsigned int relay_status);


#endif
