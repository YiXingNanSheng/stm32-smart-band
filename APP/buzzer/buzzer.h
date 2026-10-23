#ifndef _buzzer_H
#define _buzzer_H

#include "system.h"

/*  蜂鸣器时钟端口、引脚定义 */
#define BUZZER_PORT 		GPIOA
#define BUZZER_PIN 			GPIO_Pin_4


#define BUZZER_ON	1
#define BUZZER_OFF 0


void BUZZER_Init(void);
void BUZZER_Ctrl(unsigned int buzzer_status);


#endif
