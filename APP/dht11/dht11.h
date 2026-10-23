#ifndef _dht11_H
#define _dht11_H

#include "system.h"

/*  DHT11时钟端口、引脚定义（单总线，外部1K上拉到5V） */
#define DHT11_PORT 			GPIOB
#define DHT11_PIN 			GPIO_Pin_5


/*  返回值定义 */
#define DHT11_OK			0		//读取成功
#define DHT11_ERR_TIMEOUT	1		//传感器无应答（超时）
#define DHT11_ERR_CHECKSUM	2		//校验和错误
#define DHT11_ERR_RANGE		3		//数据超出量程


/*  量程（DHT11额定：湿度20~90%RH，温度0~50℃；此处放宽用于判错） */
#define DHT11_HUMI_MAX		100
#define DHT11_TEMP_MAX		60


void DHT11_Init(void);
u8 DHT11_Read_Byte(void);
u8 DHT11_Read_Data(s8 *temp, u8 *humi);


#endif
