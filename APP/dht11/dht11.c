#include "dht11.h"
#include "SysTick.h"

/*  读引脚电平 */
#define DHT11_READ()	GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)

static u8 s_err = DHT11_OK;							//保存读字节过程中的错误


/*******************************************************************************
* 函 数 名 : DHT11_Set_Mode
* 函数功能 : 切换PB5的方向（输出用于发起始信号，输入用于读数据）
*******************************************************************************/
static void DHT11_Set_Mode(GPIOMode_TypeDef mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = DHT11_PIN;
	GPIO_InitStructure.GPIO_Mode  = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DHT11_PORT, &GPIO_InitStructure);
}


/*******************************************************************************
* 函 数 名 : DHT11_Wait_Level
* 函数功能 : 等待引脚变成指定电平，带超时保护，返回0=等到 1=超时
*******************************************************************************/
static u8 DHT11_Wait_Level(u8 level, u16 timeout_us)
{
	while(DHT11_READ() != level)
	{
		if(timeout_us == 0)
			return 1;									//超时，防止死等卡死整机
		timeout_us--;
		delay_us(1);
	}
	return 0;
}


/*******************************************************************************
* 函 数 名 : DHT11_Init
* 函数功能 : 初始化PB5为输入，释放总线（外部1K上拉保持高电平）
*******************************************************************************/
void DHT11_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	DHT11_Set_Mode(GPIO_Mode_IN_FLOATING);			//外部已有上拉，浮空输入即可
	GPIO_SetBits(DHT11_PORT, DHT11_PIN);

	delay_ms(1000);									//DHT11上电后需≥1s稳定
}


/*******************************************************************************
* 函 数 名 : DHT11_Read_Byte
* 函数功能 : 读1个字节（8bit），每位=50us低电平+高电平(26~28us为0，70us为1)
*******************************************************************************/
u8 DHT11_Read_Byte(void)
{
	u8 i, dat = 0;

	for(i = 0; i < 8; i++)
	{
		if(DHT11_Wait_Level(1, 100))					//等待每位起始的50us低电平结束
		{
			s_err = DHT11_ERR_TIMEOUT;
			return 0;
		}

		delay_us(40);									//延时40us后采样：0码此时已拉低，1码仍为高
		dat <<= 1;
		if(DHT11_READ())
		{
			dat |= 0x01;
			if(DHT11_Wait_Level(0, 100))			//等待高电平结束
			{
				s_err = DHT11_ERR_TIMEOUT;
				return 0;
			}
		}
	}

	return dat;
}


/*******************************************************************************
* 函 数 名 : DHT11_Read_Data
* 函数功能 : 读一次温湿度
* 输    入 : temp-温度(℃)，humi-湿度(%RH)，均为指针回传
* 输    出 : 0=DHT11_OK，其余见dht11.h中的错误码
* 说    明 : 两次调用间隔需≥1s，DHT11采样周期为1s
*******************************************************************************/
u8 DHT11_Read_Data(s8 *temp, u8 *humi)
{
	u8 buf[5];
	u8 i, sum;

	if(temp == 0 || humi == 0)						//空指针保护
		return DHT11_ERR_RANGE;

	s_err = DHT11_OK;

	/* 1.主机起始信号：拉低至少18ms */
	DHT11_Set_Mode(GPIO_Mode_Out_PP);
	GPIO_ResetBits(DHT11_PORT, DHT11_PIN);
	delay_ms(20);

	/* 2.释放总线：拉高后切回输入，由外部上拉维持高电平 */
	GPIO_SetBits(DHT11_PORT, DHT11_PIN);
	DHT11_Set_Mode(GPIO_Mode_IN_FLOATING);
	delay_us(30);

	/* 3.等待DHT11应答：80us低 + 80us高 */
	if(DHT11_Wait_Level(0, 100)) return DHT11_ERR_TIMEOUT;
	if(DHT11_Wait_Level(1, 100)) return DHT11_ERR_TIMEOUT;
	if(DHT11_Wait_Level(0, 100)) return DHT11_ERR_TIMEOUT;

	/* 4.连续读40bit，即5个字节：湿度整数/小数、温度整数/小数、校验和 */
	for(i = 0; i < 5; i++)
		buf[i] = DHT11_Read_Byte();
	if(s_err != DHT11_OK)
		return s_err;

	/* 5.校验和处理 */
	sum = (u8)(buf[0] + buf[1] + buf[2] + buf[3]);
	if(sum != buf[4])
		return DHT11_ERR_CHECKSUM;

	/* 6.量程检查，过滤明显异常值 */
	if(buf[0] > DHT11_HUMI_MAX || buf[2] > DHT11_TEMP_MAX)
		return DHT11_ERR_RANGE;

	*humi = buf[0];									//湿度整数部分
	*temp = (s8)buf[2];								//温度整数部分

	return DHT11_OK;
}
