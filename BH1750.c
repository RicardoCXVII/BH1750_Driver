#include "BH1750.h"
#include "delay.h"
#include "main.h"
/*
XS8     SCL->PB13    SDA->PB14    BH1750
*/
//定义i2c总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚
#define GPIO_PORT_BH1750_01    GPIOB                   //BH1750       GPIO端口
#define RCC_BH1750_01_PORT     RCC_AHB1Periph_GPIOB    //BH1750       GPIO端口时钟
#define BH1750_01_SCL_PIN      GPIO_Pin_10             //BH1750       时钟线 SCL -> PB13
#define BH1750_01_SDA_PIN      GPIO_Pin_11             //BH1750       数据线 SDA -> PB14
//定义读写SCL和SDA的宏
#define BH1750_01_SCL_1        GPIO_PORT_BH1750_01->BSRRL = BH1750_01_SCL_PIN           //SCL = 1
#define BH1750_01_SCL_0        GPIO_PORT_BH1750_01->BSRRH = BH1750_01_SCL_PIN           //SCL = 0
#define BH1750_01_SDA_1        GPIO_PORT_BH1750_01->BSRRL = BH1750_01_SDA_PIN           //SDA = 1
#define BH1750_01_SDA_0        GPIO_PORT_BH1750_01->BSRRH = BH1750_01_SDA_PIN           //SDA = 0
#define BH1750_01_SDA_READ     ((GPIO_PORT_BH1750_01->IDR & BH1750_01_SDA_PIN) != 0)    //读SDA口线状态
#define BH1750_01_SCL_READ     ((GPIO_PORT_BH1750_01->IDR & BH1750_01_SCL_PIN) != 0)    //读SCL口线状态
 
BH1750_Typedef BH1750Data;
 
void BH1750_i2c_Stop01(void);
/*
函 数 名: InitI2C
功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
形    参: 无
返 回 值: 无
*/
void BH1750_InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_BH1750_01_PORT, ENABLE);    //打开GPIO时钟
 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;         //设为输出口
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;         //设为开漏模式
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;      //上下拉电阻不使能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;      //IO口最大速度
	GPIO_InitStructure.GPIO_Pin   = BH1750_01_SCL_PIN | BH1750_01_SDA_PIN;
	GPIO_Init(GPIO_PORT_BH1750_01, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_PORT_BH1750_01, BH1750_01_SCL_PIN | BH1750_01_SDA_PIN);    //输出高
 
	//给一个停止信号, 复位I2C总线上的所有设备到待机模式
	BH1750_i2c_Stop01();
}
/*
延时函数
*/
static void BH1750_i2c_Delay(void)
{
	Delay_us(4);
}
/*
函 数 名: i2c_Start
功能说明: CPU发起I2C总线启动信号
形    参: 无
返 回 值: 无
*/
void BH1750_i2c_Start01(void)
{
	//当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号
	BH1750_01_SDA_1;
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SDA_0;
	BH1750_i2c_Delay();
	BH1750_01_SCL_0;
	BH1750_i2c_Delay();
}
/*
函 数 名: i2c_Stop
功能说明: CPU发起I2C总线停止信号
形    参: 无
返 回 值: 无
*/
void BH1750_i2c_Stop01(void)
{
	//当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号
	BH1750_01_SDA_0;
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SDA_1;
	BH1750_i2c_Delay();
}
/*
函 数 名: i2c_SendByte
功能说明: CPU向I2C总线设备发送8bit数据
形    参: _ucByte ： 等待发送的字节
返 回 值: 无
*/
void BH1750_i2c_SendByte01(uint8_t _ucByte)
{
	uint8_t i;
 
	//先发送字节的高位bit7
	for (i = 0; i < 8; i++)
	{
		if(_ucByte & 0x80)    BH1750_01_SDA_1;
		else                  BH1750_01_SDA_0;
		BH1750_i2c_Delay();
		BH1750_01_SCL_1;
		BH1750_i2c_Delay();
		BH1750_01_SCL_0;
		_ucByte <<= 1;    //左移一个bit
	}
	BH1750_i2c_Delay();
	BH1750_01_SDA_1;    //释放总线
}
/*
函 数 名: i2c_ReadByte
功能说明: CPU从I2C总线设备读取8bit数据
形    参: 无
返 回 值: 读到的数据
*/
uint8_t BH1750_i2c_ReadByte01(void)
{
	uint8_t i;
	uint8_t value = 0;
	//读到第1个bit为数据的bit7
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		BH1750_01_SCL_1;
		BH1750_i2c_Delay();
		if (BH1750_01_SDA_READ)
		{
			value++;
		}
		BH1750_01_SCL_0;
		BH1750_i2c_Delay();
	}
	return value;
}
/*
函 数 名: i2c_WaitAck
功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
形    参: 无
返 回 值: 返回0表示正确应答，1表示无器件响应
*/
uint8_t BH1750_i2c_WaitAck01(void)
{
	uint8_t re;
	uint16_t ucErrTime = 0;
 
	BH1750_01_SDA_1;
	BH1750_i2c_Delay();
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	for(ucErrTime = 0; ucErrTime < 3000; ucErrTime++)
	{
		if(BH1750_01_SDA_READ == 0)
		{
			re = 0;
			break;
		}
		re = 1;
	}
	BH1750_01_SCL_0;
	BH1750_i2c_Delay();
	return re;
}
/*
函 数 名: i2c_Ack
功能说明: CPU产生一个ACK信号
形    参: 无
返 回 值: 无
*/
void BH1750_i2c_Ack01(void)
{
	BH1750_01_SDA_0;
	BH1750_i2c_Delay();
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SCL_0;
	BH1750_i2c_Delay();
	BH1750_01_SDA_1;
}
/*
函 数 名: i2c_NAck
功能说明: CPU产生1个NACK信号
形    参: 无
返 回 值: 无
*/
void BH1750_i2c_NAck01(void)
{
	BH1750_01_SDA_1;
	BH1750_i2c_Delay();
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SCL_0;
	BH1750_i2c_Delay();
}
/*
函 数 名: I2C_Write
功能说明: 写一个字节
形    参: cmd:要写入的一个字节数据
返 回 值: 无
*/
void BH1750_I2C_Write01(uint8_t cmd)
{
	BH1750_i2c_Start01();
	BH1750_i2c_SendByte01(BH1750_ADDR_WRITE);
	BH1750_i2c_WaitAck01();
	BH1750_i2c_SendByte01(cmd);
	BH1750_i2c_WaitAck01();
	BH1750_i2c_Stop01();
}
/*
函 数 名: I2C_Read
功能说明: 读取光照值
形    参: temp[]:照度输出值  light[]：寄存器读出来的值
返 回 值: 返回值 0 表示正确， 返回1表示未探测到
光照强度 =(寄存器值[15:0] * 分辨率) / 1.2 （单位：勒克斯lx）
*/
int8_t BH1750_I2C_Read01(float temp[],uint16_t light[])
{
	int8_t a;
	uint8_t Buf[2];
	uint16_t BH1750_Buf;
 
	BH1750_i2c_Start01();
	BH1750_i2c_SendByte01(BH1750_ADDR_READ);
	a = BH1750_i2c_WaitAck01();
	if(a == 1) {
			return -1;
	}
	Buf[0]=BH1750_i2c_ReadByte01();
	BH1750_i2c_Ack01();
	Buf[1]=BH1750_i2c_ReadByte01();
	BH1750_i2c_NAck01();
	BH1750_i2c_Stop01();
	//从机回复时 先回复高八位数据 再回复低八位数据
	BH1750_Buf = ((Buf[0]) << 8) | Buf[1];
	temp[0] = (float)BH1750_Buf * 0.5 / 1.2;
	light[0] = BH1750_Buf;
 
	return 0;

}