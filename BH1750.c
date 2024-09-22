#include "BH1750.h"
#include "delay.h"
#include "main.h"
/*
XS8     SCL->PB13    SDA->PB14    BH1750
*/
//����i2c�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������
#define GPIO_PORT_BH1750_01    GPIOB                   //BH1750       GPIO�˿�
#define RCC_BH1750_01_PORT     RCC_AHB1Periph_GPIOB    //BH1750       GPIO�˿�ʱ��
#define BH1750_01_SCL_PIN      GPIO_Pin_10             //BH1750       ʱ���� SCL -> PB13
#define BH1750_01_SDA_PIN      GPIO_Pin_11             //BH1750       ������ SDA -> PB14
//�����дSCL��SDA�ĺ�
#define BH1750_01_SCL_1        GPIO_PORT_BH1750_01->BSRRL = BH1750_01_SCL_PIN           //SCL = 1
#define BH1750_01_SCL_0        GPIO_PORT_BH1750_01->BSRRH = BH1750_01_SCL_PIN           //SCL = 0
#define BH1750_01_SDA_1        GPIO_PORT_BH1750_01->BSRRL = BH1750_01_SDA_PIN           //SDA = 1
#define BH1750_01_SDA_0        GPIO_PORT_BH1750_01->BSRRH = BH1750_01_SDA_PIN           //SDA = 0
#define BH1750_01_SDA_READ     ((GPIO_PORT_BH1750_01->IDR & BH1750_01_SDA_PIN) != 0)    //��SDA����״̬
#define BH1750_01_SCL_READ     ((GPIO_PORT_BH1750_01->IDR & BH1750_01_SCL_PIN) != 0)    //��SCL����״̬
 
BH1750_Typedef BH1750Data;
 
void BH1750_i2c_Stop01(void);
/*
�� �� ��: InitI2C
����˵��: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
��    ��: ��
�� �� ֵ: ��
*/
void BH1750_InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_BH1750_01_PORT, ENABLE);    //��GPIOʱ��
 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;         //��Ϊ�����
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;         //��Ϊ��©ģʽ
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;      //���������費ʹ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;      //IO������ٶ�
	GPIO_InitStructure.GPIO_Pin   = BH1750_01_SCL_PIN | BH1750_01_SDA_PIN;
	GPIO_Init(GPIO_PORT_BH1750_01, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_PORT_BH1750_01, BH1750_01_SCL_PIN | BH1750_01_SDA_PIN);    //�����
 
	//��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ
	BH1750_i2c_Stop01();
}
/*
��ʱ����
*/
static void BH1750_i2c_Delay(void)
{
	Delay_us(4);
}
/*
�� �� ��: i2c_Start
����˵��: CPU����I2C���������ź�
��    ��: ��
�� �� ֵ: ��
*/
void BH1750_i2c_Start01(void)
{
	//��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź�
	BH1750_01_SDA_1;
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SDA_0;
	BH1750_i2c_Delay();
	BH1750_01_SCL_0;
	BH1750_i2c_Delay();
}
/*
�� �� ��: i2c_Stop
����˵��: CPU����I2C����ֹͣ�ź�
��    ��: ��
�� �� ֵ: ��
*/
void BH1750_i2c_Stop01(void)
{
	//��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź�
	BH1750_01_SDA_0;
	BH1750_01_SCL_1;
	BH1750_i2c_Delay();
	BH1750_01_SDA_1;
	BH1750_i2c_Delay();
}
/*
�� �� ��: i2c_SendByte
����˵��: CPU��I2C�����豸����8bit����
��    ��: _ucByte �� �ȴ����͵��ֽ�
�� �� ֵ: ��
*/
void BH1750_i2c_SendByte01(uint8_t _ucByte)
{
	uint8_t i;
 
	//�ȷ����ֽڵĸ�λbit7
	for (i = 0; i < 8; i++)
	{
		if(_ucByte & 0x80)    BH1750_01_SDA_1;
		else                  BH1750_01_SDA_0;
		BH1750_i2c_Delay();
		BH1750_01_SCL_1;
		BH1750_i2c_Delay();
		BH1750_01_SCL_0;
		_ucByte <<= 1;    //����һ��bit
	}
	BH1750_i2c_Delay();
	BH1750_01_SDA_1;    //�ͷ�����
}
/*
�� �� ��: i2c_ReadByte
����˵��: CPU��I2C�����豸��ȡ8bit����
��    ��: ��
�� �� ֵ: ����������
*/
uint8_t BH1750_i2c_ReadByte01(void)
{
	uint8_t i;
	uint8_t value = 0;
	//������1��bitΪ���ݵ�bit7
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
�� �� ��: i2c_WaitAck
����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
��    ��: ��
�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
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
�� �� ��: i2c_Ack
����˵��: CPU����һ��ACK�ź�
��    ��: ��
�� �� ֵ: ��
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
�� �� ��: i2c_NAck
����˵��: CPU����1��NACK�ź�
��    ��: ��
�� �� ֵ: ��
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
�� �� ��: I2C_Write
����˵��: дһ���ֽ�
��    ��: cmd:Ҫд���һ���ֽ�����
�� �� ֵ: ��
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
�� �� ��: I2C_Read
����˵��: ��ȡ����ֵ
��    ��: temp[]:�ն����ֵ  light[]���Ĵ�����������ֵ
�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
����ǿ�� =(�Ĵ���ֵ[15:0] * �ֱ���) / 1.2 ����λ���տ�˹lx��
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
	//�ӻ��ظ�ʱ �Ȼظ��߰�λ���� �ٻظ��Ͱ�λ����
	BH1750_Buf = ((Buf[0]) << 8) | Buf[1];
	temp[0] = (float)BH1750_Buf * 0.5 / 1.2;
	light[0] = BH1750_Buf;
 
	return 0;

}