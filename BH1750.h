#ifndef __BH1750_H__
#define	__BH1750_H__
#include "stm32f4xx.h"
 
/*
������ַ��λ+��дλ 0д�Ĵ��� 1���Ĵ���
ADDR���� ��GND������ַ 0100 011X
         ��VCC������ַ 1011 100X
����ʹ��ʱADDR��������(�ӵ�) д�Ĵ���ʱ 0100 0110 0X46
                             ���Ĵ���ʱ 0100 0111 0X47
����ǿ�� =(�Ĵ���ֵ[15:0] * �ֱ���) / 1.2 ����λ���տ�˹lx��
*/
#define BH1750_ADDR_WRITE                    0x46    //�ӻ���ַ+���д����λ ADDR����Ϊ��ʱ
#define BH1750_ADDR_READ                     0x47    //�ӻ���ַ+��������λ ADDR����Ϊ��ʱ
 
#define BH1750_POWER_DOWN                    0x00    //No active state
                                                     //�޻״̬ �ر�ģ��
 
#define BH1750_POWER_ON                      0x01    //Wating for measurment command
                                                     //�ȴ��������� ��ģ��ȴ�����ָ��
 
#define BH1750_RESET                         0x07    //Reset data register value - not accepted in POWER_DOWN mode
                                                     //�������ݼĴ���ֵ-��POWER_DOWNģʽ����Ч POWER_ONģʽ����Ч
 
#define BH1750_CONTINUOUS_HIGH_RES_MODE      0x10    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //��1 lx�ֱ��ʿ�ʼ����������ʱ��ԼΪ120ms���߷ֱ���ģʽ1
 
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2    0x11    //Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
                                                     //��0.5 lx�ķֱ��ʿ�ʼ����������ʱ��ԼΪ120ms���߷ֱ���ģʽ2
 
#define BH1750_CONTINUOUS_LOW_RES_MODE       0x13    //Start measurement at 4lx resolution. Measurement time is approx 16ms.
                                                     //��4 lx�ֱ��ʿ�ʼ����������ʱ��ԼΪ16ms���ͷֱ���ģʽ
 
#define BH1750_ONE_TIME_HIGH_RES_MODE        0x20    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //��1 lx�ֱ��ʿ�ʼ����������ʱ��ԼΪ120ms��
                                                     //�������豸�Զ�����PowerDownģʽ��
 
#define BH1750_ONE_TIME_HIGH_RES_MODE_2      0x21    //Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //��0.5 lx�ķֱ��ʿ�ʼ����������ʱ��ԼΪ120ms��
                                                     //�������豸�Զ�����PowerDownģʽ��
 
#define BH1750_ONE_TIME_LOW_RES_MODE         0x23    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //��1 lx�ֱ��ʿ�ʼ����������ʱ��ԼΪ120ms��
                                                     //�������豸�Զ�����PowerDownģʽ��
 
 
//BH1750�ն����� �ṹ��
typedef struct
{
	float  BH1750_01;            //BH1750�նȴ���������1
	uint16_t  BH1750_01_Buf;
 
}BH1750_Typedef;
extern BH1750_Typedef BH1750Data;
 
 
void BH1750_InitI2C(void);
void BH1750_I2C_Write01(uint8_t cmd);
int8_t BH1750_I2C_Read01(float temp[],uint16_t light[]);
 
#endif