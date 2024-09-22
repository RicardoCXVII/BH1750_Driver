#ifndef __BH1750_H__
#define	__BH1750_H__
#include "stm32f4xx.h"
 
/*
器件地址七位+读写位 0写寄存器 1读寄存器
ADDR引脚 接GND器件地址 0100 011X
         接VCC器件地址 1011 100X
器件使用时ADDR引脚悬空(接地) 写寄存器时 0100 0110 0X46
                             读寄存器时 0100 0111 0X47
光照强度 =(寄存器值[15:0] * 分辨率) / 1.2 （单位：勒克斯lx）
*/
#define BH1750_ADDR_WRITE                    0x46    //从机地址+最后写方向位 ADDR引脚为低时
#define BH1750_ADDR_READ                     0x47    //从机地址+最后读方向位 ADDR引脚为低时
 
#define BH1750_POWER_DOWN                    0x00    //No active state
                                                     //无活动状态 关闭模块
 
#define BH1750_POWER_ON                      0x01    //Wating for measurment command
                                                     //等待测量命令 打开模块等待测量指令
 
#define BH1750_RESET                         0x07    //Reset data register value - not accepted in POWER_DOWN mode
                                                     //重置数据寄存器值-在POWER_DOWN模式下无效 POWER_ON模式下有效
 
#define BH1750_CONTINUOUS_HIGH_RES_MODE      0x10    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //以1 lx分辨率开始测量。测量时间约为120ms。高分辨率模式1
 
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2    0x11    //Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
                                                     //以0.5 lx的分辨率开始测量。测量时间约为120ms。高分辨率模式2
 
#define BH1750_CONTINUOUS_LOW_RES_MODE       0x13    //Start measurement at 4lx resolution. Measurement time is approx 16ms.
                                                     //以4 lx分辨率开始测量。测量时间约为16ms。低分辨率模式
 
#define BH1750_ONE_TIME_HIGH_RES_MODE        0x20    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //以1 lx分辨率开始测量。测量时间约为120ms。
                                                     //测量后，设备自动设置PowerDown模式。
 
#define BH1750_ONE_TIME_HIGH_RES_MODE_2      0x21    //Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //以0.5 lx的分辨率开始测量。测量时间约为120ms。
                                                     //测量后，设备自动设置PowerDown模式。
 
#define BH1750_ONE_TIME_LOW_RES_MODE         0x23    //Start measurement at 1lx resolution. Measurement time is approx 120ms.
                                                     //Device is automatically set to Power Down after measurement.
                                                     //以1 lx分辨率开始测量。测量时间约为120ms。
                                                     //测量后，设备自动设置PowerDown模式。
 
 
//BH1750照度数据 结构体
typedef struct
{
	float  BH1750_01;            //BH1750照度传感器数据1
	uint16_t  BH1750_01_Buf;
 
}BH1750_Typedef;
extern BH1750_Typedef BH1750Data;
 
 
void BH1750_InitI2C(void);
void BH1750_I2C_Write01(uint8_t cmd);
int8_t BH1750_I2C_Read01(float temp[],uint16_t light[]);
 
#endif