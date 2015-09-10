
#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "System_Common.h"

//定义HMC5883L内部地址及指令
#define	HMC5883L_SlaveAddress     0x3C	   //从机地址
#define A_CFG    0x00                      //(W/R)配置寄存器A，配置采样平均数，数据输出速率，测量配置，默认值：11110000
#define B_CFG    0x01                      //(W/R)配置寄存器B，配置增益；默认值：00100000
#define MODE_CFG 0x02                      //(W/R)模式寄存器，配置操作模式（连续，单一）默认值：00000001
#define XOUT_H   0x03                      //(R)数据输出寄存器X高8位
#define XOUT_L   0x04                      //(R)数据输出寄存器X低8位
#define ZOUT_H   0x05
#define ZOUT_L   0x06
#define YOUT_H   0x07
#define YOUT_L   0x08
#define STATUS   0x09                      //(R)状态寄存器
#define IDR_A    0x010                     //(R)识别寄存器A
#define IDR_B    0x011                     //(R)识别寄存器B
#define IDR_C    0x012                     //(R)识别寄存器C

//-----------------------------------------------------------------
//函数名称：hmc5883l_init
//功能概要：HMC5883L初始化    
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void MY_HMC5883L_init(void);

//-----------------------------------------------------------------
//函数名称：hmc5883l_read_mag
//功能概要：获取HMC5883的三轴磁场强度值，并存入首地址为dat的3个存储单元
//函数返回：void
//参数说明：dat :读取的3个磁场强度值存放的首地址
//-----------------------------------------------------------------
void MY_HMC5883L_Get_mag_Val(int16_t *dat);


void My_HMC5883L_Mag_Check(void);

float MY_HMC5883L_Get_angle(void);


#endif
