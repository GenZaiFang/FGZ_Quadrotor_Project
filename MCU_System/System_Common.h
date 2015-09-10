
#ifndef System_Common_H_
#define System_Common_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "Global_Variable.h"

typedef uint8_t bool;
typedef float real_T;
typedef double real64_T;
typedef int8_t int8_T;
typedef int32_t int32_T;

#define true                                1
#define false                               0

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 	((uint32_t)0x08000000) /* Base @ of Sector  0,  16 Kbytes */
#define ADDR_FLASH_SECTOR_1 	((uint32_t)0x08004000) /* Base @ of Sector  1,  16 Kbytes */
#define ADDR_FLASH_SECTOR_2 	((uint32_t)0x08008000) /* Base @ of Sector  2,  16 Kbytes */
#define ADDR_FLASH_SECTOR_3 	((uint32_t)0x0800C000) /* Base @ of Sector  3,  16 Kbytes */
#define ADDR_FLASH_SECTOR_4 	((uint32_t)0x08010000) /* Base @ of Sector  4,  64 Kbytes */
#define ADDR_FLASH_SECTOR_5 	((uint32_t)0x08020000) /* Base @ of Sector  5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6 	((uint32_t)0x08040000) /* Base @ of Sector  6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7 	((uint32_t)0x08060000) /* Base @ of Sector  7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8 	((uint32_t)0x08080000) /* Base @ of Sector  8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9 	((uint32_t)0x080A0000) /* Base @ of Sector  9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10  ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11  ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

//串口1..3 波特率设置
#define Serial_1_Baudrate                9600
#define Serial_2_Baudrate              921600
#define Serial_3_Baudrate              115200

//DMA接收缓冲区长度
#define DMA_RX_LEN1                         2
#define DMA_RX_LEN2                      1024
#define DMA_RX_LEN3                        72

//串口1.2.3 接收中断使能位设置
#define USART1_INT_FLG                      1
#define USART2_INT_FLG                      1
#define USART3_INT_FLG                      1

#define USART1_DMA_Transfer                   //串口1是否使用DMA传送
#define USART2_DMA_Transfer                   //串口2是否使用DMA传送
#define USART3_DMA_Transfer                   //串口3是否使用DMA传送

//陀螺仪采样设置
//Sample_Group:采样的组数
//Sample_Group_times:每组采样的次数
//MPU6050_MAX_Error_Range:MPU6050陀螺仪采样值最大和最小的误差范围
//L3GD20_MAX_Error_Range:L3GD20陀螺仪采样值最大和最小的误差范围

#define Sample_Group                      100
#define Sample_Group_times                  1
#define MPU6050_MAX_Gyro_Error_Range       10
#define L3GD20_MAX_Gyro_Error_Range        80

//运行周期   单位毫秒
#define Deal_Period_ms                     15

#define PI                     3.14159265359f //圆周率
#define RAD_DEG            		 57.2957795056f //弧度转化成角度的比例因子
#define DEG_RAD                0.01745329252f //角度转化成弧度的比例因子
#define GRAVITY_MSS                  9.80665f //地球重力加速度
#define earthRate                0.000072921f //地球自转角速度
#define earthRadius                6378145.0f //地球半径
#define earthRadiusInv          1.5678540e-7f //地球半径的倒数

#define MX                                  0
#define MY                                  1
#define MZ                                  2

#define Close_Throttle                  13050 // 关闭电机
#define MIN_Throttle                    13850 // 电机转动最小值
#define MAX_Throttle                    24000 // 最高油门的80%

#define RC_CH1_MAX                      22700 // GPS模式选择通道
#define RC_CH1_MIN                      13200
 
#define RC_CH2_MAX                      22700 // 偏航角控制通道
#define RC_CH2_MID                      17980
#define RC_CH2_MIN                      13000
#define RC_CH2_RDN                      13800

#define W_Zrate_Deg_k                2.11e-5f // 偏航角速度

#define RC_CH3_MAX                      22700 // 油门控制通道
#define RC_CH3_MIN                      13200

#define Throttle_k                      1.05f // 油门系数

#define RC_CH4_MAX                      22700 // 横滚角控制通道
#define RC_CH4_MID                      17980
#define RC_CH4_MIN                      13200

#define Expect_Roll_Deg_k            0.00315f // 横滚角系数

#define RC_CH5_MAX                      22700 // 俯仰角控制通道
#define RC_CH5_MID                      18000
#define RC_CH5_MIN                      13200

#define Expect_Pitch_Deg_k           0.00315f // 俯仰角系数

#define Expect_Vn                    1.11e-5f // 期望向北速度系数
#define Expect_Ve                    1.11e-5f // 期望向东速度系数
#define Expect_Vd                    1.11e-5f // 期望向下速度系数

#define ProtectAngle                       35 // 倾斜保护角度

//#define Continuous_Correction                 // 是否每周期都修正状态

//#define USE_X_MODE                            // 是否转化成X模式

#define Control_UAV_Heading                   // 是否允许遥控器控制飞行器偏航角

//#define Disable_PID_Output                     // 是否关闭电机PID输出

//#define Disable_Motor_Output                     // 是否关闭电机

#define USE_MAG                               // 是否采用磁力计

#if defined USE_MAG

#define TILT_COMPENSATION                     // 倾斜补偿

#endif

#if 0

	#define USE_MAG_HMC5883L                    // 使用HMC5883L
	
#else

	#define USE_MAG_LSM303D                     // 使用LSM303D
	
#endif

#define OLED_Dispaly_Yaw                      // 使用OLED显示Yaw

#endif


