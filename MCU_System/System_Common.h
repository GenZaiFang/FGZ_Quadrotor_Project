
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

//����1..3 ����������
#define Serial_1_Baudrate                9600
#define Serial_2_Baudrate              921600
#define Serial_3_Baudrate              115200

//DMA���ջ���������
#define DMA_RX_LEN1                         2
#define DMA_RX_LEN2                      1024
#define DMA_RX_LEN3                        72

//����1.2.3 �����ж�ʹ��λ����
#define USART1_INT_FLG                      1
#define USART2_INT_FLG                      1
#define USART3_INT_FLG                      1

#define USART1_DMA_Transfer                   //����1�Ƿ�ʹ��DMA����
#define USART2_DMA_Transfer                   //����2�Ƿ�ʹ��DMA����
#define USART3_DMA_Transfer                   //����3�Ƿ�ʹ��DMA����

//�����ǲ�������
//Sample_Group:����������
//Sample_Group_times:ÿ������Ĵ���
//MPU6050_MAX_Error_Range:MPU6050�����ǲ���ֵ������С����Χ
//L3GD20_MAX_Error_Range:L3GD20�����ǲ���ֵ������С����Χ

#define Sample_Group                      100
#define Sample_Group_times                  1
#define MPU6050_MAX_Gyro_Error_Range       10
#define L3GD20_MAX_Gyro_Error_Range        80

//��������   ��λ����
#define Deal_Period_ms                     15

#define PI                     3.14159265359f //Բ����
#define RAD_DEG            		 57.2957795056f //����ת���ɽǶȵı�������
#define DEG_RAD                0.01745329252f //�Ƕ�ת���ɻ��ȵı�������
#define GRAVITY_MSS                  9.80665f //�����������ٶ�
#define earthRate                0.000072921f //������ת���ٶ�
#define earthRadius                6378145.0f //����뾶
#define earthRadiusInv          1.5678540e-7f //����뾶�ĵ���

#define MX                                  0
#define MY                                  1
#define MZ                                  2

#define Close_Throttle                  13050 // �رյ��
#define MIN_Throttle                    13850 // ���ת����Сֵ
#define MAX_Throttle                    24000 // ������ŵ�80%

#define RC_CH1_MAX                      22700 // GPSģʽѡ��ͨ��
#define RC_CH1_MIN                      13200
 
#define RC_CH2_MAX                      22700 // ƫ���ǿ���ͨ��
#define RC_CH2_MID                      17980
#define RC_CH2_MIN                      13000
#define RC_CH2_RDN                      13800

#define W_Zrate_Deg_k                2.11e-5f // ƫ�����ٶ�

#define RC_CH3_MAX                      22700 // ���ſ���ͨ��
#define RC_CH3_MIN                      13200

#define Throttle_k                      1.05f // ����ϵ��

#define RC_CH4_MAX                      22700 // ����ǿ���ͨ��
#define RC_CH4_MID                      17980
#define RC_CH4_MIN                      13200

#define Expect_Roll_Deg_k            0.00315f // �����ϵ��

#define RC_CH5_MAX                      22700 // �����ǿ���ͨ��
#define RC_CH5_MID                      18000
#define RC_CH5_MIN                      13200

#define Expect_Pitch_Deg_k           0.00315f // ������ϵ��

#define Expect_Vn                    1.11e-5f // �������ٶ�ϵ��
#define Expect_Ve                    1.11e-5f // �������ٶ�ϵ��
#define Expect_Vd                    1.11e-5f // ���������ٶ�ϵ��

#define ProtectAngle                       35 // ��б�����Ƕ�

//#define Continuous_Correction                 // �Ƿ�ÿ���ڶ�����״̬

//#define USE_X_MODE                            // �Ƿ�ת����Xģʽ

#define Control_UAV_Heading                   // �Ƿ�����ң�������Ʒ�����ƫ����

//#define Disable_PID_Output                     // �Ƿ�رյ��PID���

//#define Disable_Motor_Output                     // �Ƿ�رյ��

#define USE_MAG                               // �Ƿ���ô�����

#if defined USE_MAG

#define TILT_COMPENSATION                     // ��б����

#endif

#if 0

	#define USE_MAG_HMC5883L                    // ʹ��HMC5883L
	
#else

	#define USE_MAG_LSM303D                     // ʹ��LSM303D
	
#endif

#define OLED_Dispaly_Yaw                      // ʹ��OLED��ʾYaw

#endif


