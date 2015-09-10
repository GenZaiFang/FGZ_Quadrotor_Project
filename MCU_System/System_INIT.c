#include "System_All_Head.h"

/****************************Global变量声明*******************************/

uint32_t Global_RunTime;

Aircraft_Struct Aircraft;

Magnetometer_Calibration_Struct Mag_LSM303D_Calibration;

Flash_Operation_Struct Flash_Mag_LSM303D_Calibration;

float Global_Now_Euler[3];
float Global_Heading_Ref = 0;
float Global_Heading_MAX_Ref = 0;
float Global_Heading_MIN_Ref = 0;
float Global_Expect_Euler[3];
float Global_W_Rate[3];
int16_t Global_Gyro_Val[3], Global_Accle_Val[3], Global_Mag_Val[3];
double Global_MPU6050_Gyro_Offset_Val[3];
double Global_L3GD20_Gyro_Offset_Val[3];
int16_t Global_Mag_Offset_Val[3];
float Global_Show_Val[22];
uint8_t Global_uint8_Val[1024];
uint8_t Global_DMA2_Stream5_Rx_Buf[1024];
uint8_t Global_DMA1_Stream5_Rx_Buf[1024];
uint8_t Global_DMA1_Stream1_Rx_Buf[1024];
nmea_msg g_gps; 			 //GPS信息

float Global_hgtRef;
uint8_t isFirst = true;
uint8_t Global_GPS_Ref_Sample_NONE = false; // GPS参考数据获取完成标志位
uint8_t Global_HOME_Check_NONE = false;
uint8_t Global_GPS_Health = false;

float Global_accle_m_s2[3], Global_gyro_rad_s[3], Global_Q[4], Global_Init_Q[4];

NavStruct Nav;       //导航结构体

//传感器信息
MAG_Sensor_Struct           Global_Mag_Sensor;
Accel_Sensor_Struct         Global_Accel_Sensor;
Gyro_Sensor_Struct          Global_Gyro_Sensor;
Altitude_Sensor_Struct      Global_Altitude_Sensor;
GPS_Sensor_Struct           Global_GPS_Sensor;

uint16_t Global_RC[6];
uint8_t Global_RC_UNLOCK = false;

uint16_t Global_U_Height = 0;


/*********************************************************************/

uint32_t config_baudrate[3] = {Serial_1_Baudrate, Serial_2_Baudrate, Serial_3_Baudrate};
uint8_t config_INT_EN_Flag[3] = {USART1_INT_FLG, USART2_INT_FLG, USART3_INT_FLG};

//-----------------------------------------------------------------//
//函数名称：My_STM32_SYSTEM_INIT
//功能概要：STM32初始化
//函数返回：void
//参数说明：void
//-----------------------------------------------------------------//
void My_STM32_SYSTEM_INIT(void)
{
    My_STM32_disable_interrupts();			
    My_STM32_LED_init();		
    My_STM32_USART_init(config_baudrate, config_INT_EN_Flag);
		My_USARTn_DMA_Config();
    My_STM32_I2C_init();
    My_MPU6050_init();
		MY_HMC5883L_init();
		My_LSM303D_init();		
    My_L3GD20_init();
    My_MS5611_init();		
		My_Aircraft_init();	
		My_OLED_init();
    My_STM32_TIMER_init();				
    My_STM32_enable_interrupts();
}
