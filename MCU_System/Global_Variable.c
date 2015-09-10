
#include "Global_Variable.h"

extern uint32_t Global_RunTime;

extern NavStruct Nav;

extern MAG_Sensor_Struct Global_Mag_Sensor;
extern Accel_Sensor_Struct Global_Accel_Sensor;
extern Gyro_Sensor_Struct Global_Gyro_Sensor;
extern Altitude_Sensor_Struct Global_Altitude_Sensor;
extern GPS_Sensor_Struct Global_GPS_Sensor;

extern Aircraft_Struct Aircraft;

extern Magnetometer_Calibration_Struct Mag_LSM303D_Calibration;

extern Flash_Operation_Struct Flash_Mag_LSM303D_Calibration;

//欧拉角
extern float Global_Now_Euler[3];
extern float Global_Heading_Ref;
extern float Global_Heading_MAX_Ref;
extern float Global_Heading_MIN_Ref;
//期望的欧拉角
extern float Global_Expect_Euler[3];

extern float Global_W_Rate[3];
//陀螺仪数据、加速度计数据、磁力计数据
extern int16_t Global_Gyro_Val[3], Global_Accle_Val[3], Global_Mag_Val[3];
//MPU6050陀螺仪零飘
extern double Global_MPU6050_Gyro_Offset_Val[3];
//L3GD20陀螺仪零飘
extern double Global_L3GD20_Gyro_Offset_Val[3];

//磁力计校正偏差
extern int16_t Global_Mag_Offset_Val[3];

extern float Global_Show_Val[22];

extern uint8_t Global_uint8_Val[1024];

extern uint8_t Global_DMA2_Stream5_Rx_Buf[1024];
extern uint8_t Global_DMA1_Stream5_Rx_Buf[1024];
extern uint8_t Global_DMA1_Stream1_Rx_Buf[1024];

extern float Global_hgtRef;

extern uint8_t isFirst;
extern uint8_t Global_GPS_Ref_Sample_NONE;
extern uint8_t Global_HOME_Check_NONE;

extern uint8_t Global_GPS_Health;

extern float Global_accle_m_s2[3], Global_gyro_rad_s[3], Global_Q[4], Global_Init_Q[4];

extern uint16_t Global_RC[6];
extern uint8_t Global_RC_UNLOCK;

extern uint16_t Global_U_Height;
