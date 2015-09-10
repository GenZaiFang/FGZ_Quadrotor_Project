
#ifndef Global_Variable_H_
#define Global_Variable_H_

#include "System_Common.h"

typedef struct 
{
    float Pos[3]; // Position in meters and relative to a local NED frame ???????? ?? m X[0] X[1] X[2]
    float Vel[3]; // Velocity in meters and in NED ????? ?? m/s X[3] X[4] X[5]
    float q[4]; // unit quaternion rotation relative to NED ??? ??? X[6] X[7] X[8] X[9]
    float gyro_bias[3]; // ????? X[10] X[11] X[12]
    float accel_bias[3];// ????? X[13] X[14] X[15]
} NavStruct;

extern NavStruct Nav;

typedef struct  
{
	uint8_t id[4];
	uint8_t updated;
	struct 
	{
		int16_t axis[3];
	} raw;
	struct 
	{
		float axis[3];
	} scaled;
	struct 
	{
		float bias[3];
		float scale[3];
		float variance[3];
	} calibration;
}MAG_Sensor_Struct;
extern MAG_Sensor_Struct Global_Mag_Sensor;

typedef struct  
{
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
	} offset;
	struct 
	{
		float x;
		float y;
		float z;
	} filtered;
	struct 
	{
		float scale[3][4];
		float variance[3];
	} calibration;
}Accel_Sensor_Struct;
extern Accel_Sensor_Struct Global_Accel_Sensor;

typedef struct  
{
	struct 
	{
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} raw;
	struct 
	{
		float x;
		float y;
		float z;
	} filtered;
	struct 
	{
		float bias[3];
		float scale[3];
		float variance[3];
		float tempcompfactor[3];
	} calibration;
	struct 
	{
		uint16_t xy;
		uint16_t z;
	} temp;
}Gyro_Sensor_Struct;
extern Gyro_Sensor_Struct Global_Gyro_Sensor;

typedef struct  
{
	float altitude;
	uint8_t updated;
}Altitude_Sensor_Struct;
extern Altitude_Sensor_Struct Global_Altitude_Sensor;

typedef struct  
{
	float NED_Pos[3];
	float NED_Vel[3];
	float heading;
	float groundspeed;
	float quality;
	uint8_t updated;
}GPS_Sensor_Struct;
extern GPS_Sensor_Struct Global_GPS_Sensor;

typedef struct
{
		float is_pid_update;
		float heading_angle;
		float throttle;
		float expect_euler[3];
		float expect_Wz;
		float expect_vel[3];
		float expect_position[3];
		float ref_pos[3];
		float posError[3];
		float lasr_posError[3];
		float sum_posError[3];
}Aircraft_Struct;
extern Aircraft_Struct Aircraft;

typedef struct
{	
		float Bp[6][3];
		float Bp_Length[6];
		float Bp_MatY[6];
		float Bp_MatX[6 * 4];
		float Bp_BeiTa[4];
		float Bp_Hard_Iron_V[3];
		float Geo_B;
		float inv_M[9];
}Magnetometer_Calibration_Struct;
extern Magnetometer_Calibration_Struct Mag_LSM303D_Calibration;

typedef union
{
		float Flash_float;
		uint8_t Flash_byte[4];				
		uint16_t Flash_half_word[2];
		uint32_t Flash_word;	
}Flash_Operation_Union;

typedef struct
{
		Flash_Operation_Union X;
		Flash_Operation_Union Y;
		Flash_Operation_Union Z;
}Flash_Operation_Struct;

extern Flash_Operation_Struct Flash_Mag_LSM303D_Calibration;

extern uint32_t Global_RunTime;

extern float Global_Heading_Ref;
extern float Global_Heading_MAX_Ref;
extern float Global_Heading_MIN_Ref;

extern float Global_Now_Euler[3];

extern float Global_Expect_Euler[3];
extern float Global_W_Rate[3];
extern int16_t Global_Gyro_Val[3], Global_Accle_Val[3], Global_Mag_Val[3];
extern double Global_MPU6050_Gyro_Offset_Val[3];
extern double Global_L3GD20_Gyro_Offset_Val[3];
//磁力计校正偏差
extern int16_t Global_Mag_Offset_Val[3];
extern float Global_Show_Val[22];
extern uint8_t Global_uint8_Val[1024];

extern uint8_t Global_DMA2_Stream5_Rx_Buf[1024];  // UART1
extern uint8_t Global_DMA1_Stream5_Rx_Buf[1024];	// UART2
extern uint8_t Global_DMA1_Stream1_Rx_Buf[1024];  // UART3

extern float Global_hgtRef;

extern float Global_accle_m_s2[3], Global_gyro_rad_s[3], Global_Q[4], Global_Init_Q[4];

extern uint8_t isFirst;
extern uint8_t Global_GPS_Ref_Sample_NONE;
extern uint8_t Global_HOME_Check_NONE;

extern uint8_t Global_GPS_Health;

extern uint16_t Global_RC[6];
extern uint8_t Global_RC_UNLOCK;

extern uint16_t Global_U_Height;


#endif
