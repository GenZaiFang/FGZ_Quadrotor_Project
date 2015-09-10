#ifndef PID_H_
#define PID_H_

#include "System_Common.h"

#define MAXPower       (MAX_Throttle - MIN_Throttle)
#define MINPower  -1 * (MAX_Throttle - MIN_Throttle)

typedef enum
{
		P_Mode_CAL = 0,
		PID_Mode_CAL = 1
}PID_MODE;

typedef struct
{		
		PID_MODE pid_mode;
		float Kp[2];
		float Ki[2];
		float Kd[2];
		float Deg_error;
		float W_error[4];
		float Expect_gyro;
		float W_Sum_error;
		float PID_result;
}PID_Cal;

float List_MaxMin(float num, float max, float min);
void PID_Parameter_Clear(PID_Cal *objectCol);
void Positional_PID_Control(float expectVal, float nowVal, PID_Cal *objectCol, uint8_t dealFlag);

#endif
