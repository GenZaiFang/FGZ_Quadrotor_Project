#include "pid.h"

#define minVal           1e-8f

float List_MaxMin(float num, float max, float min)
{
    if(num > max)
    {
        return max;
    }
    else if(num < min)
    {
        return min;
    }
    else
    {
        return num;
    }
}

void PID_Parameter_Clear(PID_Cal *objectCol)
{
		(*objectCol).W_error[0] = 0;
		(*objectCol).W_error[1] = 0;
		(*objectCol).W_error[2] = 0;
		(*objectCol).W_error[3] = 0;
		(*objectCol).W_Sum_error = 0;
}

void Positional_PID_Control(float expectVal, float nowVal, PID_Cal *objectCol, uint8_t dealFlag)
{
		if((*objectCol).pid_mode == P_Mode_CAL)
		{
				(*objectCol).Deg_error = expectVal - nowVal;	
			
				if(dealFlag == 1 && (*objectCol).Deg_error > 180)
				{
						(*objectCol).Deg_error -= 360;
				}
				else if(dealFlag == 1 && (*objectCol).Deg_error < -180)
				{
						(*objectCol).Deg_error += 360;
				}
				(*objectCol).Expect_gyro = (*objectCol).Kp[0] * (*objectCol).Deg_error;
		}
		else if((*objectCol).pid_mode == PID_Mode_CAL)
		{
				(*objectCol).W_error[0] = expectVal - nowVal;
				(*objectCol).W_Sum_error += (*objectCol).W_error[0];
				(*objectCol).W_Sum_error = List_MaxMin((*objectCol).W_Sum_error, MAXPower/((*objectCol).Ki[1] + minVal), MINPower/((*objectCol).Ki[1] + minVal));
				(*objectCol).W_error[2] = (*objectCol).W_error[0] - (*objectCol).W_error[1];
				(*objectCol).W_error[2] = ((*objectCol).W_error[2] + (*objectCol).W_error[3]) / 2;
				(*objectCol).W_error[1] = (*objectCol).W_error[0];
				(*objectCol).W_error[3] = (*objectCol).W_error[2];
				(*objectCol).PID_result = 
				(*objectCol).Kp[1] * (*objectCol).W_error[0] + (*objectCol).Ki[1] * (*objectCol).W_Sum_error + (*objectCol).Kd[1] * (*objectCol).W_error[2];
		}
}








