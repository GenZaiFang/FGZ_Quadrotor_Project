#include "AttitudeControl.h"
#include "pid.h"
#include "My_Timer.h"

PID_Cal Roll_X_Cal, Pitch_Y_Cal, Yaw_Z_Cal;
float W_Rate[3];
static uint8_t isInit = false;

uint16_t Motor_output[4];

void My_Aircraft_init(void)
{
		Aircraft.is_pid_update = false;
		Aircraft.throttle = 0;
		Aircraft.expect_euler[0] = 0;
		Aircraft.expect_euler[1] = 0;
		Aircraft.expect_euler[2] = 0;
		Aircraft.expect_Wz = 0;
		Aircraft.expect_vel[0] = 0;
		Aircraft.expect_vel[1] = 0;
		Aircraft.expect_vel[2] = 0;
		Aircraft.expect_position[0] = 0;
		Aircraft.expect_position[1] = 0;
		Aircraft.expect_position[2] = 0;
		Aircraft.ref_pos[0] = 0;
		Aircraft.ref_pos[1] = 0;
		Aircraft.ref_pos[2] = 0;
		Aircraft.posError[0] = 0;
		Aircraft.posError[1] = 0;
		Aircraft.posError[2] = 0;
		Aircraft.lasr_posError[0] = 0;
		Aircraft.lasr_posError[1] = 0;
		Aircraft.lasr_posError[2] = 0;
		Aircraft.sum_posError[0] = 0;
		Aircraft.sum_posError[1] = 0;
		Aircraft.sum_posError[2] = 0;
}

static void PID_init(void)
{	
		Roll_X_Cal.pid_mode = P_Mode_CAL;
	
		Roll_X_Cal.Kp[0] = 6.8f;
		Roll_X_Cal.Ki[0] = 0;
		Roll_X_Cal.Kd[0] = 0;
		Roll_X_Cal.Kp[1] = 6.8f;
		Roll_X_Cal.Ki[1] = 0.3f;
		Roll_X_Cal.Kd[1] = 15.8f;
	
		Roll_X_Cal.W_error[0] = 0;
		Roll_X_Cal.W_error[1] = 0;
		Roll_X_Cal.W_error[2] = 0;
		Roll_X_Cal.W_error[3] = 0;
	
		Roll_X_Cal.W_Sum_error = 0;
	
		Roll_X_Cal.Deg_error = 0;
		Roll_X_Cal.Expect_gyro = 0;
		
		Roll_X_Cal.PID_result = 0;
	
	
		Pitch_Y_Cal.pid_mode = P_Mode_CAL;
	
		Pitch_Y_Cal.Kp[0] = 6.8f;
		Pitch_Y_Cal.Ki[0] = 0;
		Pitch_Y_Cal.Kd[0] = 0;
		Pitch_Y_Cal.Kp[1] = 6.8f;
		Pitch_Y_Cal.Ki[1] = 0.3f;
		Pitch_Y_Cal.Kd[1] = 15.8f;
	
		Pitch_Y_Cal.W_error[0] = 0;
		Pitch_Y_Cal.W_error[1] = 0;
		Pitch_Y_Cal.W_error[2] = 0;
		Pitch_Y_Cal.W_error[3] = 0;
	
		Pitch_Y_Cal.W_Sum_error = 0;
	
		Pitch_Y_Cal.Deg_error = 0;
		Pitch_Y_Cal.Expect_gyro = 0;
		
		Pitch_Y_Cal.PID_result = 0;
		
		
		Yaw_Z_Cal.pid_mode = P_Mode_CAL;
	
		Yaw_Z_Cal.Kp[0] = 15.0f;
		Yaw_Z_Cal.Ki[0] = 0;
		Yaw_Z_Cal.Kd[0] = 0;
		Yaw_Z_Cal.Kp[1] = 15.0f;
		Yaw_Z_Cal.Ki[1] = 0.3f;
		Yaw_Z_Cal.Kd[1] = 20.8f;
	
		Yaw_Z_Cal.W_error[0] = 0;
		Yaw_Z_Cal.W_error[1] = 0;
		Yaw_Z_Cal.W_error[2] = 0;
		Yaw_Z_Cal.W_error[3] = 0;
	
		Yaw_Z_Cal.W_Sum_error = 0;
	
		Yaw_Z_Cal.Deg_error = 0;
		Yaw_Z_Cal.Expect_gyro = 0;
		
		Yaw_Z_Cal.PID_result = 0;
}

void PID_RESET(void)
{
		PID_Parameter_Clear(&Yaw_Z_Cal);
		PID_Parameter_Clear(&Pitch_Y_Cal);
		PID_Parameter_Clear(&Roll_X_Cal);
}

void Attitude_Control(void)
{
		if(!isInit)
		{	
				PID_init();
				isInit = true;
		}
		else
		{
				W_Rate[0] = ((float)Global_Gyro_Val[0] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[0]) * RAD_DEG;
				W_Rate[1] = ((float)Global_Gyro_Val[1] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[1]) * RAD_DEG;
				W_Rate[2] = ((float)Global_Gyro_Val[2] / 16.4f * DEG_RAD - (float)Global_MPU6050_Gyro_Offset_Val[2]) * RAD_DEG;
			
		#if defined USE_X_MODE
			
				Global_W_Rate[0] = W_Rate[0] * 0.707f +  W_Rate[1] * 0.707f;
				Global_W_Rate[1] = W_Rate[1] * 0.707f -  W_Rate[0] * 0.707f;
				Global_W_Rate[2] = W_Rate[2];
			
		#else
			
				Global_W_Rate[0] = W_Rate[0];
				Global_W_Rate[1] = W_Rate[1];
				Global_W_Rate[2] = W_Rate[2];
			
		#endif
			
				Aircraft.expect_euler[0] = List_MaxMin(Aircraft.expect_euler[0], 15, -15);
				Aircraft.expect_euler[1] = List_MaxMin(Aircraft.expect_euler[1], 15, -15);
				
			
				Roll_X_Cal.pid_mode = P_Mode_CAL;
				Positional_PID_Control(Aircraft.expect_euler[0], Global_Now_Euler[0], &Roll_X_Cal, 0);	
				Roll_X_Cal.pid_mode = PID_Mode_CAL;
				Positional_PID_Control(Roll_X_Cal.Expect_gyro, Global_W_Rate[0], &Roll_X_Cal, 0);
				
				Pitch_Y_Cal.pid_mode = P_Mode_CAL;
				Positional_PID_Control(Aircraft.expect_euler[1], Global_Now_Euler[1], &Pitch_Y_Cal, 0);	
				Pitch_Y_Cal.pid_mode = PID_Mode_CAL;
				Positional_PID_Control(Pitch_Y_Cal.Expect_gyro, Global_W_Rate[1], &Pitch_Y_Cal, 0);
								
				Yaw_Z_Cal.pid_mode = P_Mode_CAL;
				
		#if defined Control_UAV_Heading
		
				Positional_PID_Control(Aircraft.expect_euler[2], Global_Now_Euler[2], &Yaw_Z_Cal, 1);
				
		#else		
		
				Positional_PID_Control(Global_Expect_Euler[2], Global_Now_Euler[2], &Yaw_Z_Cal, 1);	
				
		#endif
					
				Yaw_Z_Cal.pid_mode = PID_Mode_CAL;
				Positional_PID_Control(Yaw_Z_Cal.Expect_gyro, Global_W_Rate[2], &Yaw_Z_Cal, 0);

		#if defined Disable_PID_Output
		
				Roll_X_Cal.PID_result  = 0;
				Pitch_Y_Cal.PID_result = 0;
				Yaw_Z_Cal.PID_result   = 0;
				
		#endif
				
				Motor_output[0] = (float)Aircraft.throttle - Roll_X_Cal.PID_result + Pitch_Y_Cal.PID_result - Yaw_Z_Cal.PID_result;
				Motor_output[1] = (float)Aircraft.throttle - Roll_X_Cal.PID_result - Pitch_Y_Cal.PID_result + Yaw_Z_Cal.PID_result;
				Motor_output[2] = (float)Aircraft.throttle + Roll_X_Cal.PID_result - Pitch_Y_Cal.PID_result - Yaw_Z_Cal.PID_result;
				Motor_output[3] = (float)Aircraft.throttle + Roll_X_Cal.PID_result + Pitch_Y_Cal.PID_result + Yaw_Z_Cal.PID_result;
		
				Motor_output[0] = List_MaxMin(Motor_output[0], MAX_Throttle, MIN_Throttle);
				Motor_output[1] = List_MaxMin(Motor_output[1], MAX_Throttle, MIN_Throttle);
				Motor_output[2] = List_MaxMin(Motor_output[2], MAX_Throttle, MIN_Throttle);
				Motor_output[3] = List_MaxMin(Motor_output[3], MAX_Throttle, MIN_Throttle);			
				
		#if defined Disable_Motor_Output
		
				Tim1_PWM_Output(Close_Throttle, Close_Throttle, Close_Throttle, Close_Throttle);
		
		#else
		
				Tim1_PWM_Output(Motor_output[0], Motor_output[1], Motor_output[2], Motor_output[3]);
				
		#endif
		}
}
