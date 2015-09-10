/*******************************************************************************************************************/
/**************通过设计一个EKF滤波器将MAG(M：磁力计 A：加速度计 G：陀螺仪)三种传感器一起融合估计姿态Attitude*************/
/*****该EKF包含了磁力计的校准(倾斜补偿、Soft iron、Hard iron)，不同于EKF_Attitude.c里面姿态估计和磁力计校准分开的EKF*****/
/*******************************************************************************************************************/
#include "EKF_MAG_Fusion.h"
#include "CommonConversions.h"
#include "CommonPlan.h"
#include "My_USART.h"

#define Xn 9
#define Qn 9
#define Zn 3
#define Rn 3

static float EKF_X[Xn];
static float EKF_Z[Zn];
static float EKF_P[Xn*Xn];
static float EKF_Q[Qn];
static float EKF_R[Rn];
static float EKF_K[Xn*Zn];
static float EKF_F[Xn*Xn]; 
static float EKF_H[Zn*Xn];

static float ekf_x_Tmp[Xn];
static int i;

static float FP[Xn*Xn];
static float FT[Xn*Xn];

static float HT[Xn*Zn];
static float PHT[Xn*Zn];
static float HP[Zn*Xn];
static float HPHT[Zn*Zn];
static float inv_HPHT[Zn*Zn];

static float HX[Zn*1];
static float KZ[Xn*1];


static float KH[Xn*Xn];
static float KHP[Xn*Xn];

static uint8_t isInit = false;
static float dT = Deal_Period_ms / 1000.0f;
static float gyro[3], mag_data[3];

static void EKF_Clear(void)
{
		for(i = 0; i < Xn; i++)
		{
				EKF_X[i] = 0;
		}
		for(i = 0; i < Qn; i++)
		{
				EKF_Q[i] = 0;
		}
		for(i = 0; i < Rn; i++)
		{
				EKF_R[i] = 0;
		}
		for(i = 0; i < Xn * Xn; i++)
		{
				EKF_P[i] = 0;
				EKF_F[i] = 0;
		}
		for(i = 0; i < Zn; i++)
		{
				EKF_Z[i] = 0;
		}
		for(i = 0; i < Xn*Zn; i++)
		{
				EKF_K[i] = 0;
		}
		for(i = 0; i < Zn*Xn; i++)
		{
				EKF_H[i] = 0;
		}
}

static void EKF_AttitudeInit(void)
{	
		EKF_Clear();
	
		EKF_P[0] = EKF_P[10] = EKF_P[20] = 1e-6f;
		EKF_P[30] = EKF_P[40] = EKF_P[50] = 1e-12f;
		EKF_P[60] = EKF_P[70] = EKF_P[80] = 1e-12f;
	
		EKF_X[0] = (float)Global_Mag_Val[0];
		EKF_X[1] = (float)Global_Mag_Val[1];
		EKF_X[2] = (float)Global_Mag_Val[2];
	
		EKF_X[3] = EKF_X[4] = EKF_X[5] = 1.0f;
	
		EKF_X[6] = EKF_X[7] = EKF_X[8] = 0.0f;		
			
		EKF_Q[0]  = EKF_Q[1] = EKF_Q[2] = 1e-6f;
		EKF_Q[3]  = EKF_Q[4] = EKF_Q[5] = 1e-6f;
		EKF_Q[6]  = EKF_Q[7] = EKF_Q[8] = 1e-6f;
	
		EKF_R[0]  = EKF_R[1] = EKF_R[2] = 1e-6f;

}

static void EKF_CAL_F_H_Matrix(const float gyro_u[3], float dt)
{
		EKF_F[0*Xn+0] = 1;
		EKF_F[0*Xn+1] = gyro_u[2] * dt;
		EKF_F[0*Xn+2] = -1 * gyro_u[1] * dt;
		
		EKF_F[1*Xn+0] = -1 * gyro_u[2] * dt;
		EKF_F[1*Xn+1] = 1;
		EKF_F[1*Xn+2] = gyro_u[0] * dt;
	
		EKF_F[2*Xn+0] = gyro_u[1] * dt;
		EKF_F[2*Xn+1] = -1 * gyro_u[0] * dt;
		EKF_F[2*Xn+2] = 1;
	
		EKF_F[3*Xn+3] = 1;
		EKF_F[4*Xn+4] = 1;
		EKF_F[5*Xn+5] = 1;
		EKF_F[6*Xn+6] = 1;
		EKF_F[7*Xn+7] = 1;
		EKF_F[8*Xn+8] = 1;
	
		
		EKF_H[0*Xn+0] = EKF_X[3];//Gx		
		EKF_H[0*Xn+3] = 0;//EKF_X[0];// + EKF_X[1] + EKF_X[2];//mx+my+mz
		EKF_H[0*Xn+6] = 1;
				
		EKF_H[1*Xn+1] = EKF_X[4];//Gy		
		EKF_H[1*Xn+4] = 0;//EKF_X[1];// + EKF_X[1] + EKF_X[2];//mx+my+mz
		EKF_H[1*Xn+7] = 1;
		
		EKF_H[2*Xn+2] = EKF_X[5];//Gz
		EKF_H[2*Xn+5] = 0;//EKF_X[2];// + EKF_X[1] + EKF_X[2];//mx+my+mz
		EKF_H[2*Xn+8] = 1;
}

static void EKF_X_Prediction(float x[Xn], float gyro_u[3], float dt)
{
		for(i = 0; i < 3; i++)
	  {
				ekf_x_Tmp[i] = x[i];
		}
		
		x[0] = ekf_x_Tmp[0] - (gyro_u[1] * ekf_x_Tmp[2] - gyro_u[2] * ekf_x_Tmp[1]) * dt;
		x[1] = ekf_x_Tmp[1] - (gyro_u[2] * ekf_x_Tmp[0] - gyro_u[0] * ekf_x_Tmp[2]) * dt;
		x[2] = ekf_x_Tmp[2] - (gyro_u[0] * ekf_x_Tmp[1] - gyro_u[1] * ekf_x_Tmp[0]) * dt;
}

static void EKF_CovariancePrediction(void)
{
		ML_R_X_ML_R(EKF_F, EKF_P, FP, Xn, Xn, Xn, Xn);
		Matrix_Tran(EKF_F, FT, Xn, Xn);
		ML_R_X_ML_R(FP, FT, EKF_P, Xn, Xn, Xn, Xn);
	
		for(i = 0; i < Xn; i++)
		{
				EKF_P[i*Xn+i] += EKF_Q[i];
		}
}

static void EKF_CAL_K(void)
{
		Matrix_Tran(EKF_H, HT, Zn, Xn);
		ML_R_X_ML_R(EKF_P, HT, PHT, Xn, Xn, Xn, Zn);
		ML_R_X_ML_R(EKF_H, EKF_P, HP, Zn, Xn, Xn, Xn);
		ML_R_X_ML_R(HP, HT, HPHT, Zn, Xn, Xn, Zn);
		for(i = 0; i < Zn; i++)
		{
				HPHT[i*Zn+i] += EKF_R[i];
		}
		Matrix_3X3_Inv(HPHT, inv_HPHT);
		ML_R_X_ML_R(PHT, inv_HPHT, EKF_K, Xn, Zn, Zn, Zn);
}

static void EKF_Correction_P(void)
{
		ML_R_X_ML_R(EKF_K, EKF_H, KH, Xn, Zn, Zn, Xn);

		ML_R_X_ML_R(KH, EKF_P, KHP, Xn, Xn, Xn, Xn);
	
		for(i = 0; i < Xn * Xn; i++)
		{
				EKF_P[i] -= KHP[i];
		}
}

static void EKF_Correction_X(const float mag_u[3])
{
		ML_R_X_ML_R(EKF_H, EKF_X, HX, Zn, Xn, Xn, 1);
		for(i = 0; i < Zn; i++)
		{
				EKF_Z[i] = mag_u[i] - HX[i];
		}
		ML_R_X_ML_R(EKF_K, EKF_Z, KZ, Xn, Zn, Zn, 1);
		for(i = 0; i < Xn; i++)
		{
				EKF_X[i] += KZ[i];
		}
}

void EKF_Mag_Calimation(void)
{
		if(!isInit)
		{
				EKF_AttitudeInit();
				isInit = true;
		}
		else
		{
				gyro[0] = Global_Gyro_Sensor.filtered.x;
				gyro[1] = Global_Gyro_Sensor.filtered.y;
				gyro[2] = Global_Gyro_Sensor.filtered.z;
								
				mag_data[0] = (float)Global_Mag_Val[0];
				mag_data[1] = (float)Global_Mag_Val[1];
				mag_data[2] = (float)Global_Mag_Val[2];
			
				EKF_CAL_F_H_Matrix(gyro, dT);
			
				EKF_X_Prediction(EKF_X, gyro, dT);				
				EKF_CovariancePrediction();
				EKF_CAL_K();
				EKF_Correction_P();
				EKF_Correction_X(mag_data);
				
				Global_Show_Val[5] = EKF_X[0];
				Global_Show_Val[6] = EKF_X[1];
				Global_Show_Val[7] = EKF_X[2];		

				Global_Now_Euler[2] = atan2(-1 * EKF_X[1],  EKF_X[0]) * RAD_DEG;
				Global_Now_Euler[1] = atan2(-1 * mag_data[1], mag_data[0]) * RAD_DEG;
		}
}
