#include "EKF_INS_GPS_Estimator.h"
#include "CommonConversions.h"
#include "CommonPlan.h"
#include "My_USART.h"
#include "My_LED.h"

#define Xn 13
#define Qn 13
#define Zn 10
#define Rn 10

// X0 X1 X2  X3 X4 X5  X6 X7 X8 X9  X10 X11 X12  13维
// Pn Pe Pd  Vn Ve Vd  Q0 Q1 Q2 Q3  Wbx Wby Wbz 

// Z0 Z1 Z2  Z3 Z4 Z5  Z6 Z7 Z8 Z9  10维
// Pn Pe Pd  Vn Ve vd  Q0 Q1 Q2 Q3

//EKF 所需的变量
static float EKF_X[Xn];
static float EKF_P[Xn*Xn];
static float EKF_Z[Zn];
static float EKF_Q[Qn];
static float EKF_R[Rn];
static float EKF_K[Xn*Zn];
static float EKF_F[Xn*Xn]; 
static float EKF_H[Zn*Xn]; 

//龙格库塔所需的变量
static float K1[Xn], K2[Xn], K3[Xn], K4[Xn], Xlast[Xn];
static float dT2;

//预测协方差矩阵所需变量
static float fdt[Xn*Xn];
static float AT[Xn*Xn];
static float Ap[Xn*Xn];
static float A[Xn*Xn];
static float ApAT[Xn*Xn];
//static float ekf_p[Xn*Xn];

//计算姿态的K的变量
static float qh[4*7];
static float qp[7*7];
static float qk[7*4];
static float qhT[7*4];
static float qhp[4*7];
static float qhphT[4*4];
static float qinv_hphT[4*4];
static float qphT[7*4];



//计算空间K的变量
static float sh[6*6];
static float sp[6*6];
static float sk[6*6];
static float shT[6*6];
static float shp[6*6];
static float shphT[6*6];
static float sinv_hphT[6*6];
static float sphT[6*6];
static uint8_t GPS_Update = false;
	
//修正状态向量所需变量
static float hx[Zn];
static float error[Zn];
static float cx[Xn];

#ifndef Continuous_Correction
static float Qerror[4];
static float Qcx[7];
//计算姿态协方差的变量
static float qkh[7*7];
static float qkhp[7*7];
#endif

//修正协方差矩阵所需变量
static float kh[Xn*Xn];
static float khp[Xn*Xn];

//测量变量
static float u[6];

//IMU传感器变量
static float gyro[3], accel[3], mag_data[3];

//四元数的模长
static float qmag = 1;

//欧拉角及四元数
static float euler[3], q[4] = {1, 0, 0, 0};

//加速度 角速度 四元数 中间变量
static float ax, ay, az, wx, wy, wz, q0, q1, q2, q3;

//算法周期
static float dT = Deal_Period_ms / 1000.0f;

//初始化标志位
static uint8_t isInit = false;

static int i, j;

static void EKF_Clear(void)
{
		for(i = 0; i < Xn; i++)
		{
				EKF_X[i] = 0;
		}
		for(i = 0; i < Zn; i++)
		{
				EKF_Z[i] = 0;
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
	
		EKF_P[0*Xn+0] = EKF_P[1*Xn+1] = EKF_P[2*Xn+2] = 4.0f;    // initial position variance (m^2)
    EKF_P[3*Xn+3] = EKF_P[4*Xn+4] = EKF_P[5*Xn+5] = 2.0f;     // initial velocity variance (m/s)^2
	
    EKF_P[6*Xn+6] = EKF_P[7*Xn+7] = EKF_P[8*Xn+8] = EKF_P[9*Xn+9] = 1e-6f;  // initial quaternion variance
    EKF_P[10*Xn+10] = EKF_P[11*Xn+11] = EKF_P[12*Xn+12] = 1e-12f; // initial gyro bias variance (rad/s)^2
	
		EKF_X[0] = EKF_X[1] = EKF_X[2] = EKF_X[3] = EKF_X[4] = EKF_X[5] = 0.0f; // initial pos and vel (m)
    EKF_X[6] = 1.0f;
    EKF_X[7] = EKF_X[8] = EKF_X[9] = 0.0f;      // initial quaternion (level and North) (m/s)
    EKF_X[10] = EKF_X[11] = EKF_X[12] = 0.0f; // initial gyro bias (rad/s)
	
		EKF_Q[0] = EKF_Q[1] = EKF_Q[2] = 2.0f;    
    EKF_Q[3] = EKF_Q[4] = EKF_Q[5] = 2.0f;     
    EKF_Q[6] = EKF_Q[7] = EKF_Q[8] = EKF_Q[9] = 1e-6f;     
    EKF_Q[10] = EKF_Q[11] = EKF_Q[12] = 1e-6f; 
	
		EKF_R[0] = EKF_R[1] = 1000.0f;   	
    EKF_R[2] = 15.0f;          
    EKF_R[3] = EKF_R[4] = 15.0f;    
		EKF_R[5] = 1.0f;
    EKF_R[6] = EKF_R[7] = EKF_R[8] = EKF_R[9]  = 0.004f;     

		EKF_X[6] = Global_Init_Q[0];
		EKF_X[7] = Global_Init_Q[1];
		EKF_X[8] = Global_Init_Q[2];
		EKF_X[9] = Global_Init_Q[3];
}

static void EKF_Xdot_Cal(float x[Xn], float mu[6], float xdot[Xn])
{
		
    wx = mu[0] - EKF_X[10];
    wy = mu[1] - EKF_X[11];
    wz = mu[2] - EKF_X[12]; // subtract the biases on gyros	
		ax = -1 * mu[3];
    ay = -1 * mu[4];
    az = -1 * mu[5];
    q0 = EKF_X[6];
    q1 = EKF_X[7];
    q2 = EKF_X[8];
    q3 = EKF_X[9];
	
	  // Pdot = V
    xdot[0] = EKF_X[3];
    xdot[1] = EKF_X[4];
    xdot[2] = EKF_X[5];

    // Vdot = Reb*a
    xdot[3] =
        (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * ax + 2.0f * (q1 * q2 - q0 * q3) * ay 
				+ 2.0f * (q1 * q3 + q0 * q2) * az;
    xdot[4] =
        2.0f * (q1 * q2 + q0 * q3) * ax + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * ay 
				+ 2.0f * (q2 * q3 - q0 * q1) * az;
    xdot[5] =
        2.0f * (q1 * q3 - q0 * q2) * ax + 2.0f * (q2 * q3 + q0 * q1) * ay 
        + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az + GRAVITY_MSS;

    // qdot = Q*w
    xdot[6]  = (-q1 * wx - q2 * wy - q3 * wz) / 2.0f;
    xdot[7]  = (q0 * wx - q3 * wy + q2 * wz) / 2.0f;
    xdot[8]  = (q3 * wx + q0 * wy - q1 * wz) / 2.0f;
    xdot[9]  = (-q2 * wx + q1 * wy + q0 * wz) / 2.0f;

    // best guess is that bias stays constant
    xdot[10] = xdot[11] = xdot[12] = 0;
}

static void EKF_RungeKutta(float X[Xn], float mu[6], float dT)
{	
		dT2 = dT / 2.0f;
    for (i = 0; i < Xn; i++) 
		{
        Xlast[i] = X[i]; // make a working copy
    }
    EKF_Xdot_Cal(X, mu, K1); // k1 = f(x,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT2 * K1[i];
    }
    EKF_Xdot_Cal(X, mu, K2); // k2 = f(x+0.5*dT*k1,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT2 * K2[i];
    }
    EKF_Xdot_Cal(X, mu, K3); // k3 = f(x+0.5*dT*k2,u)
    for (i = 0; i < Xn; i++) 
		{
        X[i] = Xlast[i] + dT * K3[i];
    }
    EKF_Xdot_Cal(X, mu, K4); // k4 = f(x+dT*k3,u)

    // Xnew  = X + dT*(k1+2*k2+2*k3+k4)/6
    for (i = 0; i < Xn; i++) 
		{
				X[i] = Xlast[i] + dT * (K1[i] + 2.0f * K2[i] + 2.0f * K3[i] + K4[i]) / 6.0f;
    }
}

static void EKF_LinearizeF(float x[Xn], float mu[6], float f[Xn*Xn])
{
		wx = mu[0] - EKF_X[10];
    wy = mu[1] - EKF_X[11];
    wz = mu[2] - EKF_X[12];
	
    ax = mu[3];
    ay = mu[4];
    az = mu[5]; 
 
    q0 = EKF_X[6];
    q1 = EKF_X[7];
    q2 = EKF_X[8];
    q3 = EKF_X[9];
	
		// Pdot = V
    f[0*Xn+3]  = f[1*Xn+4] = f[2*Xn+5] = 1.0f;

    // dVdot/dq
    f[3*Xn+6]  = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    f[3*Xn+7]  = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    f[3*Xn+8]  = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    f[3*Xn+9]  = 2.0f * (-q3 * ax - q0 * ay + q1 * az);
    f[4*Xn+6]  = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    f[4*Xn+7]  = 2.0f * (q2 * ax - q1 * ay - q0 * az);
    f[4*Xn+8]  = 2.0f * (q1 * ax + q2 * ay + q3 * az);
    f[4*Xn+9]  = 2.0f * (q0 * ax - q3 * ay + q2 * az);
    f[5*Xn+6]  = 2.0f * (-q2 * ax + q1 * ay + q0 * az);
    f[5*Xn+7]  = 2.0f * (q3 * ax + q0 * ay - q1 * az);
    f[5*Xn+8]  = 2.0f * (-q0 * ax + q3 * ay - q2 * az);
    f[5*Xn+9]  = 2.0f * (q1 * ax + q2 * ay + q3 * az);

    // dqdot/dq
    f[6*Xn+6]  = 0;
    f[6*Xn+7]  = -wx / 2.0f;
    f[6*Xn+8]  = -wy / 2.0f;
    f[6*Xn+9]  = -wz / 2.0f;
    f[7*Xn+6]  = wx / 2.0f;
    f[7*Xn+7]  = 0;
    f[7*Xn+8]  = wz / 2.0f;
    f[7*Xn+9]  = -wy / 2.0f;
    f[8*Xn+6]  = wy / 2.0f;
    f[8*Xn+7]  = -wz / 2.0f;
    f[8*Xn+8]  = 0;
    f[8*Xn+9]  = wx / 2.0f;
    f[9*Xn+6]  = wz / 2.0f;
    f[9*Xn+7]  = wy / 2.0f;
    f[9*Xn+8]  = -wx / 2.0f;
    f[9*Xn+9]  = 0;

    // dqdot/dwbias
    f[6*Xn+10] = q1 / 2.0f;
    f[6*Xn+11] = q2 / 2.0f;
    f[6*Xn+12] = q3 / 2.0f;
    f[7*Xn+10] = -q0 / 2.0f;
    f[7*Xn+11] = q3 / 2.0f;
    f[7*Xn+12] = -q2 / 2.0f;
    f[8*Xn+10] = -q3 / 2.0f;
    f[8*Xn+11] = -q0 / 2.0f;
    f[8*Xn+12] = q1 / 2.0f;
    f[9*Xn+10] = q2 / 2.0f;
    f[9*Xn+11] = -q1 / 2.0f;
    f[9*Xn+12] = -q0 / 2.0f;
}

static void EKF_StatePrediction(float gyro_data[3], float accel_data[3], float dT)
{
		u[0] = gyro_data[0];
    u[1] = gyro_data[1];
    u[2] = gyro_data[2];
	
		u[3] = accel_data[0];
		u[4] = accel_data[1];
		u[5] = accel_data[2];
	
		EKF_LinearizeF(EKF_X, u, EKF_F);
		EKF_RungeKutta(EKF_X, u, dT);

#if 1 // 防止偏航角在+/-180°是发生突变
		q[0] = EKF_X[6];
		q[1] = EKF_X[7];
		q[2] = EKF_X[8];
		q[3] = EKF_X[9];
		Quaternion2Euler(q, euler);	
		if(euler[2] > 180)
		{
				euler[2] = -180;
		}
		else if(euler[2] < -180)
		{
				euler[2] = 180;
		}
		Euler2Quaternion(euler, q);
		EKF_X[6] = q[0];
		EKF_X[7] = q[1];
		EKF_X[8] = q[2];
		EKF_X[9] = q[3];
#endif	
#if 1		
				Nav.Vel[0] = EKF_X[3];
				Nav.Vel[1] = EKF_X[4];
				Nav.Vel[2] = EKF_X[5];
#endif		
}

static void EKF_CovariancePrediction(float f[Xn*Xn], float Q[Qn], float dT, float p[Xn*Xn])
{		
		for(i = 0; i < Xn * Xn; i++)
		{				
				A[i] = fdt[i] = f[i] * dT;							
		}
		
		for(i = 0; i < Xn; i++)
		{
				A[i*Xn+i] = fdt[i*Xn+i] + 1.0f;
		}
		
		ML_R_X_ML_R(A, p, Ap, Xn, Xn, Xn, Xn);
		Matrix_Tran(A, AT, Xn, Xn);				
		ML_R_X_ML_R(Ap, AT, ApAT, Xn, Xn, Xn, Xn);

		for(i = 0; i < Xn * Xn; i++)
		{
				p[i] = ApAT[i];
		}
		
		for(i = 0; i < Xn; i++)
		{
				p[i*Xn+i] = p[i*Xn+i] + Q[i];
		}
}

static void EKF_INSCovariancePrediction(float dT)
{
    EKF_CovariancePrediction(EKF_F, EKF_Q, dT, EKF_P);
}

static void EKF_LinearizeH(float h[Zn*Xn])
{
		for(i = 0; i < Zn; i++)
		{
				h[i*Xn+i] = 1.0f;
		}
}

static void EKF_Z_Cal(float accel[3], float mag_data[3], float z[Zn])
{	
#if defined USE_MAG	
	
		#if defined USE_MAG_HMC5883L
			euler[2] = atan2(mag_data[1], mag_data[0]) * RAD_DEG;		
		#elif defined	USE_MAG_LSM303D
			Get_Ref_Euler(accel, mag_data, &Mag_LSM303D_Calibration, euler);
		#endif
			
		Global_Show_Val[5] = euler[0];
		Global_Show_Val[6] = euler[1];
		Global_Show_Val[7] = euler[2];
		//euler[2] -= Global_Heading_Ref;
		
#else		
		u[0] = accel[0];
    u[1] = accel[1];
    u[2] = accel[2];
	
		q[0] = EKF_X[6];
		q[1] = EKF_X[7];
		q[2] = EKF_X[8];
		q[3] = EKF_X[9];
		Quaternion2Euler(q, euler);//计算偏航角euler[2]
		if(euler[2] > 180)
		{
				euler[2] -= 360;
		}
		else if(euler[2] < -180)
		{
				euler[2] += 360;
		}
		euler[0] = atan2(u[1], u[2]) * RAD_DEG;		
		euler[1] = atan2(-1 * u[0], u[1] * sin(euler[0]) + u[2] * cos(euler[0])) * RAD_DEG;
#endif	

		Euler2Quaternion(euler, q);		
				
		qmag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); 
		
		q[0] /= qmag;
		q[1] /= qmag;
		q[2] /= qmag;
		q[3] /= qmag;	

		z[6] = q[0];
		z[7] = q[1];
		z[8] = q[2];
		z[9] = q[3];
		
#if 1
		if(Global_GPS_Sensor.updated)
		{
				z[0] = Global_GPS_Sensor.NED_Pos[0];
				z[1] = Global_GPS_Sensor.NED_Pos[1];
				
#if 0
				z[2] = Global_GPS_Sensor.NED_Pos[2];			
#else			
				z[2] = -1 * Global_Altitude_Sensor.altitude;
#endif			
			
				z[3] = Global_GPS_Sensor.NED_Vel[0];
				z[4] = Global_GPS_Sensor.NED_Vel[1];
				z[5] = Global_GPS_Sensor.NED_Vel[2];
			
			Global_GPS_Sensor.updated = false;
			GPS_Update = true;
		}
#endif
}

static void EKF_K_Cal(float p[Xn*Xn], float h[Zn*Xn], float r[Zn], float k[Xn*Zn])
{
/*******************************姿态K的计算开始*************************************/	
		for(i = 0; i < 4; i++)
		{
				for(j = 0; j < 7; j++)
				{
						qh[i*7+j] = h[(i+6)*Xn+(j+6)];
				}
		}
		
		for(i = 0; i < 7; i++)
		{
				for(j = 0; j < 7; j++)
				{
						qp[i*7+j] = p[(i+6)*Xn+(j+6)];
				}
		}
			
		Matrix_Tran(qh, qhT, 4, 7);

		ML_R_X_ML_R(qh, qp, qhp, 4, 7, 7, 7);
		ML_R_X_ML_R(qhp, qhT, qhphT, 4, 7, 7, 4);

		qhphT[0*4+0] += r[6];
		qhphT[1*4+1] += r[7];
		qhphT[2*4+2] += r[8];
		qhphT[3*4+3] += r[9];
		
		Matrix_4X4_Inv(qhphT, qinv_hphT);

		ML_R_X_ML_R(qp, qhT, qphT, 7, 7, 7, 4);
		ML_R_X_ML_R(qphT, qinv_hphT, qk, 7, 4, 4, 4);
		
		for(i = 0; i < 7; i++)
		{
				for(j = 0; j < 4; j++)
				{
						k[(i+6)*Zn+(j+6)] = qk[i*4+j];
				}
		}	
/*******************************姿态K的计算结束*************************************/
		
		
		
/*******************************空间K的计算开始*************************************/			
		if(GPS_Update)
		{
				for(i = 0; i < 6; i++)
				{
						for(j = 0; j < 6; j++)
						{
								sh[i*6+j] = h[i*Xn+j];
								sp[i*6+j] = p[i*Xn+j];
						}
				}
				
				Matrix_Tran(sh, shT, 6, 6);
				ML_R_X_ML_R(sh, sp, shp, 6, 6, 6, 6);
				ML_R_X_ML_R(shp, shT, shphT, 6, 6, 6, 6);
				
				shphT[0*6+0] += r[0];
				shphT[1*6+1] += r[1];
				shphT[2*6+2] += r[2];
				shphT[3*6+3] += r[3];
				shphT[4*6+4] += r[4];
				shphT[5*6+5] += r[5];
				
				Matrix_6X6_Inv(shphT, sinv_hphT);
				
				ML_R_X_ML_R(sp, shT, sphT, 6, 6, 6, 6);
				ML_R_X_ML_R(sphT, sinv_hphT, sk, 6, 6, 6, 6);
				
				for(i = 0; i < 6; i++)
				{
						for(j = 0; j < 6; j++)
						{
								k[i*Zn+j] = sk[i*6+j];
						}
				}
#if defined Continuous_Correction	
				GPS_Update = false;
#endif				
				LED4_BLINK;
		}
		
/*******************************空间K的计算结束*************************************/
}

static void EKF_Correction_X(float z[Zn], float h[Zn*Xn], float k[Xn*Zn], float x[Xn])
{	
		ML_R_X_ML_R(h, x, hx, Zn, Xn, Xn, 1);
	
		for(i = 0; i < Zn; i++)
		{
				error[i] = z[i] - hx[i];
		}
#if defined Continuous_Correction
		
		ML_R_X_ML_R(k, error, cx, Xn, Zn, Zn, 1);
			
		for(i = 0; i < Xn; i++)
		{
				x[i] += cx[i];
		}	
		
#else		
		if(GPS_Update)
		{
				ML_R_X_ML_R(k, error, cx, Xn, Zn, Zn, 1);
			
				for(i = 0; i < Xn; i++)
				{
						x[i] += cx[i];
				}					
		}
		else
		{
				Qerror[0] = error[6];
				Qerror[1] = error[7];
				Qerror[2] = error[8];
				Qerror[3] = error[9];

				ML_R_X_ML_R(qk, Qerror, Qcx, 7, 4, 4, 1);
			
				x[6] += Qcx[0];
				x[7] += Qcx[1];
				x[8] += Qcx[2];
				x[9] += Qcx[3];
				x[10] += Qcx[4];
				x[11] += Qcx[5];
				x[12] += Qcx[6];
		}
#endif
		qmag = sqrt(x[6] * x[6] + x[7] * x[7] + x[8] * x[8] + x[9] * x[9]);
		x[6] /= qmag;
		x[7] /= qmag;
		x[8] /= qmag;
		x[9] /= qmag;
}

static void EKF_Correction_P(float k[Xn*Zn], float h[Zn*Xn], float P[Xn*Xn])
{
#if defined Continuous_Correction
	
		ML_R_X_ML_R(k, h, kh, Xn, Zn, Zn, Xn);
		ML_R_X_ML_R(kh, P, khp, Xn, Xn, Xn, Xn);

		for(i = 0; i < Xn * Xn; i++)		
		{
				P[i] -= khp[i];
		}	
		
#else	
		if(GPS_Update)
		{
				ML_R_X_ML_R(k, h, kh, Xn, Zn, Zn, Xn);
				ML_R_X_ML_R(kh, P, khp, Xn, Xn, Xn, Xn);

				for(i = 0; i < Xn * Xn; i++)		
				{
						P[i] -= khp[i];
				}	
				GPS_Update = false;
		}
		else
		{				
				for(i = 0; i < 7; i++)
				{
						for(j = 0; j < 7; j++)
						{
								qp[i*7+j] = P[(i+6)*Xn+(j+6)];
						}
				}
				
				ML_R_X_ML_R(qk, qh, qkh, 7, 4, 4, 7);
				ML_R_X_ML_R(qkh, qp, qkhp, 7, 7, 7, 7);

				for(i = 0; i < 7; i++)
				{
						for(j = 0; j < 7; j++)
						{
								P[(i+6)*Xn+(j+6)] -= qkhp[i*7+j];
						}
				}
		}
#endif
}

void EKF_INS_GPS_Run(void)
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
			
				accel[0] = Global_Accel_Sensor.filtered.x;
				accel[1] = Global_Accel_Sensor.filtered.y;
				accel[2] = Global_Accel_Sensor.filtered.z;
			
				mag_data[0] = (float)Global_Mag_Val[0];
				mag_data[1] = (float)Global_Mag_Val[1];
				mag_data[2] = (float)Global_Mag_Val[2];
			
				EKF_StatePrediction(gyro, accel, dT);
			
				EKF_INSCovariancePrediction(dT);

				EKF_Z_Cal(accel, mag_data, EKF_Z);
			
				EKF_LinearizeH(EKF_H);
#if 1
				EKF_K_Cal(EKF_P, EKF_H, EKF_R, EKF_K);	
				
				EKF_Correction_X(EKF_Z, EKF_H, EKF_K, EKF_X);
				
				EKF_Correction_P(EKF_K, EKF_H, EKF_P);
#endif

#if 1
				Nav.Pos[0] = EKF_X[0];
				Nav.Pos[1] = EKF_X[1];
				Nav.Pos[2] = EKF_X[2];
#endif

#if 0		
				Nav.Vel[0] = EKF_X[3];
				Nav.Vel[1] = EKF_X[4];
				Nav.Vel[2] = EKF_X[5];
#endif				
				Nav.q[0] = EKF_X[6];
				Nav.q[1] = EKF_X[7];
				Nav.q[2] = EKF_X[8];
				Nav.q[3] = EKF_X[9];
				
				Nav.gyro_bias[0] = EKF_X[10];
				Nav.gyro_bias[1] = EKF_X[11];
				Nav.gyro_bias[2] = EKF_X[12];				
			
				Quaternion2Euler(Nav.q, euler);
				
#if defined USE_X_MODE			
				Global_Now_Euler[0] = euler[0] * 0.707f +  euler[1] * 0.707f;
				Global_Now_Euler[1] = euler[1] * 0.707f -  euler[0] * 0.707f;
				Global_Now_Euler[2] = euler[2];
#else
				Global_Now_Euler[0] = euler[0];
				Global_Now_Euler[1] = euler[1];
				Global_Now_Euler[2] = euler[2];
#endif				
		}
}





