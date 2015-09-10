
#ifndef IMU_H_
#define IMU_H_

#include "System_Common.h"

#define DCM_Kp         2.0f
#define DCM_Ki         0.005f



#define MAX_Clear_Range 0.002f
#define MIN_Clear_Range -0.002f



//void QCF_Attitude(void);

void DCM_Attitude(double gx, double gy, double gz,
									double ax, double ay, double az,
									double mx, double mz, double my,
									float *imu_yaw, float *imu_pitch, float *imu_roll);

#endif /* IMU_H_ */
