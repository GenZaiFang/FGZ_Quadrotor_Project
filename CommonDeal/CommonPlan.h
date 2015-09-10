#ifndef CommonPlan_H_
#define CommonPlan_H_

#include "System_Common.h"

void My_STM32_enable_interrupts(void);
void My_STM32_disable_interrupts(void);
void Get_AHRS_Sensor_Val(void);
void Get_Ref_Euler(const float accVal[3], const float magVal[3], Magnetometer_Calibration_Struct *SensorBpTmp, float *euler);
void HomeLocationPrepare(void);
void GPS_Dates_Deal(void);
void Rx_eCompassVal(void);
void Write_eCompassVal2Flash(uint32_t FlashAddress, Magnetometer_Calibration_Struct *SensorBpTmp);
void Read_Flash2eCompassVal(uint32_t FlashAddress, Magnetometer_Calibration_Struct *SensorBpTmp);
void eCompass_Estimate_Effect(Magnetometer_Calibration_Struct *SensorBpTmp);
void eCompassClibration(void);

#endif
