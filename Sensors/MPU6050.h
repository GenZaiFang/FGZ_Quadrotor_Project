
#ifndef MPU6050_H_
#define MPU6050_H_

#include "System_Common.h"

//-----------------------------------------------------------------
// ����MPU6050�ڲ��Ĵ�����ַ
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//�Ĵ�������  Sample Rate Divider �������ʷ�Ƶ��(����ֵ��0x07(125Hz))
//�Ĵ������ݣ�SMPLRT_DIV[7:0]
//�Ĵ������ܣ�Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//            ����������������������ʾ�SMPLRT_DIV��Ƶ�õ�
//            Gyroscope Output Rate = 8KHz/1KHz  (depend on the value of DLPF_CFG below)
//            ����accelerometer output rate = 1KHz ����
//-----------------------------------------------------------------
#define	SMPLRT_DIV		0x19

//-----------------------------------------------------------------
//�Ĵ�������Configuration ���üĴ��� (����ֵ��0x06(5Hz))
//�Ĵ������ݣ�EXT_SYNC_SET[5:3];DLPF_CFG[2:0](���ֵ�ͨ�˲������üĴ���)
//�Ĵ������ܣ�DLPF_CFG	           Accelerometer                           Gyroscope

//                         Bandwidth (Hz) Delay(ms) Fs(1kHz)    Bandwidth(Hz) Delay(ms) Fs(kHz)
//                0	          260        	0	       1           256      	0.98 	  8
//                1	          184	        2.0	       1           188	        1.9  	  1
//                2            94        	3.0        1            98	        2.8  	  1
//                3	           44         	4.9	       1            42	        4.8	      1
//                4            21        	8.5	       1            20	        8.3	      1
//                5            10          13.8	       1            10	       13.4	      1
//                6             5          19.0	       1             5	       18.6	      1
//                7	             RESERVED               	            RESERVED	      8
//             ��������ACC��GYRO�ĵ�ͨ�˲�Ƶ��
//-----------------------------------------------------------------
#define	CONFIG			0x1A

//-----------------------------------------------------------------
//�Ĵ�������  Gyroscope Configuration ���������üĴ���(�������Լ졢������Χ) (����ֵ��0x18(���Լ죬2000deg/s))
//�Ĵ������ݣ�(XG_ST,YG_ST,ZG_ST)[7:5]; FS_SEL[4:3]
//�Ĵ������ܣ�(XG_ST,YG_ST,ZG_ST)��������3���������Լ죬FS_SEL�������������������̷�Χ(���Ϊ16λ�з�����-32767~32767)
//     FS_SEL	Full Scale Range

//        0     	�� 250  ��/s
//        1     	�� 500  ��/s
//        2     	�� 1000 ��/s
//        3	        �� 2000 ��/s
//-----------------------------------------------------------------
#define	GYRO_CONFIG		0x1B

//-----------------------------------------------------------------
//�Ĵ�������  Accelerometer Configuration ���ٶȼ����üĴ���(���ټ��Լ졢������Χ����ͨ�˲�Ƶ��)
//�Ĵ������ݣ�(XA_ST,YA_ST,ZA_ST)[7:5];  AFS_SEL[4:3]; ACCEL_HPF[2:0]
//�Ĵ������ܣ�(XA_ST,YA_ST,ZA_ST)��������3����ٶȼ��Լ죬AFS_SEL�������ü��ٶȼ������̷�Χ��
//            ACCEL_HPF��������ACC�ĸ�ͨ�˲�Ƶ��(����ֵ��0x01(���Լ죬2G��5Hz))

//            AFS_SEL	Full Scale Range
//                0         	��  2g
//                1           	��  4g
//                2           	��  8g
//                3	            �� 16g

//            ACCEL_HPF	   Filter Mode   	Cut-off Frequency
//                0            Reset             	None
//                1	             On	                5Hz
//                2              On                 2.5Hz
//                3              On                 1.25Hz
//                4              On                 0.63Hz
//                7             Hold                None
//-----------------------------------------------------------------
#define	ACCEL_CONFIG	0x1C

//-----------------------------------------------------------------
//�Ĵ�������   FIFO Enable  FIFOʹ�� (����ֵ��0x00 (��ʹ��FIFO))
//�Ĵ������ݣ� (TEMP_FIFO_EN, XG_FIFO_EN, YG_FIFO_EN, ZG_FIFO_EN, ACCEL_FIFO_EN, SLV2_FIFO_EN, SLV1_FIFO_EN, SLV0_FIFO_EN)[7:0]
//�Ĵ������ܣ� Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer
//             if a sensor's respective FIFO_EN bit is set to 1 in this register.
//-----------------------------------------------------------------
#define	FIFO_EN         0x23

//-----------------------------------------------------------------
//�Ĵ�������   INT PIN / BYPASS ENABLE CONFIGURATION
//��·����
//-----------------------------------------------------------------
#define INT_PIN_CFG     0x37

//-----------------------------------------------------------------
//ACCELEROMETER MEASUREMENTS
//-----------------------------------------------------------------
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

//-----------------------------------------------------------------
//TEMPERATURE MEASUREMENT
//-----------------------------------------------------------------
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

//-----------------------------------------------------------------
//GYROSCOPE MEASUREMENTS
//-----------------------------------------------------------------
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

//-----------------------------------------------------------------
//�Ĵ�������  SIGNAL_PATH_RESET �ź�ͨ����λ (����ֵ��0x00,����λ)
//�Ĵ������ݣ�(GYRO_RESET,ACCEL_RESET,TEMP_RESET)[2:0]
//�Ĵ������ܣ�to reset the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors.
//            Note��This register does not clear the sensor registers.
//-----------------------------------------------------------------
#define SIGNAL_PATH_RESET 0x68

//-----------------------------------------------------------------
//�Ĵ�������  User Control �û����� (����ֵ��0x00(��ʹ��FIFO��I2C_MST������λFIFO,I2C_MST,SIG_COND))
//�Ĵ������ݣ�(FIFO_EN,I2C_MST_EN,I2C_IF_DIS)[6:4];(FIFO_RESET,I2C_MST_RESET,SIG_COND_RESET)[2:0]
//�Ĵ������ܣ�FIFO_EN =1: ʹ��FIFO buffer
//            I2C_MST_EN = 1: ʹ��I2C Master Mode
//            I2C_IF_DIS = 0; ʹ��primary I2C interface(6050�У���λΪ0)
//            FIFO_RESET,I2C_MST_RESET,SIG_COND_RESET: FIFO buffer, I2C Master, sensor signal paths and sensor registers reset
//-----------------------------------------------------------------
#define USER_CTRL       0x6A

//-----------------------------------------------------------------
//�Ĵ�������  Power Management 1 ��Դ����1 (����ֵ��0x00(��������))
//�Ĵ������ݣ�(DEVICE_RESET,SLEEP,CYCLE,_,TEMP_DIS)[7:3];CLKSEL[2:0]
//�Ĵ������ܣ�DEVICE_RESET = 1:�豸��λ
//            SLEEP = 1:����˯��ģʽ(low power sleep mode)  (Ĭ��SLEEP = 1,PWR_MGMT_1 = 0x40(˯��ģʽ))
//            CYCLE = 1(SLEEP = 0):����Cycle Mode
//            CLKSEL:ʱ��Դѡ��

//            CLKSEL    	Clock Source
//              0      Internal 8MHz oscillator
//              1      PLL with X axis gyroscope reference
//              2      PLL with Y axis gyroscope reference
//              3      PLL with Z axis gyroscope reference
//              4      PLL with external 32.768kHz reference
//              5      PLL with external 19.2MHz reference
//              6      Reserved
//              7      Stops the clock and keeps the timing generator in reset
//-----------------------------------------------------------------
#define	PWR_MGMT_1		0x6B

//-----------------------------------------------------------------
//�Ĵ�������  Power Management 2 ��Դ����2 (����ֵ��0x00(��������))
//�Ĵ������ݣ�LP_WAKE_CTRL[7:6]��(STBY_XA  STBY_YA  STBY_ZA	STBY_XG	STBY_YG	STBY_ZG)[5:0]
//�Ĵ������ܣ�LP_WAKE_CTRL: configure the frequency of wake-ups in Accelerometer Only Low Power Mode
//            LP_WAKE_CTRL  	Wake-up Frequency
//                0	                 1.25 Hz
//                1	                 2.5 Hz
//                2	                 5 Hz
//                3	                10 Hz
//            STBY_xx:put individual axes of the accelerometer and gyroscope into standby mode(����ģʽ).
//                    If the device is using a gyroscope axis as the clock source and this axis is put into standby mode, the clock source will automatically be changed to the internal 8MHz oscillator.
//-----------------------------------------------------------------
#define	PWR_MGMT_2		0x6C


//-----------------------------------------------------------------
//�Ĵ�������  Who Am I  Device Address(Ĭ����ֵ0x68��ֻ��)
//�Ĵ������ݣ�WHO_AM_I[6:1]
//�Ĵ������ܣ�7λDevice Address�ĸ�6λ�����λ��ֵ��AD0�˿ڵ�ֵ����(Hard coded to 0)����Ĭ��ֵ0x68(110 100 0)
//-----------------------------------------------------------------
#define	MPU6050_WHO_AM_I		0x75

//-----------------------------------------------------------------
//�ӻ���ַ��Device Address����һλ���ұ߲�0(д)/1(��) ====> SlaveAddress
//          ���� SlaveAddress = 0x68<<1+0 = 0xD0(д) / 0x68<<1+1 = 0xD1(��)
//-----------------------------------------------------------------
#define	MPU6050_SlaveAddress	0xD0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

#define MPU6050_GYRO_DEG       16.4f

void My_MPU6050_init(void);
void My_MPU6050_Get_Accle_Val(int16_t *val);
void My_MPU6050_Get_Gyro_Val(int16_t *val);
uint8_t Revise_MPU6050_GyroVal(void);

#endif

