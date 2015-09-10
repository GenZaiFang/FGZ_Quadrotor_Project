
#ifndef MPU6050_H_
#define MPU6050_H_

#include "System_Common.h"

//-----------------------------------------------------------------
// 定义MPU6050内部寄存器地址
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//寄存器名：  Sample Rate Divider 采样速率分频器(典型值：0x07(125Hz))
//寄存器内容：SMPLRT_DIV[7:0]
//寄存器介绍：Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//            采样速率由陀螺仪输出速率经SMPLRT_DIV分频得到
//            Gyroscope Output Rate = 8KHz/1KHz  (depend on the value of DLPF_CFG below)
//            但是accelerometer output rate = 1KHz 不变
//-----------------------------------------------------------------
#define	SMPLRT_DIV		0x19

//-----------------------------------------------------------------
//寄存器名：Configuration 配置寄存器 (典型值：0x06(5Hz))
//寄存器内容：EXT_SYNC_SET[5:3];DLPF_CFG[2:0](数字低通滤波器配置寄存器)
//寄存器介绍：DLPF_CFG	           Accelerometer                           Gyroscope

//                         Bandwidth (Hz) Delay(ms) Fs(1kHz)    Bandwidth(Hz) Delay(ms) Fs(kHz)
//                0	          260        	0	       1           256      	0.98 	  8
//                1	          184	        2.0	       1           188	        1.9  	  1
//                2            94        	3.0        1            98	        2.8  	  1
//                3	           44         	4.9	       1            42	        4.8	      1
//                4            21        	8.5	       1            20	        8.3	      1
//                5            10          13.8	       1            10	       13.4	      1
//                6             5          19.0	       1             5	       18.6	      1
//                7	             RESERVED               	            RESERVED	      8
//             用于配置ACC和GYRO的低通滤波频率
//-----------------------------------------------------------------
#define	CONFIG			0x1A

//-----------------------------------------------------------------
//寄存器名：  Gyroscope Configuration 陀螺仪配置寄存器(陀螺仪自检、测量范围) (典型值：0x18(不自检，2000deg/s))
//寄存器内容：(XG_ST,YG_ST,ZG_ST)[7:5]; FS_SEL[4:3]
//寄存器介绍：(XG_ST,YG_ST,ZG_ST)用于启动3轴陀螺仪自检，FS_SEL用于设置陀螺仪满量程范围(输出为16位有符号数-32767~32767)
//     FS_SEL	Full Scale Range

//        0     	± 250  °/s
//        1     	± 500  °/s
//        2     	± 1000 °/s
//        3	        ± 2000 °/s
//-----------------------------------------------------------------
#define	GYRO_CONFIG		0x1B

//-----------------------------------------------------------------
//寄存器名：  Accelerometer Configuration 加速度计配置寄存器(加速计自检、测量范围及高通滤波频率)
//寄存器内容：(XA_ST,YA_ST,ZA_ST)[7:5];  AFS_SEL[4:3]; ACCEL_HPF[2:0]
//寄存器介绍：(XA_ST,YA_ST,ZA_ST)用于启动3轴加速度计自检，AFS_SEL用于设置加速度计满量程范围，
//            ACCEL_HPF用于设置ACC的高通滤波频率(典型值：0x01(不自检，2G，5Hz))

//            AFS_SEL	Full Scale Range
//                0         	±  2g
//                1           	±  4g
//                2           	±  8g
//                3	            ± 16g

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
//寄存器名：   FIFO Enable  FIFO使能 (典型值：0x00 (不使能FIFO))
//寄存器内容： (TEMP_FIFO_EN, XG_FIFO_EN, YG_FIFO_EN, ZG_FIFO_EN, ACCEL_FIFO_EN, SLV2_FIFO_EN, SLV1_FIFO_EN, SLV0_FIFO_EN)[7:0]
//寄存器介绍： Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer
//             if a sensor's respective FIFO_EN bit is set to 1 in this register.
//-----------------------------------------------------------------
#define	FIFO_EN         0x23

//-----------------------------------------------------------------
//寄存器名：   INT PIN / BYPASS ENABLE CONFIGURATION
//旁路控制
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
//寄存器名：  SIGNAL_PATH_RESET 信号通道复位 (典型值：0x00,不复位)
//寄存器内容：(GYRO_RESET,ACCEL_RESET,TEMP_RESET)[2:0]
//寄存器介绍：to reset the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors.
//            Note：This register does not clear the sensor registers.
//-----------------------------------------------------------------
#define SIGNAL_PATH_RESET 0x68

//-----------------------------------------------------------------
//寄存器名：  User Control 用户控制 (典型值：0x00(不使能FIFO，I2C_MST；不复位FIFO,I2C_MST,SIG_COND))
//寄存器内容：(FIFO_EN,I2C_MST_EN,I2C_IF_DIS)[6:4];(FIFO_RESET,I2C_MST_RESET,SIG_COND_RESET)[2:0]
//寄存器介绍：FIFO_EN =1: 使能FIFO buffer
//            I2C_MST_EN = 1: 使能I2C Master Mode
//            I2C_IF_DIS = 0; 使能primary I2C interface(6050中，此位为0)
//            FIFO_RESET,I2C_MST_RESET,SIG_COND_RESET: FIFO buffer, I2C Master, sensor signal paths and sensor registers reset
//-----------------------------------------------------------------
#define USER_CTRL       0x6A

//-----------------------------------------------------------------
//寄存器名：  Power Management 1 电源管理1 (典型值：0x00(正常启用))
//寄存器内容：(DEVICE_RESET,SLEEP,CYCLE,_,TEMP_DIS)[7:3];CLKSEL[2:0]
//寄存器介绍：DEVICE_RESET = 1:设备复位
//            SLEEP = 1:进入睡眠模式(low power sleep mode)  (默认SLEEP = 1,PWR_MGMT_1 = 0x40(睡眠模式))
//            CYCLE = 1(SLEEP = 0):进入Cycle Mode
//            CLKSEL:时钟源选择

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
//寄存器名：  Power Management 2 电源管理2 (典型值：0x00(正常启用))
//寄存器内容：LP_WAKE_CTRL[7:6]；(STBY_XA  STBY_YA  STBY_ZA	STBY_XG	STBY_YG	STBY_ZG)[5:0]
//寄存器介绍：LP_WAKE_CTRL: configure the frequency of wake-ups in Accelerometer Only Low Power Mode
//            LP_WAKE_CTRL  	Wake-up Frequency
//                0	                 1.25 Hz
//                1	                 2.5 Hz
//                2	                 5 Hz
//                3	                10 Hz
//            STBY_xx:put individual axes of the accelerometer and gyroscope into standby mode(待机模式).
//                    If the device is using a gyroscope axis as the clock source and this axis is put into standby mode, the clock source will automatically be changed to the internal 8MHz oscillator.
//-----------------------------------------------------------------
#define	PWR_MGMT_2		0x6C


//-----------------------------------------------------------------
//寄存器名：  Who Am I  Device Address(默认数值0x68，只读)
//寄存器内容：WHO_AM_I[6:1]
//寄存器介绍：7位Device Address的高6位，最低位的值由AD0端口的值决定(Hard coded to 0)所以默认值0x68(110 100 0)
//-----------------------------------------------------------------
#define	MPU6050_WHO_AM_I		0x75

//-----------------------------------------------------------------
//从机地址：Device Address左移一位，右边补0(写)/1(读) ====> SlaveAddress
//          所以 SlaveAddress = 0x68<<1+0 = 0xD0(写) / 0x68<<1+1 = 0xD1(读)
//-----------------------------------------------------------------
#define	MPU6050_SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取

#define MPU6050_GYRO_DEG       16.4f

void My_MPU6050_init(void);
void My_MPU6050_Get_Accle_Val(int16_t *val);
void My_MPU6050_Get_Gyro_Val(int16_t *val);
uint8_t Revise_MPU6050_GyroVal(void);

#endif

