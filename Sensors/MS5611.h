
#ifndef MS5611_H_
#define MS5611_H_

#include "System_Common.h"

static uint16_t g_C[8];//静态数据成员声明，存放PROM中的6个出厂校准数据(只需要C[1]-C[6],C[0]和C[7]无用)

//-----------------------------------------------------------------
// 定义MS5611_01BA01内部地址及指令
//-----------------------------------------------------------------
//#define MS5611_01BA01_SlaveAddress    0xEE   //(111011Cx)从机地址，读地址+1（CSB=GND : C=1）
#define MS5611_01BA01_SlaveAddress    0xEC   //(111011Cx)从机地址，读地址+1  （CSB=VCC : C=0）

#define MS5611_01BA01_PROM_CRC        0xAE   //循环冗余检查位地址

//MS5611_01BA01指令集（5类）
#define MS5611_01BA01_RST_CMD         0x1E   //复位
#define MS5611_01BA01_D1_OSR_256_CMD  0x40   //600us//OSR: Over Sampling Ratio 过采样率，定义为每秒从连续信号中提取并组成离散信号的采样个数，以HZ为单位。
#define MS5611_01BA01_D1_OSR_512_CMD  0x42   //1170us
#define MS5611_01BA01_D1_OSR_1024_CMD 0x44   //2280us
#define MS5611_01BA01_D1_OSR_2048_CMD 0x46   //4540us
#define MS5611_01BA01_D1_OSR_4096_CMD 0x48   //9040us  //启动(D1)压力转换,过采样率为4096Hz
#define MS5611_01BA01_D2_OSR_256_CMD  0x50
#define MS5611_01BA01_D2_OSR_512_CMD  0x52
#define MS5611_01BA01_D2_OSR_1024_CMD 0x54
#define MS5611_01BA01_D2_OSR_2048_CMD 0x56
#define MS5611_01BA01_D2_OSR_4096_CMD 0x58    //启动(D2)温度转换,过采样率为4096Hz
#define MS5611_01BA01_ADC_RD_CMD      0x00    //读ADC（返回一个24-bit AD结果）
#define MS5611_01BA01_PROM_RD_CMD     0xA0    //读PROM（返回一个16-bit 地址结果）(1010 a2 a1 a0 0)由a2,a1,a0产生8个地址！(0xA0_0xAA为气压温度校验6个系数，每个系数站2-bit,高位在前)
//转换延时(注意：如果读取的数据不对，就很有可能是下面延时太短引起的！！！)
#define DELAY_256  700
#define DELAY_512  1270
#define DELAY_1024 2380
#define DELAY_2048 4640
#define DELAY_4096 9140

float low_pass_filter(float new_val);
int32_t slide_arithmetic_mean_filter(int32_t dat, uint8_t n);
void ms5611_reset(void);
void ms5611_prom_read(uint16_t *s);
uint32_t ms5611_do_conversion(uint8_t cmd_type, uint16_t cmd_delay);
void My_MS5611_init(void);
float ms5611_get_temperature(void);
float ms5611_get_pressure(void);
float MS5611_Get_Ref_Altitude(void);
float MS5611_Get_Altitude(float AltRef);

#endif

