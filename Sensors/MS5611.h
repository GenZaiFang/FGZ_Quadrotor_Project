
#ifndef MS5611_H_
#define MS5611_H_

#include "System_Common.h"

static uint16_t g_C[8];//��̬���ݳ�Ա���������PROM�е�6������У׼����(ֻ��ҪC[1]-C[6],C[0]��C[7]����)

//-----------------------------------------------------------------
// ����MS5611_01BA01�ڲ���ַ��ָ��
//-----------------------------------------------------------------
//#define MS5611_01BA01_SlaveAddress    0xEE   //(111011Cx)�ӻ���ַ������ַ+1��CSB=GND : C=1��
#define MS5611_01BA01_SlaveAddress    0xEC   //(111011Cx)�ӻ���ַ������ַ+1  ��CSB=VCC : C=0��

#define MS5611_01BA01_PROM_CRC        0xAE   //ѭ��������λ��ַ

//MS5611_01BA01ָ���5�ࣩ
#define MS5611_01BA01_RST_CMD         0x1E   //��λ
#define MS5611_01BA01_D1_OSR_256_CMD  0x40   //600us//OSR: Over Sampling Ratio �������ʣ�����Ϊÿ��������ź�����ȡ�������ɢ�źŵĲ�����������HZΪ��λ��
#define MS5611_01BA01_D1_OSR_512_CMD  0x42   //1170us
#define MS5611_01BA01_D1_OSR_1024_CMD 0x44   //2280us
#define MS5611_01BA01_D1_OSR_2048_CMD 0x46   //4540us
#define MS5611_01BA01_D1_OSR_4096_CMD 0x48   //9040us  //����(D1)ѹ��ת��,��������Ϊ4096Hz
#define MS5611_01BA01_D2_OSR_256_CMD  0x50
#define MS5611_01BA01_D2_OSR_512_CMD  0x52
#define MS5611_01BA01_D2_OSR_1024_CMD 0x54
#define MS5611_01BA01_D2_OSR_2048_CMD 0x56
#define MS5611_01BA01_D2_OSR_4096_CMD 0x58    //����(D2)�¶�ת��,��������Ϊ4096Hz
#define MS5611_01BA01_ADC_RD_CMD      0x00    //��ADC������һ��24-bit AD�����
#define MS5611_01BA01_PROM_RD_CMD     0xA0    //��PROM������һ��16-bit ��ַ�����(1010 a2 a1 a0 0)��a2,a1,a0����8����ַ��(0xA0_0xAAΪ��ѹ�¶�У��6��ϵ����ÿ��ϵ��վ2-bit,��λ��ǰ)
//ת����ʱ(ע�⣺�����ȡ�����ݲ��ԣ��ͺ��п�����������ʱ̫������ģ�����)
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

