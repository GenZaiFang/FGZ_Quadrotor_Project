#include "MS5611.h"
#include "My_I2C.h"
#include "delay.h"

static uint16_t g_C[8]={0, 0, 0, 0, 0, 0, 0, 0}; //静态数据成员初始化(不能缺！)

//-----------------------------------------------------------------
//函数名称：low_pass_filter
//功能概要：低通滤波(一阶滞后滤波环节),滤掉高频信号
//函数返回：float32
//参数说明：new_val:最新采样值
//-----------------------------------------------------------------
float low_pass_filter(float new_val)
{
    static uint8_t i = 0;
    static float sample_val = 0;    //存放上次滤波结果输出值
    static float alfa = 0.09f;       //(alfa<<1)（可调）
                                                                    //滤波平滑系数:alfa=1-exp(-T/tao)≈T/tao   T:采样周期，tao:滤波器的时间常数(滤波器的响应时间)
    if(i==0)
    {
        sample_val = new_val;
        i = 1;
    }//end of if(i==0) 初始化初值为new_val

    sample_val = (1 - alfa) * sample_val + alfa * new_val;

    return sample_val;
}

//-----------------------------------------------------------------
//函数名称：slide_arithmetic_mean_filter
//功能概要：滑动算术均值滤波
//函数返回：int32
//参数说明：dat:滤波对象(dat[0]为最新值)
//            n:滤波阶数
//适用对象：对周期脉动的采样值进行平滑加工处理。
//          对脉冲性干扰平滑作用不理想，不适用于脉冲性干扰比较严重的场合。
//          如需看脉冲干扰，可以去掉最小、最大值再取平均值。
//-----------------------------------------------------------------
int32_t slide_arithmetic_mean_filter(int32_t dat, uint8_t n)
{
    static uint8_t i = 0,j = 0;
    static int32_t val[100]; //建立队列并且定义队列长度为100
    static int32_t sum = 0, mean_val = 0;
    //static int32* extremum;

    if(j == 0)
    {
        for(i = 0; i < n; i++)
        {
                val[i] = dat;
        }
        j = 1;
    }//end of if(j == 0) 初始化初值为dat

    sum = 0;
    //更新队列
    for(i=n-1; i>0; i--)
    {
            val[i] = val[i-1];
            sum += val[i];
    }

    val[0] = dat;
    sum += val[0];

    mean_val = sum / n;

    //extremum = get_extremum(val,n);
    //mean_val = (sum - extremum[0] - extremum[1]) / (n-2);

    return mean_val;
}

//-----------------------------------------------------------------
//函数名称：ms5611_reset
//功能概要：MS5611_01BA01复位
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void ms5611_reset(void)
{
    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);  //发送从机地址
    I2C_check_ack();
    I2C_send_one_char(MS5611_01BA01_RST_CMD);       //发送复位命令
    I2C_check_ack();
    I2C_stop();
}

//-----------------------------------------------------------------
//函数名称：ms5611_prom_read
//功能概要：MS5611_01BA01读取PROM值C1~C6,地址:1010 a2 a1 a0 0,这六个值是出厂校准数据只需读取一次即可。
//函数返回：void
//参数说明：s:读出的C参数存放的首地址。（PROM中的6个出厂校准数据(需要C[1]-C[6])(0XA2,0XA4,0XA6,0XA8,0XAA,0XAC)）
//-----------------------------------------------------------------
void ms5611_prom_read(uint16_t *s)
{
    static uint8_t i;                 //C[i]计数变量
    static uint8_t data[16];          //临时存储C[i]变量的高低位

    for(i = 0; i < 8; i++)                //读取C[0]~C[7]
    {
        I2C_start();
        I2C_send_one_char(MS5611_01BA01_SlaveAddress);     //发送从机地址
        I2C_check_ack();
        I2C_send_one_char(MS5611_01BA01_PROM_RD_CMD + i * 2);  //发送命令，读取PROM中MS5611_01BA01_PROM_RD_CMD+i*2地址对应的C[i]系数
        I2C_check_ack();
        I2C_stop();
        I2C_start();
        I2C_send_one_char(MS5611_01BA01_SlaveAddress + 1);   //发送从机地址（读）
        I2C_check_ack();
        I2C_recv_one_char(&data[i * 2]);
        I2C_ack();
        I2C_recv_one_char(&data[i * 2 + 1]);                   //接收C[i]数据高地位
        I2C_no_ack();
        I2C_stop();

        s[i] = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];       //计算C[i]
        STM32_Delay_ms(1);
    }
}

//-----------------------------------------------------------------
//函数名称：ms5611_do_conversion
//功能概要：启动温度/压力ADC转换并返回结果
//函数返回：conversion:转换完成并且读到的数据值
//参数说明：cmd_type:需要启动的转换类型(气压转换/温度转换)
//          cmd_delay:转换需要的时间间隔(us)
//    举例：cmd_type = MS5611_01BA01_D1_OSR_256_CMD/MS5611_01BA01_D2_OSR_256_CMD
//-----------------------------------------------------------------
uint32_t ms5611_do_conversion(uint8_t cmd_type, uint16_t cmd_delay)
{
    uint8_t adc[3];
    uint32_t conversion=0;

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);
    I2C_check_ack();
    I2C_send_one_char(cmd_type);                     //命令，开始cmd_type类型转换
    I2C_check_ack();
    I2C_stop();
    //注意：如果读取的数据不对，就很有可能是下面延时太短引起的！！！
    STM32_Delay_us(cmd_delay);//等待转换完成,等待时间长度由cmd_type对应的过采样率决定

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);
    I2C_check_ack();
    I2C_send_one_char(MS5611_01BA01_ADC_RD_CMD);     //命令，读ADC
    I2C_check_ack();
    I2C_stop();

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress + 1);
    I2C_check_ack();
    I2C_recv_one_char(&adc[0]);
    I2C_ack();
    I2C_recv_one_char(&adc[1]);
    I2C_ack();
    I2C_recv_one_char(&adc[2]);                            //接收ADC值
    I2C_no_ack();
    I2C_stop();

    conversion=(((uint32_t)(adc[0])) << 16) + (((uint32_t)(adc[1]) << 8)) + adc[2];   //数据处理

    return conversion;
}

//-----------------------------------------------------------------
//函数名称：ms5611_init
//功能概要：MS5611_01BA01初始化，复位并读取出厂参数
//函数返回：void
//参数说明：void
//测得C[1]~C[6]:53679,53321,33046,27992,32233,29103(不同芯片读取出的数据不同！)
//-----------------------------------------------------------------
void My_MS5611_init(void)
{
    ms5611_reset();
    STM32_Delay_ms(1);
    ms5611_prom_read(g_C);		
    STM32_Delay_ms(1);
}

//-----------------------------------------------------------------
//函数名称：ms5611_get_temperature
//功能概要：获取温度值，将读取到的原始温度值通过出厂校准数据校准从而得到真实温度值(输出摄氏温度值，放大了100倍,如2007=20.07℃)
//函数返回：temperature:int32型，真实温度的100倍
//参数说明：void
//-----------------------------------------------------------------
float ms5611_get_temperature(void)
{
    static int32_t D2 = 0;                   //存放原始温度值
    static float dT = 0;
    static float temperature = 0;          //最终真实温度值(放大100倍)

    D2 = ms5611_do_conversion(MS5611_01BA01_D2_OSR_4096_CMD, DELAY_4096);  //启动温度ADC转换并返回结果
    dT = D2 - (((int32_t)g_C[5]) << 8);						  //计算实测温度和参考温度的差异
    temperature = (float)(2000 + dT * (int32_t)g_C[6] / 8388608);        //计算实际温度

    return temperature / 100;
}

//-----------------------------------------------------------------
//函数名称：ms5611_get_pressure
//功能概要：获取压力值，将读取到的原始压力值通过出厂校准数据校准从而得到真实压力值(输出气压单位为帕斯卡,如100009=100009帕)
//函数返回：pressure：int32型，真实大气压（帕斯卡）
//参数说明：void
//-----------------------------------------------------------------
float ms5611_get_pressure(void)
{
    static int32_t D2 = 0;                           //存放原始温度值
    static float dT = 0;
    static int32_t temperature = 0;				     //最终真实温度值

    static int32_t D1 = 0;                           //存放原始压力值
    static float OFF = 0, SENS = 0;
    static float T2 = 0, OFF2 = 0, SENS2 = 0, Aux = 0; //二阶温度补偿变量
    static float pressure = 0;  				     //最终真实压力值(标准大气压)

    D2 = ms5611_do_conversion(MS5611_01BA01_D2_OSR_512_CMD, DELAY_512);//启动温度ADC转换并返回结果		
    dT = D2 - (((int32_t)g_C[5]) << 8);                           //计算实测温度和参考温度的差异
    temperature = (int32_t)(2000 + dT * (int32_t)g_C[6] / 8388608);          //计算实际温度

    D1 = ms5611_do_conversion(MS5611_01BA01_D1_OSR_512_CMD, DELAY_512);//启动压力ADC转换并返回结果
		
    OFF = (uint32_t)g_C[2] * 65536 + ((uint32_t)g_C[4] * dT) / 128;      //计算实际温度补偿
    SENS = (uint32_t)g_C[1] * 32768 + ((uint32_t)g_C[3] * dT) / 256;     //计算实际温度灵敏度

    //------------------------------------------------------------------------------------------
    if (temperature < 2000)        //low temperature 温度低于20℃时使用二阶温度补偿
    {
        T2 = (dT * dT) / 0x80000000;
        Aux = (temperature - 2000) * (temperature - 2000);
        OFF2 = Aux * 5 / 2;
        SENS2 = Aux * 5 / 4;
        if (temperature < -1500)   //very low temperature
        {
            Aux = (temperature + 1500) * (temperature + 1500);
            OFF2 = OFF2 + 7 * Aux;
            SENS2 = SENS2 + Aux * 11 / 2;
        }
    }
    else                      //high temperature
    {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }
    //------------------------------------------------------------------------------------------
    temperature = (int32_t)((float)temperature - T2);
#if 1
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
#endif
    pressure = (float)((D1 * SENS / 2097152 - OFF) / 32768.0f);  //计算实际压力(数值=帕*10倍)

    return pressure;
}

// 获取参考海拔高度
float MS5611_Get_Ref_Altitude(void)
{
		float Altitude;
		Altitude = 44330.0f * (1.0f - powf((ms5611_get_pressure() / 101325), (1.0f / 5.255f)));
		return Altitude;
}

// AltRef 参考海拔高度 单位m
float MS5611_Get_Altitude(float AltRef)
{
		float Altitude;
		Altitude = 44330.0f * (1.0f - powf((ms5611_get_pressure() / 101325), (1.0f / 5.255f)));
		Global_Show_Val[18] = ms5611_get_pressure();
		Global_Show_Val[19] = Altitude;
		Altitude = (Altitude - AltRef);
		return Altitude;
}
