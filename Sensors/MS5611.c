#include "MS5611.h"
#include "My_I2C.h"
#include "delay.h"

static uint16_t g_C[8]={0, 0, 0, 0, 0, 0, 0, 0}; //��̬���ݳ�Ա��ʼ��(����ȱ��)

//-----------------------------------------------------------------
//�������ƣ�low_pass_filter
//���ܸ�Ҫ����ͨ�˲�(һ���ͺ��˲�����),�˵���Ƶ�ź�
//�������أ�float32
//����˵����new_val:���²���ֵ
//-----------------------------------------------------------------
float low_pass_filter(float new_val)
{
    static uint8_t i = 0;
    static float sample_val = 0;    //����ϴ��˲�������ֵ
    static float alfa = 0.09f;       //(alfa<<1)���ɵ���
                                                                    //�˲�ƽ��ϵ��:alfa=1-exp(-T/tao)��T/tao   T:�������ڣ�tao:�˲�����ʱ�䳣��(�˲�������Ӧʱ��)
    if(i==0)
    {
        sample_val = new_val;
        i = 1;
    }//end of if(i==0) ��ʼ����ֵΪnew_val

    sample_val = (1 - alfa) * sample_val + alfa * new_val;

    return sample_val;
}

//-----------------------------------------------------------------
//�������ƣ�slide_arithmetic_mean_filter
//���ܸ�Ҫ������������ֵ�˲�
//�������أ�int32
//����˵����dat:�˲�����(dat[0]Ϊ����ֵ)
//            n:�˲�����
//���ö��󣺶����������Ĳ���ֵ����ƽ���ӹ�����
//          �������Ը���ƽ�����ò����룬�������������Ը��űȽ����صĳ��ϡ�
//          ���迴������ţ�����ȥ����С�����ֵ��ȡƽ��ֵ��
//-----------------------------------------------------------------
int32_t slide_arithmetic_mean_filter(int32_t dat, uint8_t n)
{
    static uint8_t i = 0,j = 0;
    static int32_t val[100]; //�������в��Ҷ�����г���Ϊ100
    static int32_t sum = 0, mean_val = 0;
    //static int32* extremum;

    if(j == 0)
    {
        for(i = 0; i < n; i++)
        {
                val[i] = dat;
        }
        j = 1;
    }//end of if(j == 0) ��ʼ����ֵΪdat

    sum = 0;
    //���¶���
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
//�������ƣ�ms5611_reset
//���ܸ�Ҫ��MS5611_01BA01��λ
//�������أ�void
//����˵������
//-----------------------------------------------------------------
void ms5611_reset(void)
{
    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);  //���ʹӻ���ַ
    I2C_check_ack();
    I2C_send_one_char(MS5611_01BA01_RST_CMD);       //���͸�λ����
    I2C_check_ack();
    I2C_stop();
}

//-----------------------------------------------------------------
//�������ƣ�ms5611_prom_read
//���ܸ�Ҫ��MS5611_01BA01��ȡPROMֵC1~C6,��ַ:1010 a2 a1 a0 0,������ֵ�ǳ���У׼����ֻ���ȡһ�μ��ɡ�
//�������أ�void
//����˵����s:������C������ŵ��׵�ַ����PROM�е�6������У׼����(��ҪC[1]-C[6])(0XA2,0XA4,0XA6,0XA8,0XAA,0XAC)��
//-----------------------------------------------------------------
void ms5611_prom_read(uint16_t *s)
{
    static uint8_t i;                 //C[i]��������
    static uint8_t data[16];          //��ʱ�洢C[i]�����ĸߵ�λ

    for(i = 0; i < 8; i++)                //��ȡC[0]~C[7]
    {
        I2C_start();
        I2C_send_one_char(MS5611_01BA01_SlaveAddress);     //���ʹӻ���ַ
        I2C_check_ack();
        I2C_send_one_char(MS5611_01BA01_PROM_RD_CMD + i * 2);  //���������ȡPROM��MS5611_01BA01_PROM_RD_CMD+i*2��ַ��Ӧ��C[i]ϵ��
        I2C_check_ack();
        I2C_stop();
        I2C_start();
        I2C_send_one_char(MS5611_01BA01_SlaveAddress + 1);   //���ʹӻ���ַ������
        I2C_check_ack();
        I2C_recv_one_char(&data[i * 2]);
        I2C_ack();
        I2C_recv_one_char(&data[i * 2 + 1]);                   //����C[i]���ݸߵ�λ
        I2C_no_ack();
        I2C_stop();

        s[i] = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];       //����C[i]
        STM32_Delay_ms(1);
    }
}

//-----------------------------------------------------------------
//�������ƣ�ms5611_do_conversion
//���ܸ�Ҫ�������¶�/ѹ��ADCת�������ؽ��
//�������أ�conversion:ת����ɲ��Ҷ���������ֵ
//����˵����cmd_type:��Ҫ������ת������(��ѹת��/�¶�ת��)
//          cmd_delay:ת����Ҫ��ʱ����(us)
//    ������cmd_type = MS5611_01BA01_D1_OSR_256_CMD/MS5611_01BA01_D2_OSR_256_CMD
//-----------------------------------------------------------------
uint32_t ms5611_do_conversion(uint8_t cmd_type, uint16_t cmd_delay)
{
    uint8_t adc[3];
    uint32_t conversion=0;

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);
    I2C_check_ack();
    I2C_send_one_char(cmd_type);                     //�����ʼcmd_type����ת��
    I2C_check_ack();
    I2C_stop();
    //ע�⣺�����ȡ�����ݲ��ԣ��ͺ��п�����������ʱ̫������ģ�����
    STM32_Delay_us(cmd_delay);//�ȴ�ת�����,�ȴ�ʱ�䳤����cmd_type��Ӧ�Ĺ������ʾ���

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress);
    I2C_check_ack();
    I2C_send_one_char(MS5611_01BA01_ADC_RD_CMD);     //�����ADC
    I2C_check_ack();
    I2C_stop();

    I2C_start();
    I2C_send_one_char(MS5611_01BA01_SlaveAddress + 1);
    I2C_check_ack();
    I2C_recv_one_char(&adc[0]);
    I2C_ack();
    I2C_recv_one_char(&adc[1]);
    I2C_ack();
    I2C_recv_one_char(&adc[2]);                            //����ADCֵ
    I2C_no_ack();
    I2C_stop();

    conversion=(((uint32_t)(adc[0])) << 16) + (((uint32_t)(adc[1]) << 8)) + adc[2];   //���ݴ���

    return conversion;
}

//-----------------------------------------------------------------
//�������ƣ�ms5611_init
//���ܸ�Ҫ��MS5611_01BA01��ʼ������λ����ȡ��������
//�������أ�void
//����˵����void
//���C[1]~C[6]:53679,53321,33046,27992,32233,29103(��ͬоƬ��ȡ�������ݲ�ͬ��)
//-----------------------------------------------------------------
void My_MS5611_init(void)
{
    ms5611_reset();
    STM32_Delay_ms(1);
    ms5611_prom_read(g_C);		
    STM32_Delay_ms(1);
}

//-----------------------------------------------------------------
//�������ƣ�ms5611_get_temperature
//���ܸ�Ҫ����ȡ�¶�ֵ������ȡ����ԭʼ�¶�ֵͨ������У׼����У׼�Ӷ��õ���ʵ�¶�ֵ(��������¶�ֵ���Ŵ���100��,��2007=20.07��)
//�������أ�temperature:int32�ͣ���ʵ�¶ȵ�100��
//����˵����void
//-----------------------------------------------------------------
float ms5611_get_temperature(void)
{
    static int32_t D2 = 0;                   //���ԭʼ�¶�ֵ
    static float dT = 0;
    static float temperature = 0;          //������ʵ�¶�ֵ(�Ŵ�100��)

    D2 = ms5611_do_conversion(MS5611_01BA01_D2_OSR_4096_CMD, DELAY_4096);  //�����¶�ADCת�������ؽ��
    dT = D2 - (((int32_t)g_C[5]) << 8);						  //����ʵ���¶ȺͲο��¶ȵĲ���
    temperature = (float)(2000 + dT * (int32_t)g_C[6] / 8388608);        //����ʵ���¶�

    return temperature / 100;
}

//-----------------------------------------------------------------
//�������ƣ�ms5611_get_pressure
//���ܸ�Ҫ����ȡѹ��ֵ������ȡ����ԭʼѹ��ֵͨ������У׼����У׼�Ӷ��õ���ʵѹ��ֵ(�����ѹ��λΪ��˹��,��100009=100009��)
//�������أ�pressure��int32�ͣ���ʵ����ѹ����˹����
//����˵����void
//-----------------------------------------------------------------
float ms5611_get_pressure(void)
{
    static int32_t D2 = 0;                           //���ԭʼ�¶�ֵ
    static float dT = 0;
    static int32_t temperature = 0;				     //������ʵ�¶�ֵ

    static int32_t D1 = 0;                           //���ԭʼѹ��ֵ
    static float OFF = 0, SENS = 0;
    static float T2 = 0, OFF2 = 0, SENS2 = 0, Aux = 0; //�����¶Ȳ�������
    static float pressure = 0;  				     //������ʵѹ��ֵ(��׼����ѹ)

    D2 = ms5611_do_conversion(MS5611_01BA01_D2_OSR_512_CMD, DELAY_512);//�����¶�ADCת�������ؽ��		
    dT = D2 - (((int32_t)g_C[5]) << 8);                           //����ʵ���¶ȺͲο��¶ȵĲ���
    temperature = (int32_t)(2000 + dT * (int32_t)g_C[6] / 8388608);          //����ʵ���¶�

    D1 = ms5611_do_conversion(MS5611_01BA01_D1_OSR_512_CMD, DELAY_512);//����ѹ��ADCת�������ؽ��
		
    OFF = (uint32_t)g_C[2] * 65536 + ((uint32_t)g_C[4] * dT) / 128;      //����ʵ���¶Ȳ���
    SENS = (uint32_t)g_C[1] * 32768 + ((uint32_t)g_C[3] * dT) / 256;     //����ʵ���¶�������

    //------------------------------------------------------------------------------------------
    if (temperature < 2000)        //low temperature �¶ȵ���20��ʱʹ�ö����¶Ȳ���
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
    pressure = (float)((D1 * SENS / 2097152 - OFF) / 32768.0f);  //����ʵ��ѹ��(��ֵ=��*10��)

    return pressure;
}

// ��ȡ�ο����θ߶�
float MS5611_Get_Ref_Altitude(void)
{
		float Altitude;
		Altitude = 44330.0f * (1.0f - powf((ms5611_get_pressure() / 101325), (1.0f / 5.255f)));
		return Altitude;
}

// AltRef �ο����θ߶� ��λm
float MS5611_Get_Altitude(float AltRef)
{
		float Altitude;
		Altitude = 44330.0f * (1.0f - powf((ms5611_get_pressure() / 101325), (1.0f / 5.255f)));
		Global_Show_Val[18] = ms5611_get_pressure();
		Global_Show_Val[19] = Altitude;
		Altitude = (Altitude - AltRef);
		return Altitude;
}
