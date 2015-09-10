#include "My_I2C.h"
#include "delay.h"

//-----------------------------------------------------------------
//函数名称：I2C_start
//功能概要：启动总线,SCL_H时，SDA出现下降沿。
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void I2C_start(void)
{
    SDA_O;

    SDA_H;
    delay_1us();
    SCL_H;
    delay_5us();
    SDA_L;
    delay_5us();
    SCL_L;
    delay_2us();
}

//-----------------------------------------------------------------
//函数名称：I2C_stop
//功能概要：结束总线,SCL_H时，SDA出现上升沿。
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void I2C_stop(void)
{
    SDA_O;

    SDA_L;
    delay_1us();
    SCL_H;
    delay_5us();
    SDA_H;
    delay_4us();
}

//-----------------------------------------------------------------
//函数名称：I2C_ack
//功能概要：主机发送应答位，SCL_H时，保持SDA_L状态
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void I2C_ack(void)
{
    SDA_O;

    SDA_L;
    delay_1us();
    SCL_H;
    delay_5us();
    SCL_L;
    delay_2us();
}

//-----------------------------------------------------------------
//函数名称：I2C_no_ack
//功能概要：主机发送非应位，SCL_H时，保持SDA_H状态
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void I2C_no_ack(void)
{
    SDA_O;

    SDA_H;
    delay_1us();
    SCL_H;
    delay_5us();
    SCL_L;
    delay_2us();
}

//-----------------------------------------------------------------
//函数名称：I2C_check_ack
//功能概要：主机接收从机应答位，SCL_H时读取SDA，如果SDA_L则接收到应答位，否则接收到非应答位。
//          如果无应答，则主机要结束I2C总线
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
void I2C_check_ack(void)
{
    SDA_O;

    delay_3us();
    SCL_L;
    delay_3us();
    SDA_I;           //设置为输入模式

    SDA_H;
    delay_3us();
    SCL_H;
    delay_5us();

    if(SDA == 1)
    {
        I2C_stop();  //SDA!=0，未接收到应答位，结束总线
    }
    else
    {
        SCL_L;       //SDA==0，接收到应答位，拉低时钟线
    }
}

//-----------------------------------------------------------------
//函数名称：I2C_send_one_char
//功能概要：发送一个字节。
//          将数据c发送出去,可以是地址,也可以是数据,发送完成后需检查应答位
//函数返回：void
//参数说明：c :发送字节
//-----------------------------------------------------------------
void I2C_send_one_char(uint8_t c)
{
    uint8_t i;

    SDA_O;

    for (i = 0; i < 8; i++)        //8位计数器
    {
        if(c & 0x80)
          SDA_H;
        else
          SDA_L;               //送数据口

        c <<= 1;               //移出数据的最高位

        SCL_H;
        delay_3us();
        SCL_L;                 //产生SCL脉冲，在SCL_L时改变SDA状态！
        delay_3us();
    }
}

//-----------------------------------------------------------------
//函数名称：I2C_recv_one_char
//功能概要：接收一个字节。
//          用来接收从器件传来的一个字节。接收完成后需检查应答位
//函数返回：无
//参数说明：c :接收字节存放的地址
//-----------------------------------------------------------------
void I2C_recv_one_char(uint8_t *c)
{
    uint8_t i;
    uint8_t retc = 0;

    SDA_O;

    SDA_H;

    SDA_I;

    for(i = 0; i < 8; i++)
    {
         delay_1us();
         SCL_L;
         delay_5us();
         SCL_H;
         delay_2us();		//产生SCL脉冲，再SCL_H时读取SDA状态！

         retc = retc << 1;
         if(SDA == 1)
         retc = retc + 1;       //读数据位,接收的数据位放入retc中
         delay_2us();
    }
    SCL_L;

    *c = retc;
}

//-----------------------------------------------------------------
//函数名称：i2c_init
//功能概要：i2c初始化，初始化模拟i2c的IO口
//函数返回：void
//参数说明：无
//-----------------------------------------------------------------
uint8_t oos = 0;
void My_STM32_I2C_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //使能时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//模拟i2c用PB10、PB11口！
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //初始化为输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;         //开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       //无上下拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    SCL_H;
    SDA_H;
    oos = 9;
}

//-----------------------------------------------------------------
//函数名称：I2C_send_str
//功能概要：向有寄存器地址器件发送一个字符串。
//          从启动总线到发送地址，寄存器地址,发送数据，结束总线的全过程。
//          从机地址slave，寄存器地址reg,发送的内容是s指向的内容，发送num个字节。
//函数返回：void
//参数说明：slave:从机地址
//          reg  :寄存器地址
//          s    :发送字符串首地址
//          num  :字节个数
//-----------------------------------------------------------------
void I2C_send_str(uint8_t slave,uint8_t reg,uint8_t *s,uint8_t num)
{
    int i;

    I2C_start();                    //起始信号

    I2C_send_one_char(slave);       //发送设备地址(写)
    I2C_check_ack();
    I2C_send_one_char(reg);         //内部寄存器地址
    I2C_check_ack();

    for (i=0; i < num; i++)
    {
        I2C_send_one_char(s[i]);       //内部寄存器数据
        I2C_check_ack();               //寄存器地址有自增功能
    }

    I2C_stop();                     //发送停止信号
}

//-----------------------------------------------------------------
//函数名称：I2C_recv_str
//功能概要：向有寄存器地址器件接收一个字符串。
//          从启动总线到发送地址，寄存器地址,接收数据，结束总线的全过程。
//          从机地址slave，寄存器地址reg,接收的内容放入s指向的存储区，接收num个字节。
//函数返回：void
//参数说明：slave:从机地址
//          reg  :寄存器地址
//          s    :接收字符串首地址
//          num  :字节个数
//-----------------------------------------------------------------
void I2C_recv_str(uint8_t slave,uint8_t reg,uint8_t *s,uint8_t num)
{
     I2C_start();                    //起始信号

     I2C_send_one_char(slave);       //发送设备地址(写)
     I2C_check_ack();
     I2C_send_one_char(reg);         //发送存储单元地址，从0开始
     I2C_check_ack();

     I2C_start();                    //起始信号，表示下面发的是从机地址，而不是第三个字节！！！

     I2C_send_one_char(slave + 1);     //发送设备地址(读)
     I2C_check_ack();

     while(num)
     {
         I2C_recv_one_char(s);     //读出寄存器数据
         if (num == 1)
            I2C_no_ack();          //无应答，停止传输
         else
            I2C_ack();             //有应答，继续传输
         s++;
         num--;
     }

     I2C_stop();                     //停止信号
}
