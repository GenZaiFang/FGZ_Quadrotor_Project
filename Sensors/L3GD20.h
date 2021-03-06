
#ifndef L3GD20_H_
#define L3GD20_H_

#include "System_Common.h"

// ����L3GD20�ڲ��Ĵ�����ַ
#define	L3GD20_WHO_AM_I		    0x0F
#define L3GD20_SlaveAddress         0xD6    //(11010110)

#define	L3GD20_CTRL_REG1	    0x20
#define	L3GD20_CTRL_REG2	    0x21    //��ͨ�˲����ƼĴ���
#define	L3GD20_CTRL_REG3	    0x22    //�жϿ��ƼĴ���
#define	L3GD20_CTRL_REG4	    0x23
#define	L3GD20_CTRL_REG5	    0x24

#define	L3GD20_REF_DATA		    0x25
#define	L3GD20_OUT_TEMP		    0x26    //�¶�����
#define	L3GD20_STATUS_REG	    0x27    //״̬�Ĵ���

#define	L3GD20_OUT_X_L		    0x28
#define	L3GD20_OUT_X_H		    0x29
#define	L3GD20_OUT_Y_L		    0x2A
#define	L3GD20_OUT_Y_H		    0x2B
#define	L3GD20_OUT_Z_L		    0x2C
#define	L3GD20_OUT_Z_H		    0x2D

#define	L3GD20_FIFO_CTRL_REG        0x2E   //FIFO���ƼĴ���
#define	L3GD20_FIFO_SRC_REG	    0x2F

#define	L3GD20_INT1_CFG		    0x30
#define	L3GD20_INT1_SRC		    0x31

#define	L3GD20_INT1_THS_XH	    0x32
#define	L3GD20_INT1_THS_XL	    0x33
#define	L3GD20_INT1_THS_YH	    0x34
#define	L3GD20_INT1_THS_YL	    0x35
#define	L3GD20_INT1_THS_ZH	    0x36
#define	L3GD20_INT1_THS_ZL	    0x37

#define	L3GD20_INT1_DURATION        0x38

void My_L3GD20_init(void);
void My_L3GD20_Get_gyro(int16_t *dat);
uint8_t Revise_L3GD20_GyroVal(void);

#endif
