//oled�����ļ�

#ifndef OLED_H_
#define OLED_H_

#include "System_Common.h"


//����Ӣ����ģ�Ĵ洢�ṹ��
typedef struct asciimat 
{
    char index;
    char Mat[16];
}ASCII_MAT;    

//����������ģ�Ĵ洢�ṹ�� 
typedef struct gb2312mat 
{
	char index[3];
	char Mat[32];
}GB2312_MAT;             

//�˿�����궨�� (SCL,SDA,RST,DC:PC9,PC8,PC7,PC6)
#define OLED_SCL_SET    GPIOC->BSRRL=GPIO_Pin_9			
#define OLED_SCL_CLR		GPIOC->BSRRH=GPIO_Pin_9
#define OLED_SDA_SET		GPIOC->BSRRL=GPIO_Pin_8			
#define OLED_SDA_CLR		GPIOC->BSRRH=GPIO_Pin_8
#define OLED_RST_SET		GPIOC->BSRRL=GPIO_Pin_7		
#define OLED_RST_CLR		GPIOC->BSRRH=GPIO_Pin_7
#define OLED_DC_SET			GPIOC->BSRRL=GPIO_Pin_6			
#define OLED_DC_CLR			GPIOC->BSRRH=GPIO_Pin_6

//���ܸ�Ҫ��OLED����,����ģ��: oled_wdat; oled_wcmd;
void oled_clear_screen(void);
//���ܸ�Ҫ��OLED��ʾһ����ֵ������ģ��: oled_ascii;
void oled_display_num(uint8_t location, int16_t number, uint8_t length, uint8_t pm, uint8_t inverse);
//���ܸ�Ҫ��OLED��ʾһ���ַ���(����Ϊ����),����ģ��: oled_ascii; oled_gb2312
void oled_display_str(uint8_t location, char *string, uint8_t length, uint8_t inverse);
//���ܸ�Ҫ��OLED��ʾһ��ͼ��,����ģ��: oled_wcmd;
void oled_display_fig(void);
//���ܸ�Ҫ����OLEDָ������λ�÷�ɫ��ʾ,oled_ascii; oled_gb2312
void oled_display_inv(uint8_t location, uint8_t length, uint8_t inverse);
//���ܸ�Ҫ��OLED��ʼ��������ģ��: oled_wdat; oled_wcmd
void My_OLED_init(void);

#endif
