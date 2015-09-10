//oled程序文件

#ifndef OLED_H_
#define OLED_H_

#include "System_Common.h"


//定义英文字模的存储结构体
typedef struct asciimat 
{
    char index;
    char Mat[16];
}ASCII_MAT;    

//定义中文字模的存储结构体 
typedef struct gb2312mat 
{
	char index[3];
	char Mat[32];
}GB2312_MAT;             

//端口命令宏定义 (SCL,SDA,RST,DC:PC9,PC8,PC7,PC6)
#define OLED_SCL_SET    GPIOC->BSRRL=GPIO_Pin_9			
#define OLED_SCL_CLR		GPIOC->BSRRH=GPIO_Pin_9
#define OLED_SDA_SET		GPIOC->BSRRL=GPIO_Pin_8			
#define OLED_SDA_CLR		GPIOC->BSRRH=GPIO_Pin_8
#define OLED_RST_SET		GPIOC->BSRRL=GPIO_Pin_7		
#define OLED_RST_CLR		GPIOC->BSRRH=GPIO_Pin_7
#define OLED_DC_SET			GPIOC->BSRRL=GPIO_Pin_6			
#define OLED_DC_CLR			GPIOC->BSRRH=GPIO_Pin_6

//功能概要：OLED清屏,调用模块: oled_wdat; oled_wcmd;
void oled_clear_screen(void);
//功能概要：OLED显示一个数值，调用模块: oled_ascii;
void oled_display_num(uint8_t location, int16_t number, uint8_t length, uint8_t pm, uint8_t inverse);
//功能概要：OLED显示一个字符串(可以为汉字),调用模块: oled_ascii; oled_gb2312
void oled_display_str(uint8_t location, char *string, uint8_t length, uint8_t inverse);
//功能概要：OLED显示一幅图像,调用模块: oled_wcmd;
void oled_display_fig(void);
//功能概要：将OLED指定连续位置反色显示,oled_ascii; oled_gb2312
void oled_display_inv(uint8_t location, uint8_t length, uint8_t inverse);
//功能概要：OLED初始化，调用模块: oled_wdat; oled_wcmd
void My_OLED_init(void);

#endif
