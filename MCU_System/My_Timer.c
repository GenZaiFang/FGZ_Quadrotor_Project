#include "My_Timer.h"
#include "delay.h"

void Systick_init(uint8_t period)
{	
    //�����NVIC_SetPriority����Ĭ������SysTick_IRQn���ȼ�Ϊ15(1111),���ȼ���ͣ�
    while(SysTick_Config(168 * 1000 * (uint32_t)period)); //�ȴ��������
    NVIC_SetPriority (SysTick_IRQn, 15);             //��������systick���ȼ�
}



//Tim1��ʼ��ΪPWMģʽ
void Tim1_init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_OCInitTypeDef TIM_OCInitStructure;

		uint16_t Prescaler = 14-1;     //PSCԤ��Ƶֵ(0-65535),��PCLK1��168M����14��Ƶ���ü���ʱ��Ƶ��Ϊ12MHz
		uint16_t Period = 30000-1;    //ARR�Զ�����ֵ(0-65535)������=period/12MHz=30000/12000000=2.5ms(400Hz)
		uint16_t CCR1_Val  = 13050;   //ռ�ձ�=CCRX/ARR=13050/30000=43.5%   (��ˢ�����С����)
		uint16_t CCR2_Val  = 13050;   //ռ�ձ�=CCRX/ARR=13050/30000=43.5%   (��ˢ�����С����)
		uint16_t CCR3_Val  = 13050;   //ռ�ձ�=CCRX/ARR=13050/30000=43.5%   (��ˢ�����С����)
		uint16_t CCR4_Val  = 13050;   //ռ�ձ�=CCRX/ARR=13050/30000=43.5%   (��ˢ�����С����)

		//GPIOʱ��ʹ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��PAʱ��
		//GPIO���� (TIM1 CH1:PA8,CH2:PA9,CH3:PA10,CH4:PA11)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //��ʼ������ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //�˿�����100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //����
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PA��ʼ��
		//ָ����������
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);

		//����TIM1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 		    //ʹ��TIM1ʱ��								    
		//ʱ������
		TIM_DeInit(TIM1);                                               //TIM1�Ĵ�����λ
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;                //���ü���ʱ��Ԥ��Ƶֵ   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //���ü�������
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //��Ƶֵ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);                 //TIM1ʱ������

		//PWMģʽ����
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //PWM1:����ռ�ձ�ģʽ��PWM2:������ģʽ
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //�������
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //�����������
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //High:ռ�ձ�Ϊ�߼��ԣ�Low:ռ�ձ�Ϊ������
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //����������������෴
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
		//Channel 1
		TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                       //ռ�ձ���ֵ�����ü��ص���׽�ȽϼĴ���������ֵ,��ֵ�����������ֵ�Ƚϲ����̶�ռ�ձȵ�PWM���� ռ�ձ�=CCRX/ARR
		TIM_OC1Init(TIM1,&TIM_OCInitStructure);							//Channel 1��ʼ��
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               //CCRX�Զ�װ�ش�
		//Channel 2
		TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
		TIM_OC2Init(TIM1,&TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		//Channel 3
		TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
		TIM_OC3Init(TIM1,&TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		//Channel 4
		TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
		TIM_OC4Init(TIM1,&TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM1,ENABLE);              				//ʹ��TIM1 ARR�Զ�װ�ؼĴ���
		TIM_Cmd(TIM1,ENABLE);              								      //ʹ��TIM1������
		TIM_CtrlPWMOutputs(TIM1, ENABLE);                       //ʹ��TIM1��PWM�����TIM1��TIM8��Ч
}

void Tim1_PWM_Output(uint16_t pwm1Val, uint16_t pwm2Val, uint16_t pwm3Val, uint16_t pwm4Val)
{
		TIM1->CCR1 = pwm4Val;
		TIM1->CCR2 = pwm3Val;
		TIM1->CCR3 = pwm2Val;
		TIM1->CCR4 = pwm1Val;
}

void Tim3_init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
			
		uint16_t Prescaler = 7-1;       //PSCԤ��Ƶֵ(0-65535),��PCLK1��84M����7��Ƶ���ü���ʱ��Ƶ��Ϊ12MHz
		uint16_t Period = 65535;         //ARR�Զ�����ֵ(0-65535)������=period/12M=65535/4000000s=65.535/12ms
		
		//GPIOʱ��ʹ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��PAʱ��
		//GPIO���� (TIM3 CH1:PA6,CH2:PA7)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //��ʼ������ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //�˿�����50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //����
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PA��ʼ��
		//ָ����������
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
			
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //ʹ��PBʱ��
		//GPIO���� (TIM3 CH3:PB0,CH4:PB1)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //��ʼ������ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //�˿�����50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //����
		GPIO_Init(GPIOB, &GPIO_InitStructure);                 //PB��ʼ��
		//ָ����������
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);
			
			//ʹ��TIM3ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//ʱ������
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//���ü���ʱ��Ԥ��Ƶֵ   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //���ü�������
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //��Ƶֵ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM3,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM3,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM3,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM3,&TIM_ICInitStructure);
			
		TIM_Cmd(TIM3,ENABLE);                                           //������ʹ��
		 
		//����TIM3ȫ���ж�
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                 //ָ��TIM3_IRQnͨ��ʹ�ܻ�ʧ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //��ռʽ���ȼ�Ϊ2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //��Ӧʽ���ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��TIM3_IRQn�ж�
		NVIC_Init(&NVIC_InitStructure);  
		
		//�ж�ʹ��
		TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//�������ж�
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
		TIM_ClearFlag(TIM3, TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);
}

void Tim3_Init_Enable(void)
{
		TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
}

void Tim3_Init_Disable(void)
{
		TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}

void Tim4_init(void)
{
#if 0	
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    uint16_t Prescaler = 8400-1;     //PSCԤ��Ƶֵ(0-65535),��PCLK1��84M����8400��Ƶ���ü���ʱ��Ƶ��Ϊ10KHz
    uint16_t Period = Deal_Period_ms * 10;         //ARR�Զ�����ֵ(0-65535)������ = period/10KHz = 100/10000 = 0.01s

    //ʹ��TIM4ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //ʱ������
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;                //���ü���ʱ��Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_Period = Period;                      //���ü�������
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //��Ƶֵ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //����TIM4ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //ָ��TIM4_IRQnͨ��ʹ�ܻ�ʧ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;       //��ռʽ���ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //��Ӧʽ���ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��TIM4_IRQn�ж�
    NVIC_Init(&NVIC_InitStructure);
    //�ж�ʹ��
    TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);
    //������ʹ��
    TIM_Cmd(TIM4, ENABLE);
#else
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		uint16_t Prescaler = 7-1;        //PSCԤ��Ƶֵ(0-65535),��PCLK1��84M����7��Ƶ���ü���ʱ��Ƶ��Ϊ12MHz
		uint16_t Period = 65535;         //ARR�Զ�����ֵ(0-65535)������=period/12M=65535/4000000s=65.535/12ms

		//GPIOʱ��ʹ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  //ʹ��PDʱ��
		//GPIO���� (TIM5 CH2:PD13,CH3:PD14,CH4:PD15)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //��ʼ������ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //�˿�����50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //����
		GPIO_Init(GPIOD, &GPIO_InitStructure);                 //PD��ʼ��
		//ָ����������
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);

		//ʹ��TIM4ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//ʱ������
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//���ü���ʱ��Ԥ��Ƶֵ   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //���ü�������
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //��Ƶֵ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                //ѡ��ͨ����
		TIM_ICInitStructure.TIM_ICFilter = 0;                           //���˲�
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //�����ز���
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //��IC1ӳ�䵽TI1��
		TIM_ICInit(TIM4,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM4,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;                
		TIM_ICInitStructure.TIM_ICFilter = 0;                           
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM4,&TIM_ICInitStructure);

		TIM_Cmd(TIM4,ENABLE);                                           //������ʹ��

		//����TIM4ȫ���ж�
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //ָ��TIM4_IRQnͨ��ʹ�ܻ�ʧ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //��ռʽ���ȼ�Ϊ2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //��Ӧʽ���ȼ�Ϊ1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��TIM4_IRQn�ж�
		NVIC_Init(&NVIC_InitStructure);  

		//�ж�ʹ��
		TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);    //�������ж�
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
		TIM_ClearFlag(TIM4, TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);
#endif
}

void Tim4_Init_Enable(void)
{
		TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
}

void Tim4_Init_Disable(void)
{
		TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}

void Tim14_init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		uint16_t Prescaler = 84-1;       //PSCԤ��Ƶֵ(0-65535),��PCLK1��84M����84��Ƶ���ü���ʱ��Ƶ��Ϊ1MHz
		uint16_t Period = 65535;         //ARR�Զ�����ֵ(0-65535)������=period/12M=65535/4000000s=65.535/1 ms

		//GPIOʱ��ʹ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��PAʱ��
		//GPIO���� (TIM14 CH1:PA7)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //��ʼ������ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //�˿�����50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //����
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PD��ʼ��
		//ָ����������
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM14);

		//ʹ��TIM14ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
		//ʱ������
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//���ü���ʱ��Ԥ��Ƶֵ   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //���ü�������
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //��Ƶֵ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
		TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //ѡ��ͨ����
		TIM_ICInitStructure.TIM_ICFilter = 0;                           //���˲�
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //�����ز���
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //��IC1ӳ�䵽TI1��
		TIM_ICInit(TIM14,&TIM_ICInitStructure);

		TIM_Cmd(TIM14,ENABLE);                                           //������ʹ��

		//����TIM14ȫ���ж�
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;   //ָ��TIM8_TRG_COM_TIM14_IRQnͨ��ʹ�ܻ�ʧ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //��ռʽ���ȼ�Ϊ2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;              //��Ӧʽ���ȼ�Ϊ3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��TIM8_TRG_COM_TIM14_IRQn�ж�
		NVIC_Init(&NVIC_InitStructure);  

		//�ж�ʹ��
		TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);                         //�������ж�
		TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
		TIM_ClearFlag(TIM14, TIM_FLAG_CC1);
}

void My_STM32_TIMER_init(void)
{		
		Tim1_init();
		Tim3_init();
		Tim4_init();
		Tim14_init();
		Systick_init(Deal_Period_ms);		
}
