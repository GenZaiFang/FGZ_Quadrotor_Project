#include "My_Timer.h"
#include "delay.h"

void Systick_init(uint8_t period)
{	
    //里面的NVIC_SetPriority函数默认设置SysTick_IRQn优先级为15(1111),优先级最低！
    while(SysTick_Config(168 * 1000 * (uint32_t)period)); //等待配置完成
    NVIC_SetPriority (SysTick_IRQn, 15);             //重新设置systick优先级
}



//Tim1初始化为PWM模式
void Tim1_init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_OCInitTypeDef TIM_OCInitStructure;

		uint16_t Prescaler = 14-1;     //PSC预分频值(0-65535),对PCLK1的168M进行14分频，得计数时钟频率为12MHz
		uint16_t Period = 30000-1;    //ARR自动重载值(0-65535)，周期=period/12MHz=30000/12000000=2.5ms(400Hz)
		uint16_t CCR1_Val  = 13050;   //占空比=CCRX/ARR=13050/30000=43.5%   (无刷电机最小油门)
		uint16_t CCR2_Val  = 13050;   //占空比=CCRX/ARR=13050/30000=43.5%   (无刷电机最小油门)
		uint16_t CCR3_Val  = 13050;   //占空比=CCRX/ARR=13050/30000=43.5%   (无刷电机最小油门)
		uint16_t CCR4_Val  = 13050;   //占空比=CCRX/ARR=13050/30000=43.5%   (无刷电机最小油门)

		//GPIO时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能PA时钟
		//GPIO配置 (TIM1 CH1:PA8,CH2:PA9,CH3:PA10,CH4:PA11)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //初始化复用模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //端口速率100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PA初始化
		//指定复用引脚
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);

		//配置TIM1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 		    //使能TIM1时钟								    
		//时基配置
		TIM_DeInit(TIM1);                                               //TIM1寄存器复位
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;                //设置计数时钟预分频值   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //设置计数周期
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //分频值
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);                 //TIM1时基配置

		//PWM模式配置
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //PWM1:正常占空比模式，PWM2:反极性模式
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //允许输出
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //互补输出允许
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //High:占空比为高极性，Low:占空比为反极性
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //互补输出，与以上相反
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
		//Channel 1
		TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                       //占空比数值，设置加载到捕捉比较寄存器的脉冲值,该值将与计数器的值比较产生固定占空比的PWM波形 占空比=CCRX/ARR
		TIM_OC1Init(TIM1,&TIM_OCInitStructure);							//Channel 1初始化
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);               //CCRX自动装载打开
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

		TIM_ARRPreloadConfig(TIM1,ENABLE);              				//使能TIM1 ARR自动装载寄存器
		TIM_Cmd(TIM1,ENABLE);              								      //使能TIM1计数器
		TIM_CtrlPWMOutputs(TIM1, ENABLE);                       //使能TIM1的PWM输出，TIM1与TIM8有效
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
			
		uint16_t Prescaler = 7-1;       //PSC预分频值(0-65535),对PCLK1的84M进行7分频，得计数时钟频率为12MHz
		uint16_t Period = 65535;         //ARR自动重载值(0-65535)，周期=period/12M=65535/4000000s=65.535/12ms
		
		//GPIO时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能PA时钟
		//GPIO配置 (TIM3 CH1:PA6,CH2:PA7)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //初始化复用模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //下拉
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PA初始化
		//指定复用引脚
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
			
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //使能PB时钟
		//GPIO配置 (TIM3 CH3:PB0,CH4:PB1)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //初始化复用模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //下拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);                 //PB初始化
		//指定复用引脚
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);
			
			//使能TIM3时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		//时基配置
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//设置计数时钟预分频值   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //设置计数周期
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //分频值
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
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
			
		TIM_Cmd(TIM3,ENABLE);                                           //计数器使能
		 
		//启用TIM3全局中断
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                 //指定TIM3_IRQn通道使能或失能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //抢占式优先级为2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //响应式优先级为0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能TIM3_IRQn中断
		NVIC_Init(&NVIC_InitStructure);  
		
		//中断使能
		TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许捕获中断
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
    uint16_t Prescaler = 8400-1;     //PSC预分频值(0-65535),对PCLK1的84M进行8400分频，得计数时钟频率为10KHz
    uint16_t Period = Deal_Period_ms * 10;         //ARR自动重载值(0-65535)，周期 = period/10KHz = 100/10000 = 0.01s

    //使能TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //时基配置
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;                //设置计数时钟预分频值
    TIM_TimeBaseStructure.TIM_Period = Period;                      //设置计数周期
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //分频值
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //启用TIM4全局中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //指定TIM4_IRQn通道使能或失能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;       //抢占式优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //响应式优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能TIM4_IRQn中断
    NVIC_Init(&NVIC_InitStructure);
    //中断使能
    TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);
    //计数器使能
    TIM_Cmd(TIM4, ENABLE);
#else
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		uint16_t Prescaler = 7-1;        //PSC预分频值(0-65535),对PCLK1的84M进行7分频，得计数时钟频率为12MHz
		uint16_t Period = 65535;         //ARR自动重载值(0-65535)，周期=period/12M=65535/4000000s=65.535/12ms

		//GPIO时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  //使能PD时钟
		//GPIO配置 (TIM5 CH2:PD13,CH3:PD14,CH4:PD15)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //初始化复用模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //下拉
		GPIO_Init(GPIOD, &GPIO_InitStructure);                 //PD初始化
		//指定复用引脚
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);

		//使能TIM4时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		//时基配置
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//设置计数时钟预分频值   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //设置计数周期
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //分频值
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                //选择通道号
		TIM_ICInitStructure.TIM_ICFilter = 0;                           //不滤波
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //上升沿捕获
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //将IC1映射到TI1上
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

		TIM_Cmd(TIM4,ENABLE);                                           //计数器使能

		//启用TIM4全局中断
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                 //指定TIM4_IRQn通道使能或失能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //抢占式优先级为2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //响应式优先级为1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能TIM4_IRQn中断
		NVIC_Init(&NVIC_InitStructure);  

		//中断使能
		TIM_ITConfig(TIM4, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);    //允许捕获中断
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

		uint16_t Prescaler = 84-1;       //PSC预分频值(0-65535),对PCLK1的84M进行84分频，得计数时钟频率为1MHz
		uint16_t Period = 65535;         //ARR自动重载值(0-65535)，周期=period/12M=65535/4000000s=65.535/1 ms

		//GPIO时钟使能
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能PA时钟
		//GPIO配置 (TIM14 CH1:PA7)
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //初始化复用模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //端口速率50MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;         //下拉
		GPIO_Init(GPIOA, &GPIO_InitStructure);                 //PD初始化
		//指定复用引脚
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM14);

		//使能TIM14时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
		//时基配置
		TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;           		//设置计数时钟预分频值   
		TIM_TimeBaseStructure.TIM_Period = Period;                      //设置计数周期
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //分频值
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
		TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //选择通道号
		TIM_ICInitStructure.TIM_ICFilter = 0;                           //不滤波
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //上升沿捕获
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //将IC1映射到TI1上
		TIM_ICInit(TIM14,&TIM_ICInitStructure);

		TIM_Cmd(TIM14,ENABLE);                                           //计数器使能

		//启用TIM14全局中断
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;   //指定TIM8_TRG_COM_TIM14_IRQn通道使能或失能
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //抢占式优先级为2
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;              //响应式优先级为3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能TIM8_TRG_COM_TIM14_IRQn中断
		NVIC_Init(&NVIC_InitStructure);  

		//中断使能
		TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);                         //允许捕获中断
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
