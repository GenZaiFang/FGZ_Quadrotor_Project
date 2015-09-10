#include "My_USART.h"

union
{
    float f32;
    int32_t i32;
    uint32_t u32;
    uint8_t u8[4];
}mux_bytes_val;

void My_USARTn_DMA_Config(void)
{
#if defined USART1_DMA_Transfer || defined USART2_DMA_Transfer || defined USART3_DMA_Transfer
		NVIC_InitTypeDef  NVIC_InitStructure;	
		DMA_InitTypeDef DMA_InitStructure;
#endif
	
#ifdef USART1_DMA_Transfer			
		//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);           
	
		//DMA接收中断配置
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
		
		//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

		//DMA通道设置
		DMA_DeInit(DMA2_Stream5);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//外设地址
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
		
		//内存地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA2_Stream5_Rx_Buf;
		
		//DMA传输方向
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//设置DMA在传输时缓冲区长度
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN1;
		
		//设置DMA外设递增模式，只有一个外设
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//设置DMA内存递增模式
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//外设数据字长
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//内存数据字长
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//设置DMA传输模式
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//设置DMA优先级
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//指定FIFO模式或直接模式将用于指定流:不使用FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//指定FIFO的阈值水平
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//指定的Burst转移配置内存传输
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//指定的Burst转移配置外围转移
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//初始化DMA通道
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);

		//使能DMA通道
		DMA_Cmd(DMA2_Stream5, ENABLE);
		
		//采用DMA方式接收
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);		
#endif

#ifdef USART2_DMA_Transfer			
		//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);           
		
		//DMA接收中断配置
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure); 
			
		//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		//DMA通道设置
		DMA_DeInit(DMA1_Stream5);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//外设地址
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
		
		//内存地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA1_Stream5_Rx_Buf;
		
		//DMA传输方向
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//设置DMA在传输时缓冲区长度
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN2;
		
		//设置DMA外设递增模式，只有一个外设
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//设置DMA内存递增模式
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//外设数据字长
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//内存数据字长
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//设置DMA传输模式
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//设置DMA优先级
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//指定FIFO模式或直接模式将用于指定流:不使用FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//指定FIFO的阈值水平
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//指定的Burst转移配置内存传输
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//指定的Burst转移配置外围转移
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//初始化DMA通道
		DMA_Init(DMA1_Stream5, &DMA_InitStructure);

		//使能DMA通道
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		//采用DMA方式接收
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
#endif

#ifdef USART3_DMA_Transfer
//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);           
		
		//DMA接收中断配置
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure); 
			
		//启动DMA时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		//DMA通道设置
		DMA_DeInit(DMA1_Stream1);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//外设地址
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
		
		//内存地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA1_Stream1_Rx_Buf;
		
		//DMA传输方向
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//设置DMA在传输时缓冲区长度
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN3;
		
		//设置DMA外设递增模式，只有一个外设
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//设置DMA内存递增模式
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//外设数据字长
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//内存数据字长
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//设置DMA传输模式
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//设置DMA优先级
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//指定FIFO模式或直接模式将用于指定流:不使用FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//指定FIFO的阈值水平
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//指定的Burst转移配置内存传输
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//指定的Burst转移配置外围转移
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//初始化DMA通道
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);

		//使能DMA通道
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		//采用DMA方式接收
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
#endif		
}

// 初始化USART1..3; baudrate:设置波特率; INT_EN_FLAG:是否使能串口接收中断
void My_STM32_USART_init(uint32_t *baudrate, uint8_t *INT_EN_FLAG)
{
    GPIO_InitTypeDef  GPIO_InitStructure_Usart;                    //IO口复用为串口
    USART_InitTypeDef USART_InitStructure;                         //串口初始化
    NVIC_InitTypeDef  NVIC_InitStructure;                          //串口中断初始化		

/***************************************配置USART1*********************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   //P6，P7
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //初始化为复用模式
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //推挽输出
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //端口速率50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure_Usart);                   //PB初始化
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);      //TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);      //RX

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = baudrate[0];               //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8位数据模式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件溢出控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //双工模式
    USART_Init(USART1,&USART_InitStructure);
    USART_Cmd(USART1,ENABLE);                                       //使能USART1
    USART_ClearFlag(USART1, USART_FLAG_TXE);                        //清发送完成标志位

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //指定USART1_IRQn通道使能或失能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //抢占式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //响应式优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能USART1_IRQn中断
    NVIC_Init(&NVIC_InitStructure);                                 //中断向量配置
    USART_ITConfig(USART1, USART_IT_RXNE, (INT_EN_FLAG[0] == 1 ? ENABLE : DISABLE));//根据flag值决定是否使能USART1串口接收中断			

/***************************************配置USART2*********************************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;   //PD5，PD6
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //初始化为复用模式
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //推挽输出
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //端口速率50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure_Usart);                   //PC初始化
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);      //TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);      //RX

    USART_DeInit(USART2);
    USART_InitStructure.USART_BaudRate = baudrate[1];                //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8位数据模式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件溢出控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //双工模式
    USART_Init(USART2,&USART_InitStructure);
    USART_Cmd(USART2,ENABLE);                                       //使能USART2
    USART_ClearFlag(USART2, USART_FLAG_TXE);                        //清发送完成标志位

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //指定USART2_IRQn通道使能或失能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //抢占式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //响应式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能USART2_IRQn中断
    NVIC_Init(&NVIC_InitStructure);                                 //中断向量配置
    USART_ITConfig(USART2, USART_IT_RXNE, (INT_EN_FLAG[1] == 1 ? ENABLE : DISABLE));//根据flag值决定是否使能USART2串口接收中断

/***************************************配置USART3*********************************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //P10，P11
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //初始化为复用模式
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //推挽输出
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //端口速率50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure_Usart);                   //PC初始化
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);     //TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);     //RX

    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate = baudrate[2];               //波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8位数据模式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件溢出控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //双工模式
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);                                      //使能USART3
    USART_ClearFlag(USART3, USART_FLAG_TXE);                        //清发送完成标志位

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;               //指定USART3_IRQn通道使能或失能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //抢占式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //响应式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //使能USART3_IRQn中断
    NVIC_Init(&NVIC_InitStructure);                                 //中断向量配置
    USART_ITConfig(USART3, USART_IT_RXNE, (INT_EN_FLAG[2] == 1 ? ENABLE : DISABLE));//根据flag值决定是否使能USART3串口接收中断

/***************************************配置结束*********************************************/
}

// USARTn : 1..3
void My_USART_send_U8(uint8_t USARTn, uint8_t val)
{
    switch(USARTn)
    {
      case 1:
        USART_SendData(USART1, val);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        break;
      case 2:
        USART_SendData(USART2, val);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        break;
      case 3:
        USART_SendData(USART3, val);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        break;
    }
}

// x_flag : 0x0f ~ 0x6f
void My_USART_send_MUX_Bytes_x(uint8_t USARTn, uint8_t x_flag, float val)
{
    mux_bytes_val.f32 = val;
    My_USART_send_U8(USARTn, 0x55);    
    My_USART_send_U8(USARTn, x_flag);    
    My_USART_send_U8(USARTn, mux_bytes_val.u8[0]);   
    My_USART_send_U8(USARTn, mux_bytes_val.u8[1]);    
    My_USART_send_U8(USARTn, mux_bytes_val.u8[2]);    
    My_USART_send_U8(USARTn, mux_bytes_val.u8[3]);    
    My_USART_send_U8(USARTn, 0xaa);    
}

// USARTn : 1..3
uint8_t My_USART_read_U8(uint8_t USARTn)
{
		uint8_t val_tmp = 0;
		switch(USARTn)
		{
				case 1:
					val_tmp = (uint8_t)USART_ReceiveData(USART1);
					break;
				case 2:
					val_tmp = (uint8_t)USART_ReceiveData(USART2);
					break;
				case 3:
					val_tmp = (uint8_t)USART_ReceiveData(USART3);
					break;
		}
		return val_tmp;		
}
