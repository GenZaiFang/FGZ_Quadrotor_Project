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
		//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);           
	
		//DMA�����ж�����
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
		
		//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

		//DMAͨ������
		DMA_DeInit(DMA2_Stream5);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//�����ַ
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
		
		//�ڴ��ַ
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA2_Stream5_Rx_Buf;
		
		//DMA���䷽��
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//����DMA�ڴ���ʱ����������
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN1;
		
		//����DMA�������ģʽ��ֻ��һ������
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//����DMA�ڴ����ģʽ
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//���������ֳ�
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//�ڴ������ֳ�
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//����DMA����ģʽ
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//����DMA���ȼ�
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//ָ��FIFOģʽ��ֱ��ģʽ������ָ����:��ʹ��FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//ָ��FIFO����ֵˮƽ
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//ָ����Burstת�������ڴ洫��
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//ָ����Burstת��������Χת��
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//��ʼ��DMAͨ��
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);

		//ʹ��DMAͨ��
		DMA_Cmd(DMA2_Stream5, ENABLE);
		
		//����DMA��ʽ����
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);		
#endif

#ifdef USART2_DMA_Transfer			
		//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);           
		
		//DMA�����ж�����
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure); 
			
		//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		//DMAͨ������
		DMA_DeInit(DMA1_Stream5);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//�����ַ
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
		
		//�ڴ��ַ
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA1_Stream5_Rx_Buf;
		
		//DMA���䷽��
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//����DMA�ڴ���ʱ����������
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN2;
		
		//����DMA�������ģʽ��ֻ��һ������
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//����DMA�ڴ����ģʽ
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//���������ֳ�
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//�ڴ������ֳ�
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//����DMA����ģʽ
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//����DMA���ȼ�
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//ָ��FIFOģʽ��ֱ��ģʽ������ָ����:��ʹ��FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//ָ��FIFO����ֵˮƽ
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//ָ����Burstת�������ڴ洫��
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//ָ����Burstת��������Χת��
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//��ʼ��DMAͨ��
		DMA_Init(DMA1_Stream5, &DMA_InitStructure);

		//ʹ��DMAͨ��
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		//����DMA��ʽ����
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
#endif

#ifdef USART3_DMA_Transfer
//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);           
		
		//DMA�����ж�����
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure); 
			
		//����DMAʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		//DMAͨ������
		DMA_DeInit(DMA1_Stream1);  
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			
		//�����ַ
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
		
		//�ڴ��ַ
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Global_DMA1_Stream1_Rx_Buf;
		
		//DMA���䷽��
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		
		//����DMA�ڴ���ʱ����������
		DMA_InitStructure.DMA_BufferSize = DMA_RX_LEN3;
		
		//����DMA�������ģʽ��ֻ��һ������
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		
		//����DMA�ڴ����ģʽ
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		
		//���������ֳ�
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		//�ڴ������ֳ�
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		
		//����DMA����ģʽ
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		//����DMA���ȼ�
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		
		//ָ��FIFOģʽ��ֱ��ģʽ������ָ����:��ʹ��FIFO
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		
		//ָ��FIFO����ֵˮƽ
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		
		//ָ����Burstת�������ڴ洫��
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		
		//ָ����Burstת��������Χת��
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
		//��ʼ��DMAͨ��
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);

		//ʹ��DMAͨ��
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		//����DMA��ʽ����
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
#endif		
}

// ��ʼ��USART1..3; baudrate:���ò�����; INT_EN_FLAG:�Ƿ�ʹ�ܴ��ڽ����ж�
void My_STM32_USART_init(uint32_t *baudrate, uint8_t *INT_EN_FLAG)
{
    GPIO_InitTypeDef  GPIO_InitStructure_Usart;                    //IO�ڸ���Ϊ����
    USART_InitTypeDef USART_InitStructure;                         //���ڳ�ʼ��
    NVIC_InitTypeDef  NVIC_InitStructure;                          //�����жϳ�ʼ��		

/***************************************����USART1*********************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   //P6��P7
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //��ʼ��Ϊ����ģʽ
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //�������
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //�˿�����50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //����
    GPIO_Init(GPIOB, &GPIO_InitStructure_Usart);                   //PB��ʼ��
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);      //TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);      //RX

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = baudrate[0];               //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8λ����ģʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1λֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ���������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //˫��ģʽ
    USART_Init(USART1,&USART_InitStructure);
    USART_Cmd(USART1,ENABLE);                                       //ʹ��USART1
    USART_ClearFlag(USART1, USART_FLAG_TXE);                        //�巢����ɱ�־λ

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;               //ָ��USART1_IRQnͨ��ʹ�ܻ�ʧ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //��ռʽ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;              //��Ӧʽ���ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��USART1_IRQn�ж�
    NVIC_Init(&NVIC_InitStructure);                                 //�ж���������
    USART_ITConfig(USART1, USART_IT_RXNE, (INT_EN_FLAG[0] == 1 ? ENABLE : DISABLE));//����flagֵ�����Ƿ�ʹ��USART1���ڽ����ж�			

/***************************************����USART2*********************************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;   //PD5��PD6
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //��ʼ��Ϊ����ģʽ
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //�������
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //�˿�����50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //����
    GPIO_Init(GPIOD, &GPIO_InitStructure_Usart);                   //PC��ʼ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);      //TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);      //RX

    USART_DeInit(USART2);
    USART_InitStructure.USART_BaudRate = baudrate[1];                //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8λ����ģʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1λֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ���������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //˫��ģʽ
    USART_Init(USART2,&USART_InitStructure);
    USART_Cmd(USART2,ENABLE);                                       //ʹ��USART2
    USART_ClearFlag(USART2, USART_FLAG_TXE);                        //�巢����ɱ�־λ

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //ָ��USART2_IRQnͨ��ʹ�ܻ�ʧ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //��ռʽ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //��Ӧʽ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��USART2_IRQn�ж�
    NVIC_Init(&NVIC_InitStructure);                                 //�ж���������
    USART_ITConfig(USART2, USART_IT_RXNE, (INT_EN_FLAG[1] == 1 ? ENABLE : DISABLE));//����flagֵ�����Ƿ�ʹ��USART2���ڽ����ж�

/***************************************����USART3*********************************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure_Usart.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //P10��P11
    GPIO_InitStructure_Usart.GPIO_Mode = GPIO_Mode_AF;             //��ʼ��Ϊ����ģʽ
    GPIO_InitStructure_Usart.GPIO_OType = GPIO_OType_PP;           //�������
    GPIO_InitStructure_Usart.GPIO_Speed = GPIO_Speed_50MHz;        //�˿�����50MHz
    GPIO_InitStructure_Usart.GPIO_PuPd = GPIO_PuPd_UP;             //����
    GPIO_Init(GPIOC, &GPIO_InitStructure_Usart);                   //PC��ʼ��
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);     //TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);     //RX

    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate = baudrate[2];               //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8λ����ģʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1λֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;             //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ���������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //˫��ģʽ
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);                                      //ʹ��USART3
    USART_ClearFlag(USART3, USART_FLAG_TXE);                        //�巢����ɱ�־λ

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;               //ָ��USART3_IRQnͨ��ʹ�ܻ�ʧ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //��ռʽ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //��Ӧʽ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ��USART3_IRQn�ж�
    NVIC_Init(&NVIC_InitStructure);                                 //�ж���������
    USART_ITConfig(USART3, USART_IT_RXNE, (INT_EN_FLAG[2] == 1 ? ENABLE : DISABLE));//����flagֵ�����Ƿ�ʹ��USART3���ڽ����ж�

/***************************************���ý���*********************************************/
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
