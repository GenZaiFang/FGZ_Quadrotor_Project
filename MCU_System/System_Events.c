#include "System_All_Head.h"

#define UpTIMES        60
static uint16_t times = 0;
static uint16_t Ctimes = 0;
static float cTmp[2];
static float last_altitude[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t ultTimes = 0;

static uint8_t is_first_sample_heading = true;
static uint16_t sample_heading_times = 0;


void SysTick_Handler(void)
{	
		HomeLocationPrepare();
				
		if(Revise_MPU6050_GyroVal() && Global_HOME_Check_NONE)					
		{			
			
#if 1			
				if(is_first_sample_heading)
				{
						Global_Heading_MAX_Ref = Global_Heading_MIN_Ref = Global_Heading_Ref;
						is_first_sample_heading = false;
				}
				else
				{
						if(Global_Heading_MAX_Ref < Global_Heading_Ref)
						{
								Global_Heading_MAX_Ref = Global_Heading_Ref;
						}
						else if(Global_Heading_MIN_Ref > Global_Heading_Ref)
						{
								Global_Heading_MIN_Ref = Global_Heading_Ref;
						}
				}
				
				//sample_heading_times++;
				
				if(sample_heading_times > 100)
				{
						sample_heading_times = 0;
						Global_HOME_Check_NONE = false;
				}
#endif	
				
				times++;
				ultTimes++;
				if(ultTimes == 2)
				{
						My_HC_SR04_Start();
						ultTimes = 0;
				}
				
				if(times >= 10)
				{
						times = 0;						
						LED1_BLINK;
				}						

				Get_AHRS_Sensor_Val();						

#if 1				
				Global_hgtRef = 0;
				Global_Altitude_Sensor.altitude = MS5611_Get_Altitude((float)Global_hgtRef);		
				Global_Altitude_Sensor.altitude = (Global_Altitude_Sensor.altitude + last_altitude[0] + last_altitude[1] + 
				last_altitude[2] + last_altitude[3] + last_altitude[4] + 
				last_altitude[5] + last_altitude[6] + last_altitude[7] +
				last_altitude[8] + last_altitude[9] + last_altitude[10]) / 12;
				
				if(is_first_sample_heading)
				{
						Global_Heading_MAX_Ref = Global_Heading_MIN_Ref = Global_Altitude_Sensor.altitude;
						is_first_sample_heading = false;
				}
				else
				{
						if(Global_Heading_MAX_Ref < Global_Altitude_Sensor.altitude)
						{
								Global_Heading_MAX_Ref = Global_Altitude_Sensor.altitude;
						}
						else if(Global_Heading_MIN_Ref > Global_Altitude_Sensor.altitude)
						{
								Global_Heading_MIN_Ref = Global_Altitude_Sensor.altitude;
						}
				}
				
				last_altitude[10] = last_altitude[9];
				last_altitude[9] = last_altitude[8];
				last_altitude[8] = last_altitude[7];
				last_altitude[7] = last_altitude[6];
				last_altitude[6] = last_altitude[5];
				last_altitude[5] = last_altitude[4];
				last_altitude[4] = last_altitude[3];
				last_altitude[3] = last_altitude[2];
				last_altitude[2] = last_altitude[1];
				last_altitude[1] = last_altitude[0];
				last_altitude[0] = Global_Altitude_Sensor.altitude;
				Global_Altitude_Sensor.updated = true;		
				
				
#endif								
				
				GPS_Dates_Deal();
								
				EKF_INS_GPS_Run();				
				
				if(Global_RC[2] <= RC_CH3_MIN && Global_RC[2] > 0 && Global_RC[1] > 0 && Global_RC[1] < RC_CH2_RDN)
				{
						Ctimes++;
						if(Ctimes >= UpTIMES)
						{
								Global_RC_UNLOCK = true;									
								LED3_ON;								
								Ctimes = 0;
						}						
				}
				else if(Global_RC[2] <= RC_CH3_MIN && Global_RC[2] > 0 && Global_RC[1] > RC_CH2_MAX)
				{
						Ctimes++;
						if(Ctimes > UpTIMES)
						{
								Global_RC_UNLOCK = false;
								LED3_OFF;										
								Ctimes = 0;
						}
				}
				
				if(Global_RC_UNLOCK)
				{
						if(Global_RC[0] < RC_CH1_MIN && Global_RC[0] > 0)
						{
								LED5_OFF;
							
								Aircraft.throttle = MIN_Throttle + (Global_RC[2] - RC_CH3_MIN) * Throttle_k;	
								
								cTmp[0] = -1 * (Global_RC[3] - RC_CH4_MID) * Expect_Roll_Deg_k;				
								if(cTmp[0] <= 0.1f && cTmp[0] >= -0.1f)
								{
										cTmp[0] = 0;
								}
								
								cTmp[1] = (Global_RC[4] - RC_CH5_MID) * Expect_Pitch_Deg_k;
								if(cTmp[1] <= 0.1f && cTmp[1] >= -0.1f)
								{
										cTmp[1] = 0;
								}
								
								Aircraft.expect_Wz = (Global_RC[1] - RC_CH2_MID) * W_Zrate_Deg_k;
								if(Aircraft.expect_Wz <= 0.001f && Aircraft.expect_Wz >= -0.001f)
								{
										Aircraft.expect_Wz = 0;
								}
								
								
#if defined Control_UAV_Heading
								
								Aircraft.expect_euler[0] = cTmp[0] * cos(Global_Now_Euler[2]) + cTmp[1] * sin(Global_Now_Euler[2]);
								Aircraft.expect_euler[1] = -1 * cTmp[0] * sin(Global_Now_Euler[2]) + cTmp[1] * cos(Global_Now_Euler[2]);
								Aircraft.expect_euler[2] += Aircraft.expect_Wz;
								
#else							
								
								Aircraft.expect_euler[0] = cTmp[0];
								Aircraft.expect_euler[1] = cTmp[1];
								
#endif								
																
								if(abs(Global_Now_Euler[0]) < ProtectAngle && abs(Global_Now_Euler[1]) < ProtectAngle)
								{
										if(Aircraft.throttle > MIN_Throttle)
										{								
												Aircraft.expect_euler[2] += Aircraft.expect_Wz;
												Attitude_Control();
										}
										else
										{
												Tim1_PWM_Output(MIN_Throttle, MIN_Throttle, MIN_Throttle, MIN_Throttle);
												PID_RESET();
										}
								}
								else
								{
										Global_RC_UNLOCK = false;
										Tim1_PWM_Output(Close_Throttle, Close_Throttle, Close_Throttle, Close_Throttle);
										PID_RESET();
										LED3_OFF;	
								}
						}
						else if(Global_RC[0] > RC_CH1_MAX)//GPS_MODE
						{
								LED5_ON;
								if(Global_GPS_Health)
								{
										Aircraft.expect_vel[0] = (Global_RC[4] - RC_CH5_MID) * Expect_Vn;
										if(Aircraft.expect_vel[0] <= 0.002f && Aircraft.expect_vel[0] >= -0.002f)
										{
												Aircraft.expect_vel[0] = 0;
										}
										
										
										Aircraft.expect_vel[1] = -1 * (Global_RC[3] - RC_CH4_MID) * Expect_Ve;										
										if(Aircraft.expect_vel[1] <= 0.002f && Aircraft.expect_vel[1] >= -0.002f)
										{
												Aircraft.expect_vel[1] = 0;
										}
										
										
										Aircraft.expect_vel[2] = (Global_RC[1] - RC_CH2_MID) * Expect_Vd;
										if(Aircraft.expect_vel[2] <= 0.001f && Aircraft.expect_vel[2] >= -0.001f)
										{
												Aircraft.expect_vel[2] = 0;
										}

										
										Aircraft.expect_position[0] += Aircraft.expect_vel[0] * 1.0f;
										Aircraft.expect_position[1] += Aircraft.expect_vel[1] * 1.0f;
										Aircraft.expect_position[2] += Aircraft.expect_vel[2];
										
										Aircraft.posError[0] = Aircraft.expect_position[0] - (Nav.Pos[0] - Aircraft.ref_pos[0]);
										Aircraft.posError[1] = Aircraft.expect_position[1] - (Nav.Pos[1] - Aircraft.ref_pos[1]);
										Aircraft.posError[2] = Aircraft.expect_position[2] - (Nav.Pos[2] - Aircraft.ref_pos[2]);
									
										Aircraft.sum_posError[1] += Aircraft.posError[1];
										Aircraft.expect_euler[0] = Aircraft.posError[1] * 4.5f + Aircraft.sum_posError[1] * 0.0f - (Nav.Vel[1]) * 4.5f;										

										Aircraft.sum_posError[0] += Aircraft.posError[0];
										//Aircraft.expect_euler[1] = Aircraft.posError[0] * 4.5f + Aircraft.sum_posError[0] * 0.0f - (Nav.Vel[0]) * 4.5f;										
																														
										Aircraft.throttle = MIN_Throttle + (Global_RC[2] - RC_CH3_MIN) * Throttle_k;
#if 1
										if(abs(Global_Now_Euler[0]) < ProtectAngle && abs(Global_Now_Euler[1]) < ProtectAngle)
										{										
												if(Aircraft.throttle > MIN_Throttle)
												{																																					
														Attitude_Control();
												}
												else
												{
														Aircraft.ref_pos[0] = Nav.Pos[0];
														Aircraft.ref_pos[1] = Nav.Pos[1];
														Aircraft.ref_pos[2] = Nav.Pos[2];
														Nav.Vel[1] = 0;
														Tim1_PWM_Output(MIN_Throttle, MIN_Throttle, MIN_Throttle, MIN_Throttle);													
														PID_RESET();
												}
										}
										else
										{
												Global_RC_UNLOCK = false;
												Tim1_PWM_Output(Close_Throttle, Close_Throttle, Close_Throttle, Close_Throttle);												
												PID_RESET();
												LED3_OFF;	
										}
#endif										
								}
								else
								{
										Tim1_PWM_Output(Close_Throttle, Close_Throttle, Close_Throttle, Close_Throttle);
										PID_RESET();
								}
						}
				}
				else
				{
						LED5_OFF;						
						Tim1_PWM_Output(Close_Throttle, Close_Throttle, Close_Throttle, Close_Throttle);
						PID_RESET();
				}
		}
}

void TIM3_IRQHandler(void)
{
		static uint16_t CNT_CH2=0, CNT_CH3=0, CNT_CH4=0;     //各通道的计数器值
		static uint8_t RISE_CH2=0,RISE_CH3=0,RISE_CH4=0;		
		if(TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
		{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC2 | TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM3, TIM_FLAG_CC2 | TIM_FLAG_Update);

				if(RISE_CH2 == 0)
				{
						TIM_SetCounter(TIM3, 0);//计数器清零
						TIM3->CCER |= 1<<5;     //下降沿捕捉
						
						RISE_CH2=1;
				}
				else if(RISE_CH2 == 1)
				{
						CNT_CH2 = TIM_GetCapture2(TIM3);
						
						TIM3->CCER &= ~(1<<5);  //上升沿捕捉

						RISE_CH2=0;
				}
		}
		if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
		{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC3 | TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM3, TIM_FLAG_CC3 | TIM_FLAG_Update);

				if(RISE_CH3 == 0)
				{
						TIM_SetCounter(TIM3, 0);//计数器清零
						TIM3->CCER |= 1<<9;     //下降沿捕捉
						
						RISE_CH3=1;
				}
				else if(RISE_CH3 == 1)
				{
						CNT_CH3 = TIM_GetCapture3(TIM3);
						
						TIM3->CCER &= ~(1<<9); //上升沿捕捉

						RISE_CH3=0;
				}
		}

		if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
		{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC4 | TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM3, TIM_FLAG_CC4 | TIM_FLAG_Update);

				if(RISE_CH4 == 0)
				{
						TIM_SetCounter(TIM3, 0);//计数器清零
						TIM3->CCER |= 1<<13;    //下降沿捕捉
						
						RISE_CH4=1;
				}
				else if(RISE_CH4 == 1)
				{
						CNT_CH4 = TIM_GetCapture4(TIM3);
						
						TIM3->CCER &= ~(1<<13); //上升沿捕捉

						RISE_CH4=0;
				}
		}
		Global_RC[5]=CNT_CH2;
		Global_RC[4]=CNT_CH3;
		Global_RC[3]=CNT_CH4;
}

void TIM4_IRQHandler(void)
{
		static uint16_t CNT_CH2=0,CNT_CH3=0,CNT_CH4=0;     //各通道的计数器值
		static uint8_t RISE_CH2=0,RISE_CH3=0,RISE_CH4=0;

		if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
		{
				TIM_ClearITPendingBit(TIM4, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM4, TIM_FLAG_CC2|TIM_FLAG_Update);

				if(RISE_CH2 == 0)
				{
						TIM_SetCounter(TIM4, 0);//计数器清零
						TIM4->CCER |= 1<<5;     //下降沿捕捉
						
						RISE_CH2=1;
				}
				else if(RISE_CH2 == 1)
				{
						CNT_CH2 = TIM_GetCapture2(TIM4);
						
						TIM4->CCER &= ~(1<<5); //上升沿捕捉

						RISE_CH2=0;
				}
		}


		if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
		{
				TIM_ClearITPendingBit(TIM4, TIM_IT_CC3|TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM4, TIM_FLAG_CC3|TIM_FLAG_Update);

				if(RISE_CH3 == 0)
				{
						TIM_SetCounter(TIM4, 0);//计数器清零
						TIM4->CCER |= 1<<9;     //下降沿捕捉
						
						RISE_CH3=1;
				}
				else if(RISE_CH3 == 1)
				{
						CNT_CH3 = TIM_GetCapture3(TIM4);
						
						TIM4->CCER &= ~(1<<9); //上升沿捕捉

						RISE_CH3=0;
				}
		}

		if(TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
		{
				TIM_ClearITPendingBit(TIM4, TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
				TIM_ClearFlag(TIM4, TIM_FLAG_CC4|TIM_FLAG_Update);

				if(RISE_CH4 == 0)
				{
						TIM_SetCounter(TIM4, 0);//计数器清零
						TIM4->CCER |= 1<<13;    //下降沿捕捉
						
						RISE_CH4=1;
				}
				else if(RISE_CH4 == 1)
				{
						CNT_CH4 = TIM_GetCapture4(TIM4);
						
						TIM4->CCER &= ~(1<<13); //上升沿捕捉

						RISE_CH4=0;
				}
		}

		Global_RC[2]=CNT_CH2;
		Global_RC[1]=CNT_CH3;
		Global_RC[0]=CNT_CH4;			
}

void USART1_IRQHandler(void)
{
		//中断接收串口1数据
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{			
				USART1->SR;  
				USART1->DR;
				USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			
#ifdef USART1_DMA_Transfer			
				DMA_Cmd(DMA2_Stream5, DISABLE);										
				DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
/*********************write your code there********************/
				Global_U_Height = Global_DMA2_Stream5_Rx_Buf[0] * 256 + Global_DMA2_Stream5_Rx_Buf[1];
/**************************************************************/			
				DMA_SetCurrDataCounter(DMA2_Stream5, DMA_RX_LEN1);
				DMA_Cmd(DMA2_Stream5, ENABLE);
#endif			
		}		
}

void USART2_IRQHandler(void)
{
		//中断接收串口2数据
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
				USART2->SR;  
				USART2->DR;
				USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			
#ifdef USART2_DMA_Transfer
				DMA_Cmd(DMA1_Stream5, DISABLE);							
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
/*********************write your code there********************/
/**************************************************************/				
				DMA_SetCurrDataCounter(DMA1_Stream5, DMA_RX_LEN2);				
				DMA_Cmd(DMA1_Stream5, ENABLE);
#endif
		}
}

void USART3_IRQHandler(void)
{
		//中断接收串口3数据
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
				USART3->SR;  
				USART3->DR;
				USART_ClearITPendingBit(USART3, USART_IT_RXNE);
			
#ifdef USART3_DMA_Transfer
				DMA_Cmd(DMA1_Stream1, DISABLE);							
				DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
				DMA_SetCurrDataCounter(DMA1_Stream1, DMA_RX_LEN3);
/*********************write your code there********************/								
/**************************************************************/							
				DMA_Cmd(DMA1_Stream1, ENABLE);
#endif
		}
}
