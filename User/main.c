#include "System_All_Head.h"

int main()
{
		My_STM32_SYSTEM_INIT();		
	
#if defined OLED_Dispaly_Yaw			
				oled_display_str(0x31, "Yaw:", 7, 0);
				oled_display_num(0x35, (int16_t)Global_Now_Euler[2], 4, 1, 0);
#endif	
		My_HC_SR04_Start();
		for(;;)
		{
				eCompassClibration();
				//My_HC_SR04_Get_Height(&Global_U_Height);
			
#if defined OLED_Dispaly_Yaw							
				oled_display_num(0x35, (int16_t)Global_Now_Euler[2], 4, 1, 0);
#endif
				
#if 1
		#if 1
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x0f, Global_Altitude_Sensor.altitude + Global_hgtRef);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, Global_Accel_Sensor.filtered.x);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, Nav.Vel[0]);
				My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, Global_Now_Euler[2]);
			  My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Now_Euler[1]);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x1f, Global_Show_Val[0]);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x2f, Global_Show_Val[1]);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, Global_Show_Val[2]);					
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x5f, Global_Show_Val[11]);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Show_Val[12]);				
		#else				
				#if 1		
					My_USART_send_MUX_Bytes_x(Serial_3, 0x1f, Global_Mag_Val[0]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x2f, Global_Mag_Val[1]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, Global_Mag_Val[2]);
				#else
					My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, Global_Accel_Sensor.filtered.x);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x5f, Global_Accel_Sensor.filtered.y);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Accel_Sensor.filtered.z);	
				#endif
				#if 1
					My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, Global_Show_Val[5]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x5f, Global_Show_Val[6]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Show_Val[7]);
				#else
					My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, Global_Now_Euler[0]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x5f, Global_Now_Euler[1]);
					My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Now_Euler[2]);
				#endif
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x3f, Global_Heading_MIN_Ref);
			  //My_USART_send_MUX_Bytes_x(Serial_3, 0x6f, Global_Accel_Sensor.filtered.z);
				//My_USART_send_MUX_Bytes_x(Serial_3, 0x4f, Global_Heading_MAX_Ref - Global_Heading_MIN_Ref);
				
		#endif			
#endif
		}	
}
