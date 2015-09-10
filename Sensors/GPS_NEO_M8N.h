
#ifndef GPS_NEO_M8N_H_
#define GPS_NEO_M8N_H_

#include "System_Common.h"

//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 

//GPS������Ϣ
__packed typedef struct  
{										    
		uint8_t num;		//���Ǳ��
		uint8_t eledeg;	//��������
		uint16_t azideg;	//���Ƿ�λ��
		uint8_t sn;		//�����		   
}nmea_slmsg_gps; 

//GLONASS������Ϣ
__packed typedef struct  
{										    
		uint8_t num;		//���Ǳ��
		uint8_t eledeg;	//��������
		uint16_t azideg;	//���Ƿ�λ��
		uint8_t sn;		//�����		   
}nmea_slmsg_glonass; 


//UTCʱ����Ϣ(GNSS)
__packed typedef struct  
{										    
		uint16_t year;	//���
		uint8_t month;	//�·�
		uint8_t date;	//����
		uint8_t hour; 	//Сʱ
		uint8_t min; 	//����
		uint8_t sec; 	//����
}nmea_utc_time;  

//NMEA-0183Э�����������Ϣ�ṹ��
__packed typedef struct  
{	
		//GPGSV
		uint8_t svnum_gps;				         //GPS�ɼ���������
		nmea_slmsg_gps slmsg_gps[21];	     //GPS�ɼ�������Ϣ�����21�����ǣ�4��*6�Σ�
		//GLGSV
		uint8_t svnum_glonass;				     //GLONASS�ɼ���������
		nmea_slmsg_glonass slmsg_glonass[24];//GLONASS�ɼ�������Ϣ�����24�����ǣ�4��*6�Σ�

		//RMC
		nmea_utc_time utc;			         //UTCʱ��
		uint32_t latitude;				       //γ�� ������100000��,ʵ��Ҫ����100000
		uint8_t nshemi;					         //��γ/��γ,N:��γ;S:��γ				  
		uint32_t longitude;			         //���� ������100000��,ʵ��Ҫ����100000
		uint8_t ewhemi;					         //����/����,E:����;W:����
		//GGA
		uint8_t gps_State;			             //(����:��λ����)GPS״̬:0��δ��λ��1���ǲ�ֶ�λ��2����ֶ�λ��6�����ڹ���.
		uint8_t posslnum;	               //(����:�ɼ�������)���ڶ�λ��������,��12.
		int32_t altitude;			 	         //���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
		//GSA
		uint8_t fixmode;					       //��λ����:1��û�ж�λ��2��2D��λ��3��3D��λ
		uint8_t possl[24];				       //���ڶ�λ�����Ǳ�ţ��������ͣ�
		uint16_t pdop;					         //λ�þ������� 0~5000,��Ӧʵ��ֵ0~50.0
		uint16_t hdop;					         //ˮƽ�������� 0~5000,��Ӧʵ��ֵ0~50.0
		uint16_t vdop;					         //��ֱ�������� 0~5000,��Ӧʵ��ֵ0~50.0 
		//VTG
		uint16_t course_earth;                    //?????????????(0-359?)
    uint16_t course_mag;                      //?????????????(0-359?)
		uint16_t speed;					         //��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ
}nmea_msg; 

extern nmea_msg g_gps; 			 //GPS��Ϣ

//ע�⣺GP/GL/GN/BD: GPS / GLONASS / ��ϵͳ���϶�λ / ����

//����GPGSV��Ϣ(�ɼ�������Ϣ) (GPS)     ����Σ�
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);
//����GLGSV��Ϣ(�ɼ�������Ϣ) (GLONASS) ����Σ�
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);

//����GNGGA��Ϣ(GPS��λ��Ϣ) (GNSS) ��һ�Σ�
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);

//����GNGSA��Ϣ(��ǰ������Ϣ) (GNSS)    ����Σ�
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf);

//����GNRMC��Ϣ(�Ƽ���λ��Ϣ) (GNSS) ��һ�Σ�
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);

//����GNVTG��Ϣ(�����ٶ���Ϣ) (GNSS) ��һ�Σ�
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf);

//��ȡNMEA-0183��Ϣ(����������6���Ӻ���)
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf);

void Get_NED_Pos(float lat, float lon, float high, float NED_pos_TMP[3]);


#endif
