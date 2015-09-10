
#ifndef GPS_NEO_M8N_H_
#define GPS_NEO_M8N_H_

#include "System_Common.h"

//GPS NMEA-0183协议重要参数结构体定义 

//GPS卫星信息
__packed typedef struct  
{										    
		uint8_t num;		//卫星编号
		uint8_t eledeg;	//卫星仰角
		uint16_t azideg;	//卫星方位角
		uint8_t sn;		//信噪比		   
}nmea_slmsg_gps; 

//GLONASS卫星信息
__packed typedef struct  
{										    
		uint8_t num;		//卫星编号
		uint8_t eledeg;	//卫星仰角
		uint16_t azideg;	//卫星方位角
		uint8_t sn;		//信噪比		   
}nmea_slmsg_glonass; 


//UTC时间信息(GNSS)
__packed typedef struct  
{										    
		uint16_t year;	//年份
		uint8_t month;	//月份
		uint8_t date;	//日期
		uint8_t hour; 	//小时
		uint8_t min; 	//分钟
		uint8_t sec; 	//秒钟
}nmea_utc_time;  

//NMEA-0183协议解析数据信息结构体
__packed typedef struct  
{	
		//GPGSV
		uint8_t svnum_gps;				         //GPS可见卫星总数
		nmea_slmsg_gps slmsg_gps[21];	     //GPS可见卫星信息，最多21颗卫星（4个*6段）
		//GLGSV
		uint8_t svnum_glonass;				     //GLONASS可见卫星总数
		nmea_slmsg_glonass slmsg_glonass[24];//GLONASS可见卫星信息，最多24颗卫星（4个*6段）

		//RMC
		nmea_utc_time utc;			         //UTC时间
		uint32_t latitude;				       //纬度 分扩大100000倍,实际要除以100000
		uint8_t nshemi;					         //北纬/南纬,N:北纬;S:南纬				  
		uint32_t longitude;			         //经度 分扩大100000倍,实际要除以100000
		uint8_t ewhemi;					         //东经/西经,E:东经;W:西经
		//GGA
		uint8_t gps_State;			             //(区别:定位类型)GPS状态:0：未定位；1：非差分定位；2：差分定位；6：正在估算.
		uint8_t posslnum;	               //(区别:可见卫星数)用于定位的卫星数,≤12.
		int32_t altitude;			 	         //海拔高度,放大了10倍,实际除以10.单位:0.1m	 
		//GSA
		uint8_t fixmode;					       //定位类型:1：没有定位；2：2D定位；3：3D定位
		uint8_t possl[24];				       //用于定位的卫星编号（所有类型）
		uint16_t pdop;					         //位置精度因子 0~5000,对应实际值0~50.0
		uint16_t hdop;					         //水平精度因子 0~5000,对应实际值0~50.0
		uint16_t vdop;					         //垂直精度因子 0~5000,对应实际值0~50.0 
		//VTG
		uint16_t course_earth;                    //?????????????(0-359?)
    uint16_t course_mag;                      //?????????????(0-359?)
		uint16_t speed;					         //地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
}nmea_msg; 

extern nmea_msg g_gps; 			 //GPS信息

//注意：GP/GL/GN/BD: GPS / GLONASS / 多系统联合定位 / 北斗

//分析GPGSV信息(可见卫星信息) (GPS)     （多段）
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);
//分析GLGSV信息(可见卫星信息) (GLONASS) （多段）
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);

//分析GNGGA信息(GPS定位信息) (GNSS) （一段）
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);

//分析GNGSA信息(当前卫星信息) (GNSS)    （多段）
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf);

//分析GNRMC信息(推荐定位信息) (GNSS) （一段）
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);

//分析GNVTG信息(地面速度信息) (GNSS) （一段）
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf);

//提取NMEA-0183信息(调用上面面6个子函数)
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf);

void Get_NED_Pos(float lat, float lon, float high, float NED_pos_TMP[3]);


#endif
