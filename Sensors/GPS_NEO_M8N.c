
//GPS_NEO_M8N模块应用文件

#include "GPS_NEO_M8N.h"

//-------------------------------------------------------
//-------------------------------------------------------
//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
		uint8_t *p=buf;
		while(cx)
		{		 
				if(*buf == '*' || *buf < ' ' || *buf > 'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
				if(*buf == ',')cx--;
				buf++;
		}
		return buf - p;	 
}

//m^n函数
//返回值:m^n次方.
uint32_t NMEA_Pow(uint8_t m, uint8_t n)
{
		uint32_t result = 1;	 
		while(n--)result *= m;    
		return result;
}

//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(uint8_t *buf, uint8_t *dx)
{
		uint8_t *p = buf;
		uint32_t ires = 0, fres = 0;
		uint8_t ilen = 0, flen = 0, i;
		uint8_t mask = 0;
		int res;
		while(1) //得到整数和小数的长度
		{
				if(*p == '-'){mask|=0X02;p++;}//是负数
				if(*p == ','||(*p=='*'))break;//遇到结束了
				if(*p == '.'){mask|=0X01;p++;}//遇到小数点了
				else if(*p > '9' || (*p < '0'))	//有非法字符
				{	
						ilen=0;
						flen=0;
						break;
				}	
				if(mask&0X01)flen++;
				else ilen++;
				p++;
		}
		if(mask&0X02)buf++;	//去掉负号
		for(i=0;i<ilen;i++)	//得到整数部分数据
		{  
				ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
		}
		if(flen>5)flen=5;	//最多取5位小数
		*dx=flen;	 		//小数点位数
		for(i=0;i<flen;i++)	//得到小数部分数据
		{  
				fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
		} 
		res=ires*NMEA_Pow(10,flen)+fres;
		if(mask&0X02)res=-res;		   
		return res;
}


//-------------------------------------------------------
//-------------------------------------------------------
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;
		uint8_t len,i,j,slx=0;
		uint8_t posx;   	 
		p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");   //strstr:搜索字符串$GPGSV在p中第一次出现的位置
		len=p1[7]-'0';								//得到GPGSV的条数（段数）
		posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
		if(posx!=0XFF)gpsx->svnum_gps=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)//一共有len段GPGSV数据
		{	 
				p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
				for(j=0;j<4;j++)
				{	  
						posx=NMEA_Comma_Pos(p1,4+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
						else break; 
						posx=NMEA_Comma_Pos(p1,5+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
						else break;
						posx=NMEA_Comma_Pos(p1,6+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
						else break; 
						posx=NMEA_Comma_Pos(p1,7+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
						else break;
						slx++;	   
				}   
				p=p1+1;//切换到下一个GPGSV信息
		}   
}
//分析GLGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;
		uint8_t len,i,j,slx=0;
		uint8_t posx;   	 
		p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GLGSV");   //strstr:搜索字符串$GLGSV在p中第一次出现的位置
		len=p1[7]-'0';								//得到GLGSV的条数
		posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
		if(posx!=0XFF)gpsx->svnum_glonass=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)
		{	 
				p1=(uint8_t*)strstr((const char *)p,"$GLGSV");  
				for(j=0;j<4;j++)
				{	  
						posx=NMEA_Comma_Pos(p1,4+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
						else break; 
						posx=NMEA_Comma_Pos(p1,5+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
						else break;
						posx=NMEA_Comma_Pos(p1,6+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
						else break; 
						posx=NMEA_Comma_Pos(p1,7+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
						else break;
						slx++;	   
				}   
				p=p1+1;//切换到下一个GLGSV信息
		}   
}


//分析GNGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;    
		p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");             //搜索字符串$GNGGA在p中第一次出现的位置
		posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
		if(posx!=0XFF)gpsx->gps_State=NMEA_Str2num(p1+posx,&dx);	
		posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
		if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
		posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
		if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}

//分析GNGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;			 
		uint8_t posx; 
		uint8_t i;
    
    p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GNGSA");             //搜索字符串$GNGSA在p中第一次出现的位置
		posx=NMEA_Comma_Pos(p1,2);							  //得到定位类型
		if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);
    
    //通常有2条GNGSA数据段
    for(i=0;i<12;i++)   //得到定位卫星编号(一条GNGSA语句最多包含12颗卫星)
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }
		p=p1+1;             //切换到下一个GNGSA信息
		p1=(uint8_t*)strstr((const char *)p,"$GNGSA");  
    for(i=0;i<12;i++)  //得到定位卫星编号(一条GNGSA语句最多包含12颗卫星)
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i+12]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }
        
		posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
		if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
		posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
		if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
		posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
		if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}

//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx, uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;     
		uint32_t temp;	   
		float rs;  
		p1=(uint8_t*)strstr((const char *)buf,"GNRMC");              //"$GNRMC",经常有&和GNRMC分开的情况,故只判断GNRMC.
		posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
				gpsx->utc.hour=temp/10000 + 8;                      // + 8:转换为北京时间
				gpsx->utc.min=(temp/100)%100;
				gpsx->utc.sec=temp%100;	 	 
		}	
		posx=NMEA_Comma_Pos(p1,3);								//得到纬度
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx);		 	 
				gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
				rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
				gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 				
		}
		posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
		if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
		posx=NMEA_Comma_Pos(p1,5);								//得到经度
		if(posx!=0XFF)
		{												  
				temp=NMEA_Str2num(p1+posx,&dx);		 	 
				gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
				rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
				gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
		}
		posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
		if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
		posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
				gpsx->utc.date=temp/10000;
				gpsx->utc.month=(temp/100)%100;
				gpsx->utc.year=2000+temp%100;	 	 
		} 
}

//分析GNVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
#if 0
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;    
		p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");				//搜索字符串$GNVTG在p中第一次出现的位置		 
		posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
		if(posx!=0XFF)
		{
			gpsx->speed=NMEA_Str2num(p1+posx,&dx);
			if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
		}
}  
#else
void NMEA_GNVTG_Analysis(nmea_msg *gpsx, uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;    
		p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");				//?????$GNVTG?p?????????		 
		posx=NMEA_Comma_Pos(p1,1);								
		if(posx!=0XFF)
		{
			gpsx->course_earth=NMEA_Str2num(p1+posx,&dx);       //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,3);								
		if(posx!=0XFF)
		{
			gpsx->course_mag=NMEA_Str2num(p1+posx,&dx);         //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,7);								//??????
		if(posx!=0XFF)
		{
			gpsx->speed=NMEA_Str2num(p1+posx,&dx);
			if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);             //????1000?(1??/?? -> 0.001??/??)
			else if(dx>3)gpsx->speed/=NMEA_Pow(10,dx-3);        //????1000?(1??/?? -> 0.001??/??)(??1000?)
		}
} 
#endif
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析(GPS可见卫星)
		NMEA_GLGSV_Analysis(gpsx,buf);  //GLGSV解析(GLONASS可见卫星)
		NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析(GNSS定位信息)	
		NMEA_GNGSA_Analysis(gpsx,buf);	//GNGSA解析(GNSS当前卫星)
		NMEA_GNRMC_Analysis(gpsx,buf);	//GNRMC解析(GNSS推荐定位)
		NMEA_GNVTG_Analysis(gpsx,buf);	//GNVTG解析(GNSS速度信息)
}

//将WGS-84坐标系转化为CH-1903坐标系
void Get_NED_Pos(float lat, float lon, float high, float NED_pos_TMP[3])
{
		float B, L;
		float Bt, Lt;
		B = lat * 3600;
		L = lon * 3600;
		Bt = (B - 169028.66f) / 10000;
		Lt = (L - 26782.5f) / 10000;
		NED_pos_TMP[0] = 200147.07f + (308807.95f * Bt) + (3745.25f * Lt * Lt) + (76.63f * Bt * Bt) - (194.56f * Lt * Lt * Bt) + (119.79f * Bt * Bt * Bt);
		NED_pos_TMP[1] = 600072.37f + (211455.93f * Lt) - (10938.51f * Lt * Bt) - (0.36f * Lt * Bt * Bt) - (44.54f * Lt * Lt * Lt);
		NED_pos_TMP[2] = (high - 49.55f) + (2.73f * Lt) + (6.94f * Bt);
}


