
//GPS_NEO_M8Nģ��Ӧ���ļ�

#include "GPS_NEO_M8N.h"

//-------------------------------------------------------
//-------------------------------------------------------
//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
		uint8_t *p=buf;
		while(cx)
		{		 
				if(*buf == '*' || *buf < ' ' || *buf > 'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
				if(*buf == ',')cx--;
				buf++;
		}
		return buf - p;	 
}

//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m, uint8_t n)
{
		uint32_t result = 1;	 
		while(n--)result *= m;    
		return result;
}

//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf, uint8_t *dx)
{
		uint8_t *p = buf;
		uint32_t ires = 0, fres = 0;
		uint8_t ilen = 0, flen = 0, i;
		uint8_t mask = 0;
		int res;
		while(1) //�õ�������С���ĳ���
		{
				if(*p == '-'){mask|=0X02;p++;}//�Ǹ���
				if(*p == ','||(*p=='*'))break;//����������
				if(*p == '.'){mask|=0X01;p++;}//����С������
				else if(*p > '9' || (*p < '0'))	//�зǷ��ַ�
				{	
						ilen=0;
						flen=0;
						break;
				}	
				if(mask&0X01)flen++;
				else ilen++;
				p++;
		}
		if(mask&0X02)buf++;	//ȥ������
		for(i=0;i<ilen;i++)	//�õ�������������
		{  
				ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
		}
		if(flen>5)flen=5;	//���ȡ5λС��
		*dx=flen;	 		//С����λ��
		for(i=0;i<flen;i++)	//�õ�С����������
		{  
				fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
		} 
		res=ires*NMEA_Pow(10,flen)+fres;
		if(mask&0X02)res=-res;		   
		return res;
}


//-------------------------------------------------------
//-------------------------------------------------------
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;
		uint8_t len,i,j,slx=0;
		uint8_t posx;   	 
		p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");   //strstr:�����ַ���$GPGSV��p�е�һ�γ��ֵ�λ��
		len=p1[7]-'0';								//�õ�GPGSV��������������
		posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
		if(posx!=0XFF)gpsx->svnum_gps=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)//һ����len��GPGSV����
		{	 
				p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
				for(j=0;j<4;j++)
				{	  
						posx=NMEA_Comma_Pos(p1,4+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
						else break; 
						posx=NMEA_Comma_Pos(p1,5+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
						else break;
						posx=NMEA_Comma_Pos(p1,6+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
						else break; 
						posx=NMEA_Comma_Pos(p1,7+j*4);
						if(posx!=0XFF)gpsx->slmsg_gps[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
						else break;
						slx++;	   
				}   
				p=p1+1;//�л�����һ��GPGSV��Ϣ
		}   
}
//����GLGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GLGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;
		uint8_t len,i,j,slx=0;
		uint8_t posx;   	 
		p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GLGSV");   //strstr:�����ַ���$GLGSV��p�е�һ�γ��ֵ�λ��
		len=p1[7]-'0';								//�õ�GLGSV������
		posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
		if(posx!=0XFF)gpsx->svnum_glonass=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)
		{	 
				p1=(uint8_t*)strstr((const char *)p,"$GLGSV");  
				for(j=0;j<4;j++)
				{	  
						posx=NMEA_Comma_Pos(p1,4+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
						else break; 
						posx=NMEA_Comma_Pos(p1,5+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
						else break;
						posx=NMEA_Comma_Pos(p1,6+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
						else break; 
						posx=NMEA_Comma_Pos(p1,7+j*4);
						if(posx!=0XFF)gpsx->slmsg_glonass[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
						else break;
						slx++;	   
				}   
				p=p1+1;//�л�����һ��GLGSV��Ϣ
		}   
}


//����GNGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;    
		p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");             //�����ַ���$GNGGA��p�е�һ�γ��ֵ�λ��
		posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
		if(posx!=0XFF)gpsx->gps_State=NMEA_Str2num(p1+posx,&dx);	
		posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
		if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
		posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
		if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}

//����GNGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p,*p1,dx;			 
		uint8_t posx; 
		uint8_t i;
    
    p=buf;
		p1=(uint8_t*)strstr((const char *)p,"$GNGSA");             //�����ַ���$GNGSA��p�е�һ�γ��ֵ�λ��
		posx=NMEA_Comma_Pos(p1,2);							  //�õ���λ����
		if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);
    
    //ͨ����2��GNGSA���ݶ�
    for(i=0;i<12;i++)   //�õ���λ���Ǳ��(һ��GNGSA���������12������)
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }
		p=p1+1;             //�л�����һ��GNGSA��Ϣ
		p1=(uint8_t*)strstr((const char *)p,"$GNGSA");  
    for(i=0;i<12;i++)  //�õ���λ���Ǳ��(һ��GNGSA���������12������)
    {
        posx=NMEA_Comma_Pos(p1,3+i);					 
        if(posx!=0XFF)gpsx->possl[i+12]=NMEA_Str2num(p1+posx,&dx);
        else break; 
    }
        
		posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
		if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
		posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
		if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
		posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
		if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}

//����GNRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNRMC_Analysis(nmea_msg *gpsx, uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;     
		uint32_t temp;	   
		float rs;  
		p1=(uint8_t*)strstr((const char *)buf,"GNRMC");              //"$GNRMC",������&��GNRMC�ֿ������,��ֻ�ж�GNRMC.
		posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
				gpsx->utc.hour=temp/10000 + 8;                      // + 8:ת��Ϊ����ʱ��
				gpsx->utc.min=(temp/100)%100;
				gpsx->utc.sec=temp%100;	 	 
		}	
		posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx);		 	 
				gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
				rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
				gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 				
		}
		posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
		if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
		posx=NMEA_Comma_Pos(p1,5);								//�õ�����
		if(posx!=0XFF)
		{												  
				temp=NMEA_Str2num(p1+posx,&dx);		 	 
				gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
				rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
				gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
		}
		posx=NMEA_Comma_Pos(p1,6);								//������������
		if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
		posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
		if(posx!=0XFF)
		{
				temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
				gpsx->utc.date=temp/10000;
				gpsx->utc.month=(temp/100)%100;
				gpsx->utc.year=2000+temp%100;	 	 
		} 
}

//����GNVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
#if 0
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		uint8_t *p1,dx;			 
		uint8_t posx;    
		p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");				//�����ַ���$GNVTG��p�е�һ�γ��ֵ�λ��		 
		posx=NMEA_Comma_Pos(p1,7);								//�õ���������
		if(posx!=0XFF)
		{
			gpsx->speed=NMEA_Str2num(p1+posx,&dx);
			if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
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
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
		NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����(GPS�ɼ�����)
		NMEA_GLGSV_Analysis(gpsx,buf);  //GLGSV����(GLONASS�ɼ�����)
		NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA����(GNSS��λ��Ϣ)	
		NMEA_GNGSA_Analysis(gpsx,buf);	//GNGSA����(GNSS��ǰ����)
		NMEA_GNRMC_Analysis(gpsx,buf);	//GNRMC����(GNSS�Ƽ���λ)
		NMEA_GNVTG_Analysis(gpsx,buf);	//GNVTG����(GNSS�ٶ���Ϣ)
}

//��WGS-84����ϵת��ΪCH-1903����ϵ
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


