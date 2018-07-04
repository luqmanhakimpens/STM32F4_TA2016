#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"
#include "my_delay.h"
#include "peripheral.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_hd44780.h"
#include <stdio.h>
#include <math.h>

#define TickTime 1000
//#define TickTime 1000000

#define rotA	rot[0]
#define rotB	rot[1]
#define rotC	rot[2]
#define rotD	rot[3]

#define dir_x		2
#define dir_y		1
#define dir_minx	4
#define dir_miny	3

{// variable init
char string_buff[],lcd_buff[32],BT_buff[],char_buff[];
unsigned char r_buff6[10];

uint16_t delay_counter=0;
uint8_t isna=0;
uint8_t rx6_n=0,val,val2;

int16_t x=500;

float odo_x1=0,odo_x2=0,odo_y1=0,odo_y2=0,odo_x=0,odo_y=0;
float bat=0;

float trisna=0,irnanti=0;
float dwm_x=0.0,dwm_y=0,r1=0.0,r2=0.0,r3=0.0,tri_x=0.0,tri_y=0.0;

int deg;

int data_parse=0;
int k=0,n=0;
uint8_t sign=0,rx6_parse=0;

int data_parse2=0;
int k2=0,n2=0;
uint8_t sign2=0,rx2_parse=0;

uint32_t rot[4]={0,0,0,0};
uint32_t percobaan=1,panjang=0;

uint8_t dwm_timeout=15;

int head_error,headi,spd;
int last_error=0;
int set_speed=750,set_dir=0,Kp,mr,ml,head_tgt;
float trgt_x=0.0,trgt_y=0.0,distance=0.0;
uint8_t run_flag=0;
unsigned char array[10]={0},ar,test1,test2,test3;
float cmps_calfactor=-0.37;

int cmps_val;
float h_val;

float set_anc1x=3,set_anc1y=0;
float set_anc2x=5,set_anc2y=5;
float set_anc3x=0,set_anc3y=3;

unsigned char flag_stamp;
unsigned long systick_val=0,stamp=0;
}

void mylcd_clr()
{
	char ii,jj;

	for(ii=0;ii<=1;++ii)
	{
		for(jj=0;jj<=15;++jj)
		{
			lcd_xy(jj,ii);
			lcd_write(" ");
		}
	}
}

uint16_t get_compass()
{
	uint16_t headingDegrees=0;
	float heading=0;
	uint8_t hmc[6];
	uint16_t hmc_x,hmc_y,hmc_z;
	uint16_t raw;
	int32_t raw_x,raw_y,raw_z,rawe;

	//const int X_offset=0;
	//const int Y_offset=0;

	const int X_offset=75;
	const int Y_offset=258;

	if(!check_i2c_device(I2C2,0x3D))
	{
		BT_Send("hmc disconected ");
		return 0;
	}
	HMC5883_read(hmc);
	//xo = xl|(xh << 8);
	hmc_x = hmc[1]|(hmc[0]<<8);
	hmc_y = hmc[5]|(hmc[4]<<8);
	hmc_z = hmc[3]|(hmc[2]<<8);

	//used for debug
	//sprintf(BT_buff,"hmc raw  %3d  %3d  %3d  %3d  %3d  %3d   ",hmc[0],hmc[1],hmc[4],hmc[5],hmc[2],hmc[3]);
	//BT_Send(BT_buff);

	if(hmc_x>=32768)
	{
		hmc_x=~hmc_x;
		raw_x=-1-hmc_x;
		//rawe=-1-(~hmc_z);
	}else raw_x=hmc_x;

	if(hmc_y>=32768)
	{
		hmc_y=~hmc_y;
		raw_y=-1-hmc_y;
		//rawe=-1-(~hmc_z);
	}else raw_y=hmc_y;

	if(hmc_z>=32768)
	{
		hmc_z=~hmc_z;
		raw_z=-1-hmc_z;
		//rawe=-1-(~hmc_z);
	}else raw_z=hmc_z;

	raw_x+=X_offset;
	raw_y+=Y_offset;

	//used for debug
	//sprintf(BT_buff,"hmc par  %5d  %5d  %5d   ",raw_x,raw_y,raw_z);
	//BT_Send(BT_buff);

	heading=atan2(raw_y,raw_x);

	//sprintf(BT_buff,"\nhead %.2f\n",heading);
	//FTDI_Send(BT_buff);

	heading += -0.3086174271; //declination correction

	heading -= cmps_calfactor;

	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2*M_PI;

	// Check for wrap due to addition of declination.
	if(heading > 2*M_PI)
		heading -= 2*M_PI;

	 headingDegrees = heading * 180/M_PI;
	 return headingDegrees;
}

float convert_rotary(uint32_t rotary)
{
	//mengubah pulsa rotary ke satuan cm
	//jarak translasi ditentukan oleh diameter roda
	return ((float)rotary/5.0);
}

void get_odo()
{
	odo_x1=convert_rotary(rotA);
	odo_x2=convert_rotary(rotC);
	odo_y1=convert_rotary(rotB);
	odo_y2=convert_rotary(rotD);

	odo_x=(odo_x1 + odo_x2)/2;
	odo_y=(odo_y1 + odo_y2)/2;
}
void reset_odo()
{
	rotA=0;
	rotB=0;
	rotC=0;
	rotD=0;
}

//void trilaterasi(float *x, float *y)
void trilaterasi(float *x, float *y, float r1,float r2,float r3)
{
	//float r1=14.6, r2=8.4, r3=3.6 ,sr1=pow(r1,2),sr2=pow(r2,2),sr3=pow(r3,2);
	//float x1=6.5,  x2=0.5, x3=5.5 ,sx1=pow(x1,2),sx2=pow(x2,2),sx3=pow(x3,2);
	//float y1=12.5, y2=6.5, y3=0.5 ,sy1=pow(y1,2),sy2=pow(y2,2),sy3=pow(y3,2);

	float sr1=pow(r1,2),sr2=pow(r2,2),sr3=pow(r3,2);
	float x1=set_anc1x, x2=set_anc2x, x3=set_anc3x ,sx1=pow(x1,2),sx2=pow(x2,2),sx3=pow(x3,2);
	float y1=set_anc1y, y2=set_anc2y, y3=set_anc3y ,sy1=pow(y1,2),sy2=pow(y2,2),sy3=pow(y3,2);

	float pembilang,penyebut,temp;
	pembilang= (sr1-sr2 + sx2-sx1 + sy2-sy1)*(2*(y3-y2)) - (sr2-sr3 + sx3-sx2 + sy3-sy2)*(2*(y2-y1));
	penyebut=  (2*(x2-x3))*(2*(y2-y1)) - (2*(x1-x2))*(2*(y3-y2));
	temp= pembilang/penyebut;
	*x=temp;

	pembilang = (sr1-sr2 + sx2-sx1 + sy2-sy1) + (temp*2*(x1-x2));
	penyebut  = 2*(y2-y1);
	temp=pembilang/penyebut;
	*y=temp;
}

void move(uint8_t dir, int speed1,int speed2)
{
	if(dir==dir_y)
	{
		if(speed1>0)motor(MA,fwd,speed1);

		else if(speed1<0)
		{
			speed1*=-1;
			motor(MA,bwd,speed1);
		}
		else if(speed1==0)motor(MA,stop,speed1);

		if(speed2>0)motor(MC,fwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MC,bwd,speed2);
		}
		else if(speed2==0)motor(MC,stop,speed2);
	}


	else if(dir==dir_x)
	{
		if(speed1>0)motor(MD,bwd,speed1);
		else if(speed1<0)
		{
			speed1*=-1;
			motor(MD,fwd,speed1);
		}
		else if(speed1==0)motor(MD,stop,speed1);

		if(speed2>0)motor(MB,bwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MB,fwd,speed2);
		}
		else if(speed2==0)motor(MB,stop,speed2);
	}
}
void move_set(uint8_t dir, int degree, int hd_tgt,int speed1,int kp,int Ki)
{
	int speed2,speed3;
	int correction,error;

	int Pout,Iout;
	error=hd_tgt-degree;

	if(degree>=270 && degree<=359 && hd_tgt>=0 && hd_tgt<=90)
	{
		Pout=kp*(360+error);
		Iout=Ki*(360+error+last_error)*0.01;
	}
	else if(degree>=0 && degree<=90 && hd_tgt>=270 && hd_tgt<=359)
	{
		Pout=kp*(error-360);
		Iout=Ki*(last_error+error-360)*0.01;
	}
	else
	{
		Pout=kp*(error);
		Iout=Ki*(last_error+error)*0.01;
	}

	correction=Pout+Iout;

	last_error=error;

	speed2=speed1+correction;
	speed1-=correction;
	speed3=correction;

	if(dir==dir_y)
	{
		if(speed1>0)motor(MA,fwd,speed1);

		else if(speed1<0)
		{
			speed1*=-1;
			motor(MA,bwd,speed1);
		}
		else if(speed1==0)motor(MA,stop,speed1);

		if(speed2>0)motor(MC,fwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MC,bwd,speed2);
		}
		else if(speed2==0)motor(MC,stop,speed2);


		if(speed3>0)
		{
			motor(MB,fwd,speed3);
			motor(MD,bwd,speed3);
		}
		else if(speed3<0)
		{
			speed3*=-1;
			motor(MB,bwd,speed3);
			motor(MD,fwd,speed3);
		}
		else if(speed3==0)
		{
			motor(MB,stop,speed3);
			motor(MD,stop,speed3);
		}
	}

	else if(dir==dir_miny)
	{
		if(speed1>0)motor(MC,bwd,speed1);

		else if(speed1<0)
		{
			speed1*=-1;
			motor(MC,fwd,speed1);
		}
		else if(speed1==0)motor(MC,stop,speed1);

		if(speed2>0)motor(MA,bwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MA,fwd,speed2);
		}
		else if(speed2==0)motor(MA,stop,speed2);


		if(speed3>0)
		{
			motor(MB,fwd,speed3);
			motor(MD,bwd,speed3);
		}
		else if(speed3<0)
		{
			speed3*=-1;
			motor(MB,bwd,speed3);
			motor(MD,fwd,speed3);
		}
		else if(speed3==0)
		{
			motor(MB,stop,speed3);
			motor(MD,stop,speed3);
		}
	}

	else if(dir==dir_x)
	{
		if(speed1>0)motor(MB,bwd,speed1);
		else if(speed1<0)
		{
			speed1*=-1;
			motor(MB,fwd,speed1);
		}
		else if(speed1==0)motor(MB,stop,speed1);

		if(speed2>0)motor(MD,bwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MD,fwd,speed2);
		}
		else if(speed2==0)motor(MD,stop,speed2);

		if(speed3>0)
		{
			motor(MA,bwd,speed3);
			motor(MC,fwd,speed3);
		}
		else if(speed3<0)
		{
			speed3*=-1;
			motor(MA,fwd,speed3);
			motor(MC,bwd,speed3);
		}
		else if(speed3==0)
		{
			motor(MA,stop,speed3);
			motor(MC,stop,speed3);
		}
	}

	else if(dir==dir_minx)
	{
		if(speed1>0)motor(MD,fwd,speed1);
		else if(speed1<0)
		{
			speed1*=-1;
			motor(MD,bwd,speed1);
		}
		else if(speed1==0)motor(MD,stop,speed1);

		if(speed2>0)motor(MB,fwd,speed2);
		else if(speed2<0)
		{
			speed2*=-1;
			motor(MB,bwd,speed2);
		}
		else if(speed2==0)motor(MB,stop,speed2);

		if(speed3>0)
		{
			motor(MA,bwd,speed3);
			motor(MC,fwd,speed3);
		}
		else if(speed3<0)
		{
			speed3*=-1;
			motor(MA,fwd,speed3);
			motor(MC,bwd,speed3);
		}
		else if(speed3==0)
		{
			motor(MA,stop,speed3);
			motor(MC,stop,speed3);
		}
	}

	else if(dir==stop)
	{
		motor(MA,stop,0);
		motor(MC,stop,0);
		motor(MB,stop,0);
		motor(MD,stop,0);
	}

	//sprintf(BT_buff,"dir %1d c %4d r %4d l %4d h %3d\n",dir,correction,speed1,speed2,hd_tgt);
	//BT_Send(BT_buff);
}
void rotate(int speed)
{
	if(speed==stop)
	{
		motor(MA,stop,0);
		motor(MC,stop,0);
		motor(MB,stop,0);
		motor(MD,stop,0);
	}
	else if(speed>0)
	{
		motor(MA,bwd,speed);
		motor(MC,fwd,speed);
		motor(MB,fwd,speed);
		motor(MD,bwd,speed);
	}
	else if(speed<0)
	{
		speed*=-1;
		motor(MA,fwd,speed);
		motor(MC,bwd,speed);
		motor(MB,bwd,speed);
		motor(MD,fwd,speed);
	}
}

void point_heading(int head)
{
	int error=head-deg;
	uint8_t kp=100;

	head_error=error;
	rotate(kp*error);
}
uint16_t calc_heading(float tgt_x, float tgt_y,float ori_x,float ori_y)
{
	float heading=0;
	uint16_t headingDegrees=0;

	heading=atan2((tgt_x-ori_x),(tgt_y-ori_y));
	// Correct for when signs are reversed.

	if(heading < 0)
		heading += 2*M_PI;

	// Check for wrap due to addition of declination.
	if(heading > 2*M_PI)
		heading -= 2*M_PI;

	 headingDegrees = heading * 180/M_PI;
	 return headingDegrees;
}
float calc_distance(float tgt_x, float tgt_y,float ori_x,float ori_y)
{
	return (sqrt( pow((tgt_y-ori_y),2) + pow((tgt_x-ori_x),2)) );
}
void move_decission(int head, float dis, int *new_head, uint8_t *dir)
{
	int tmp=(*new_head);
	int tmp1;
	int tmp2;
	int tmp3;
	int tmp4;

	int min;
	uint8_t flag_tmp,it;

	//sprintf(BT_buff,"tmp %d",tmp);
	//BT_Send(BT_buff);

	if(dis<0.1)
	{
		*dir=stop;
	}
	//tmp2=360-head+tmp;
	else
	{
		tmp1=head;

		tmp2=tmp1+90;
		if(tmp2>=360)tmp2-=360;

		tmp3=tmp2+90;
		if(tmp3>=360)tmp3-=360;

		tmp4=tmp3+90;
		if(tmp4>=360)tmp4-=360;

		if(tmp1>=270 && tmp1<=359 && tmp>=0 && tmp<=90)
		{
			tmp1=360-tmp1+tmp;
		}
		else if(tmp1>=0 && tmp1<=90 && tmp>270 && tmp<=359)
		{
			tmp1=tmp1+360-tmp;
		}
		else tmp1-=tmp;

		if(tmp2>=270 && tmp2<=359 && tmp>=0 && tmp<=90)
		{
			tmp2=360-tmp2+tmp;
		}
		else if(tmp2>=0 && tmp2<=90 && tmp>=270 && tmp<=359)
		{
			tmp2=tmp2+360-tmp;
		}
		else tmp2-=tmp;

		if(tmp3>=270 && tmp3<=359 && tmp>=0 && tmp<=90)
		{
			tmp3=360-tmp3+tmp;
		}
		else if(tmp3>=0 && tmp3<=90 && tmp>=270 && tmp<=359)
		{
			tmp3=tmp3+360-tmp;
		}
		else tmp3-=tmp;

		if(tmp4>=270 && tmp4<=359 && tmp>=0 && tmp<=90)
		{
			tmp4=360-tmp4+tmp;
		}
		else if(tmp4>=0 && tmp4<=90 && tmp>=270 && tmp<=359)
		{
			tmp4=tmp4+360-tmp;
		}
		else tmp4-=tmp;

		tmp1=abs(tmp1);
		tmp2=abs(tmp2);
		tmp3=abs(tmp3);
		tmp4=abs(tmp4);

		//sprintf(BT_buff,"tmp %3d tmp2 %3d\n",tmp1,tmp2);
		//BT_Send(BT_buff);

		min=tmp1;
		flag_tmp=1;
		*dir=dir_y;

		if(tmp2<min)
		{
			min=tmp2;
			*dir=dir_x;
			flag_tmp=2;

			*new_head=tmp-90;
			if(*new_head<0)*new_head+=360;
		}

		if(tmp3<min)
		{
			min=tmp3;
			*dir=dir_miny;
			flag_tmp=3;

			*new_head=tmp+180;
			if(*new_head>359)*new_head-=360;
		}

		if(tmp4<min)
		{
			min=tmp4;
			*dir=dir_minx;
			flag_tmp=4;

			*new_head=tmp+90;
			if(*new_head>359)*new_head-=360;
		}

		//sprintf(BT_buff,"tmp1 %3d tmp2 %3d tmp3 %3d tmp4 %3d fl% 1d ht %3d\n",tmp1,tmp2,tmp3,tmp4,flag_tmp,*new_head);
		//BT_Send(BT_buff);


	}

}

void cal_compass()
{
	TIM_Cmd(TIM7, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

	delay_ms(50);
	//lcd_clr;
	mylcd_clr();

	while(1)
	{
		float heading=0;
		uint8_t hmc[6];
		uint16_t hmc_x,hmc_y,hmc_z;
		uint16_t raw;
		int32_t raw_x,raw_y,raw_z,rawe;

		const int X_offset=75;
		const int Y_offset=258;

		if(!check_i2c_device(I2C2,0x3D))
		{
			BT_Send("hmc disconected ");
			return 0;
		}

		HMC5883_read(hmc);

		hmc_x = hmc[1]|(hmc[0]<<8);
		hmc_y = hmc[5]|(hmc[4]<<8);

		if(hmc_x>=32768)
		{
			hmc_x=~hmc_x;
			raw_x=-1-hmc_x;
		}else raw_x=hmc_x;

		if(hmc_y>=32768)
		{
			hmc_y=~hmc_y;
			raw_y=-1-hmc_y;
		}else raw_y=hmc_y;

		raw_x+=X_offset;
		raw_y+=Y_offset;

		//used for debug
		//sprintf(BT_buff,"hmc par  %5d  %5d  %5d   ",raw_x,raw_y,raw_z);
		//BT_Send(BT_buff);

		heading=atan2(raw_y,raw_x);

		heading += -0.3086174271;

		sprintf(BT_buff,"\nhead %.2f\n",heading);
		FTDI_Send(BT_buff);

		h_val=heading;

	//if(cmps_val<=180)h_val=(cmps_val*M_PI)/180;
	//else h_val=((cmps_val-360)*M_PI)/180;

	lcd_xy(0,0);
	lcd_write("head_calib");
	sprintf(lcd_buff,"r_head=%.2frad",h_val);
	lcd_xy(0,1);
	lcd_write(lcd_buff);

	if(button0)
	{
		delay_ms(100);
		while(button0);
		delay_ms(100);

		cmps_calfactor=h_val;

		mylcd_clr();
		lcd_xy(0,0);
		lcd_write("calib finished");
		delay_ms(1500);
		mylcd_clr();

		TIM_Cmd(TIM7, ENABLE);
		TIM_Cmd(TIM3, ENABLE);
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
		break;
	}

	}

}

void set_anchor()
{
	unsigned char state_a=1,state_b=1;

	TIM_Cmd(TIM7, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

	mylcd_clr();
	lcd_xy(0,0);
	lcd_write("anchors setup");
	delay_ms(1500);
	mylcd_clr();

	while(1)
	{
		if(state_a==1)
		{
			if(state_b==1)
			{
				if(button2)set_anc1x+=0.1;
				if(button1)set_anc1x-=0.1;
				if(button3)state_b=2;

				lcd_xy(0,1);
				lcd_write("  ^      ");
			}
			else if(state_b==2)
			{
				if(button2)set_anc1y+=0.1;
				if(button1)set_anc1y-=0.1;
				if(button3)
				{
					state_a=2;
					state_b=1;
				}

				lcd_xy(0,1);
				lcd_write("      ^ ");
			}

			sprintf(lcd_buff,"(%.1f,%.1f)anch1  ",set_anc1x,set_anc1y);
			lcd_xy(0,0);
			lcd_write(lcd_buff);

		}

		else if(state_a==2)
		{
			if(state_b==1)
			{
				if(button2)set_anc2x+=0.1;
				if(button1)set_anc2x-=0.1;
				if(button3)state_b=2;

				lcd_xy(0,1);
				lcd_write("  ^      ");
			}
			else if(state_b==2)
			{
				if(button2)set_anc2y+=0.1;
				if(button1)set_anc2y-=0.1;
				if(button3)
				{
					state_a=3;
					state_b=1;
				}

				lcd_xy(0,1);
				lcd_write("      ^ ");
			}

			sprintf(lcd_buff,"(%.1f,%.1f)anch2  ",set_anc2x,set_anc2y);
			lcd_xy(0,0);
			lcd_write(lcd_buff);
		}

		else if(state_a==3)
		{
			if(state_b==1)
			{
				if(button2)set_anc3x+=0.1;
				if(button1)set_anc3x-=0.1;
				if(button3)state_b=2;

				lcd_xy(0,1);
				lcd_write("  ^      ");
			}
			else if(state_b==2)
			{
				if(button2)set_anc3y+=0.1;
				if(button1)set_anc3y-=0.1;
				if(button3)
				{
					state_a=1;
					state_b=1;
				}

				lcd_xy(0,1);
				lcd_write("      ^ ");
			}

			sprintf(lcd_buff,"(%.1f,%.1f)anch3  ",set_anc3x,set_anc3y);
			lcd_xy(0,0);
			lcd_write(lcd_buff);
		}

		if(button0)
		{
			delay_ms(100);
			while(button0);

			mylcd_clr();
			lcd_xy(0,0);
			lcd_write("anchors are set");
			delay_ms(1500);

			mylcd_clr();
			TIM_Cmd(TIM7, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
			USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
			break;
		}
		delay_ms(200);
	}
}

void set_tgt()
{
	unsigned char state_a=1,state_b=1;
	float set_tgtx=tri_x,set_tgty=tri_y;

	TIM_Cmd(TIM7, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);

	mylcd_clr();
	lcd_xy(0,0);
	lcd_write("set target");
	delay_ms(1500);
	mylcd_clr();

	while(1)
	{
		if(state_a==1)
		{
			if(state_b==1)
			{
				if(button2)set_tgtx+=0.1;
				if(button1)set_tgtx-=0.1;
				if(button3)state_b=2;

				lcd_xy(0,1);
				lcd_write("  ^      ");
			}
			else if(state_b==2)
			{
				if(button2)set_tgty+=0.1;
				if(button1)set_tgty-=0.1;
				if(button3)
				{
					state_a=2;
					state_b=1;
				}

				lcd_xy(0,1);
				lcd_write("      ^ ");
			}

			sprintf(lcd_buff,"(%.1f,%.1f) tgt",set_tgtx,set_tgty);
			lcd_xy(0,0);
			lcd_write(lcd_buff);
		}

		else if(state_a==2 && button2)
		{
			delay_ms(100);
			while(button0);

			trgt_x=set_tgtx;
			trgt_y=set_tgty;

			mylcd_clr();
			lcd_xy(0,0);
			lcd_write("tgr is set");
			delay_ms(1500);

			mylcd_clr();
			TIM_Cmd(TIM7, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
			USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
			break;
		}

		else if(state_a==2 && button1)
		{
			delay_ms(100);
			while(button0);

			mylcd_clr();
			lcd_xy(0,0);
			lcd_write("cancel set");
			delay_ms(1500);

			mylcd_clr();
			TIM_Cmd(TIM7, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
			USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
			break;
		}

		else if(state_a==2)
		{
			lcd_xy(0,0);
			lcd_write("up  >> set");
			lcd_xy(0,1);
			lcd_write("down>> cancel");

		}

		delay_ms(200);
	}
}
/*

* section for interrupt handler
 */

void SysTick_Handler(void)
{
  TimeTickDec();
  //systick_val++;

	//untuk robi
	/*if(flag_stamp==1)
	{
		stamp++;
	}else stamp=0;*/


  /*if(systick_val==1000)
  {
	  	  sprintf(char_buff,"systick %5d \n",systick_val);
		  FTDI_Send(char_buff);
		  systick_val=0;
  }*/

}

void USART2_IRQHandler() //BT
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		//USART_SendData(USART2,USART_ReceiveData(USART2));
		//#  500&$  300&
		//if(USART_ReceiveData(USART2)=='w')set_dir=dir_y;
		if(USART_ReceiveData(USART2)=='w')run_flag=1;
		//else if(USART_ReceiveData(USART2)=='d')set_dir=dir_x;
		else if(USART_ReceiveData(USART2)=='d')set_dir=dir_miny;
		else if(USART_ReceiveData(USART2)=='e')set_dir=stop;
		else if(USART_ReceiveData(USART2)=='a')set_dir=dir_minx;
		else if(USART_ReceiveData(USART2)=='s')run_flag=0;


		if (USART_ReceiveData(USART2)=='#' || USART_ReceiveData(USART2)=='$'||
			USART_ReceiveData(USART2)=='!' || USART_ReceiveData(USART2)=='@')
		{
			n2=0;
			k2=4;
			data_parse2=0;

			if(USART_ReceiveData(USART2)=='#')rx2_parse=1;
			else if(USART_ReceiveData(USART2)=='$')rx2_parse=2;
			else if(USART_ReceiveData(USART2)=='!')rx2_parse=3;
			else if(USART_ReceiveData(USART2)=='@')rx2_parse=4;
		}

		if(rx2_parse)
		{
			if((USART_ReceiveData(USART2)>=0x30 && USART_ReceiveData(USART2)<=0x39) || USART_ReceiveData(USART2)==' ' || USART_ReceiveData(USART2)=='-')
			{
				if(USART_ReceiveData(USART2)=='-')
				{
					val=0;
					sign2=1;
					//USART_SendString(USART2,"sign");
				}
				else val=USART_ReceiveData(USART2) & 0x0F;

				data_parse2+= val * pow(10,k2);
				k2--;

				if(rx2_parse==3)
				{
					data_parse2=val;
				}
			}

			else if(USART_ReceiveData(USART2)!='#' && USART_ReceiveData(USART2)!='$' &&
					USART_ReceiveData(USART2)!='!' && USART_ReceiveData(USART2)!='@' && USART_ReceiveData(USART2)!='&')
			{
				n2=0;
				rx2_parse=0;
				USART_SendString(USART2,"no parse");
			}

			if(USART_ReceiveData(USART2)=='&'&& rx2_parse==1)
			{
				n2=0;
				rx2_parse=0;
				trgt_x=data_parse2/100.0;
				if(sign2==1)trgt_x*=-1;
				sign2=0;
			}
			else if(USART_ReceiveData(USART2)=='&' && rx2_parse==2)
			{
				n2=0;
				rx2_parse=0;
				trgt_y=data_parse2/100.0;
				if(sign2==1)trgt_y*=-1;
				sign2=0;
			}
			else if(USART_ReceiveData(USART2)=='&' && rx2_parse==3)
			{
				n2=0;
				rx2_parse=0;
				set_dir=data_parse2;
				//if(sign2==1)trgt_y*=-1;
				//sign2=0;
			}
			else if(USART_ReceiveData(USART2)=='&' && rx2_parse==4)
			{
				n2=0;
				rx2_parse=0;
				set_speed=data_parse2;
				//if(sign2==1)set_speed*=-1;
				//sign2=0;
			}
		}
	}
}
void USART3_IRQHandler() // FTDI
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE))
	{
		if(USART_ReceiveData(USART3)=='a')USART_SendData(USART3,'t');
	}
}
void USART6_IRQHandler() //DWM
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE))
	{
		//USART_SendData(USART2,USART_ReceiveData(USART6));

		//// "! -111&@ -112&^   69&#  489&$  498&*\n"

		dwm_timeout=RESET;

		if (USART_ReceiveData(USART6)=='#' || USART_ReceiveData(USART6)=='$' ||
			USART_ReceiveData(USART6)=='!' || USART_ReceiveData(USART6)=='@' || USART_ReceiveData(USART6)=='^')
		{
			n=0;
			k=4;
			data_parse=0;

			if(USART_ReceiveData(USART6)=='#')rx6_parse=1;
			else if(USART_ReceiveData(USART6)=='$')rx6_parse=2;
			else if(USART_ReceiveData(USART6)=='!')rx6_parse=3;
			else if(USART_ReceiveData(USART6)=='@')rx6_parse=4;
			else rx6_parse=5;
		}
		else if(USART_ReceiveData(USART6)=='*') //data feedback
		{
			//sprintf(char_buff,"r1=%.2f r2=%.2f r3=%.2f x=%.2f , y=%.2f\n",r1,r2,r3,dwm_x,dwm_y);
			//BT_Send(char_buff);
			sprintf(lcd_buff,"%.1f %.1f %.1f   ",r1,r2,r3);
			lcd_xy(0,0);
			lcd_write(lcd_buff);
			sprintf(lcd_buff,"x=%.1f , y=%.1f   ",tri_x,tri_y);
			lcd_xy(0,1);
			lcd_write(lcd_buff);

			/*if(systick_val>50 && systick_val<500)
			{
				sprintf(char_buff,"%3d,%4d\n",stamp,systick_val);
				BT_Send(char_buff);
				systick_val=0;
				stamp++;
				if(stamp==100)stamp=0;
			}*/
		}

		if(rx6_parse)
		{
			if((USART_ReceiveData(USART6)>=0x30 && USART_ReceiveData(USART6)<=0x39) || USART_ReceiveData(USART6)==' ' || USART_ReceiveData(USART6)=='-')
			{
				if(USART_ReceiveData(USART6)=='-')
				{
					val2=0;
					sign=1;
					//USART_SendString(USART6,"sign");
				}
				else val2=USART_ReceiveData(USART6) & 0x0F;

				data_parse+= val2 * pow(10,k);
				k--;
			}
			else if(USART_ReceiveData(USART6)!='#' && USART_ReceiveData(USART6)!='$' && USART_ReceiveData(USART6)!='&' &&
					USART_ReceiveData(USART6)!='!' && USART_ReceiveData(USART6)!='@' && USART_ReceiveData(USART6)!='^' &&
					USART_ReceiveData(USART6)!='*' )
			{
				n=0;
				rx6_parse=0;
				USART_SendString(USART6,"no parse");
			}

			if(USART_ReceiveData(USART6)=='&'&& rx6_parse==1)
			{
				n=0;
				rx6_parse=0;
				dwm_x=data_parse/100.0;
				if(sign==1)dwm_x*=-1;
				sign=0;

			}
			else if(USART_ReceiveData(USART6)=='&' && rx6_parse==2)
			{
				n=0;
				rx6_parse=0;
				dwm_y=data_parse/100.0;
				if(sign==1)dwm_y*=-1;
				sign=0;

			}
			else if(USART_ReceiveData(USART6)=='&' && rx6_parse==3)
			{
				n=0;
				rx6_parse=0;
				r1=data_parse/100.0;
				if(sign==1)r1=-1;
				sign=0;

			}
			else if(USART_ReceiveData(USART6)=='&' && rx6_parse==4)
			{
				n=0;
				rx6_parse=0;
				r2=data_parse/100.0;
				if(sign==1)r2=-1;
				sign=0;
			}
			else if(USART_ReceiveData(USART6)=='&' && rx6_parse==5)
			{
				n=0;
				rx6_parse=0;
				r3=data_parse/100.0;
				if(sign==1)r3=-1;
				sign=0;
			}
		}
	}
}

void TIM3_IRQHandler()
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		deg=get_compass();
		headi=180;
		spd=set_speed;

		//move_set(dir_y,spd-correction,spd+correction,correction);

		//move(dir_y,spd-correction,spd+correction);

		trilaterasi(&tri_x,&tri_y,r1,r2,r3);
		head_tgt=calc_heading(trgt_x,trgt_y,tri_x,tri_y);
		//sprintf(BT_buff,"d %3d c %3d dir %1d ss %4d r %4d l %4d  %.2f,%.2f %3d\n",deg,correction,set_dir,set_speed,mr,ml,trgt_x,trgt_y,head_tgt);
		//BT_Send(BT_buff);

		distance=calc_distance(trgt_x,trgt_y,tri_x,tri_y);
		move_decission(deg,distance,&head_tgt,&set_dir);

		if(!run_flag)
		{
			move_set(stop,deg,head_tgt,spd,20,40);
			BT_Send("stop ");
		}
		else
		{
			move_set(set_dir,deg,head_tgt,spd,20,40);
			BT_Send("run ");
		}

		sprintf(BT_buff,"n_dir %1d hd %3d hd_tgt %3d pos(%.2f,%.2f) tgt(%.2f,%.2f) dis%.2f\n",set_dir,deg,head_tgt,tri_x,tri_y,trgt_x,trgt_y,distance);
		BT_Send(BT_buff);

		//point_heading(180);
		//USART_SendString(USART3,"test tim3");
	}
}
void TIM7_IRQHandler()
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//timer7 overflow ISR
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		//bat=(read_bat()*3.3)/4096;
		bat=read_bat();

		//trisna=read_us(0);
		//irnanti=read_us(1);

		//trilaterasi(&tri_x,&tri_y);

		sprintf(string_buff,"%3d p=%.2f ccl=%.2f an1(%.1f,%.1f) an2(%.1f,%.1f) an1(%.1f,%.1f) \n ",isna,bat,cmps_calfactor,set_anc1x,set_anc1y,set_anc2x,set_anc2y,set_anc3x,set_anc3y);
		USART_SendString(USART3,string_buff);

		isna++;

		//sprintf(string_buff,"hmc xyz  %4d  %4d %5d  heading=%.1f DEGREES=%d\n",raw_x,raw_y,raw_z,heading,headingDegrees);
		//USART_SendString(USART3,string_buff);

		dwm_timeout++;
		if(dwm_timeout>=15)
		{
			dwm_x=0.0;
			dwm_y=0.0;
			r1=0.0;
			r2=0.0;
			r3=0.0;
			dwm_timeout=15;

			lcd_xy(0,0);
			sprintf(lcd_buff,"dwm out  p=%.1fV",bat);
			lcd_write(lcd_buff);
			sprintf(lcd_buff,"nire ama :)     ");
			lcd_xy(0,1);
			lcd_write(lcd_buff);
		}
	}
}

void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		led0_togle;
		//USART_SendString(USART3,"exti12\n");
		 //Clear the EXTI line 0 pending bit
		EXTI_ClearITPendingBit(EXTI_Line12);
		rot[0]++;
	}

	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		led1_togle;
		//USART_SendString(USART3,"exti13\n");
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line13);
		rot[1]++;
	}

	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		led2_togle;
		//USART_SendString(USART3,"exti14\n");
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line14);
		rot[2]++;
	}

	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		led3_togle;
		//USART_SendString(USART3,"exti15\n");
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line15);
		rot[3]++;
	}
}
/*
 * end of section for interrupt handler
 */

int main(void)
{
	SystemInit();
	RCC_HSEConfig(RCC_HSE_ON);
	while(!RCC_WaitForHSEStartUp());

	while(SysTick_Config(SystemCoreClock/TickTime)!=0);

	led_init();
	button_init();
	us_init();
	USART2_init();
	USART3_init();
	USART6_init();
	EXTI_12_15_init();
	m_control_init();
	lcd_init();
	adc1_init();
	I2C2_init();
	HMC5883_init();USART_SendString(USART3,"hmc ok\n");

	TIM7_init();TIM_Cmd(TIM7, DISABLE);
	//TIM6_init();TIM_Cmd(TIM6, DISABLE);
	TIM4_pwm_init();

	sprintf(string_buff,"core clock=%d\n",SystemCoreClock);
	USART_SendString(USART3,string_buff);

	lcd_clr();
	USART_SendString(USART3,"start\n");
	led3_off;

	TIM_Cmd(TIM7, ENABLE);
	TIM3_init();

	//TIM_Cmd(TIM3, DISABLE);
	//TIM_Cmd(TIM7, DISABLE);

    while(1)
    {
    	if(button0)
    	{
    		while(button0);
    		delay_ms(100);
    		cal_compass();
    	}

    	if(button1)
    	{
    		while(button1);
    		delay_ms(100);
    		set_anchor();
    	}

    	if(button2)
		{
			while(button2);
			delay_ms(100);
			set_tgt();
		}

    	if(button3 && run_flag==0)
		{
			while(button3);
			delay_ms(100);
			run_flag=1;
		}
    	else if(button3 && run_flag==1)
		{
			while(button3);
			delay_ms(100);
			run_flag=0;
		}
	 }
}
