#ifndef __GPS_H
#define __GPS_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "delay.h"

#define UART3_REC_LEN  			100  	//定义最大接收字节数 100
#define EN_UART3_RX 			1		//使能（1）/禁止（0）串口1接收
	
typedef struct{
	unsigned char satellite_num;
	unsigned char Location_stu;			//定位状态
	double latitude;						//纬度		
	double longitude;						//经度		
	int N_vel;							//南北向速度
	int E_vel;							//东西向速度
	
	float last_N_vel;							//上次南北向速度
	float last_E_vel;							//上次东西向速度
	
	float real_N_vel;
	float real_E_vel;
	
	unsigned char run_heart;			//运行心跳
	double start_latitude;					//起始纬度		
	double start_longitude;					//起始经度		
	int latitude_offset;
	int longitude_offset;
	float hope_latitude;
	float hope_longitude;
	float hope_latitude_err;
	float hope_longitude_err;
	unsigned char new_pos_get;
	unsigned char Back_home_f;
	float home_latitude;
	float home_longitude;
	
}GPS_INF;


extern u8  UART3_RX_BUF[UART3_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 UART3_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart3_init(void);
void Usart3_Send(const unsigned char* Data,unsigned int Length);		//串口3发送数据
unsigned char GPS_ubx_check_sum(unsigned char *Buffer);
void GPS_data_analysis(void);


#endif


