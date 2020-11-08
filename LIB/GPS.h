#ifndef __GPS_H
#define __GPS_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "delay.h"

#define UART3_REC_LEN  			100  	//�����������ֽ��� 100
#define EN_UART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	
typedef struct{
	unsigned char satellite_num;
	unsigned char Location_stu;			//��λ״̬
	double latitude;						//γ��		
	double longitude;						//����		
	int N_vel;							//�ϱ����ٶ�
	int E_vel;							//�������ٶ�
	
	float last_N_vel;							//�ϴ��ϱ����ٶ�
	float last_E_vel;							//�ϴζ������ٶ�
	
	float real_N_vel;
	float real_E_vel;
	
	unsigned char run_heart;			//��������
	double start_latitude;					//��ʼγ��		
	double start_longitude;					//��ʼ����		
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


extern u8  UART3_RX_BUF[UART3_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 UART3_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart3_init(void);
void Usart3_Send(const unsigned char* Data,unsigned int Length);		//����3��������
unsigned char GPS_ubx_check_sum(unsigned char *Buffer);
void GPS_data_analysis(void);


#endif


