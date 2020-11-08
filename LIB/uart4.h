#ifndef __UART4_H
#define __UART4_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define UART4_REC_LEN  			100  	//�����������ֽ��� 100
#define EN_UART4_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  UART4_RX_BUF[UART4_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 UART4_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart4_init(u32 bound);
void Uart4_Send(char* Data,unsigned int Length);		//����1��������
#endif


