#ifndef __UART4_H
#define __UART4_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define UART4_REC_LEN  			100  	//定义最大接收字节数 100
#define EN_UART4_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  UART4_RX_BUF[UART4_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 UART4_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart4_init(u32 bound);
void Uart4_Send(char* Data,unsigned int Length);		//串口1发送数据
#endif


