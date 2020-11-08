#ifndef __iic_H
#define	__iic_H
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=0X0FFFFFFF;GPIOB->MODER|=(u32)0<<2;}
#define SDA_OUT() {GPIOB->MODER&=0X0FFFFFFF;GPIOB->MODER|=(u32)1<<2;}
 
//IO操作函数	 
#define IIC_SCL    PBout(0) //SCL
#define IIC_SDA    PBout(1) //SDA	 
#define READ_SDA   PBin(1)  //输入SDA 
 
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
#endif
