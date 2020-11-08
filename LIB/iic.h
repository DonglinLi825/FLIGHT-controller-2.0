#ifndef __iic_H
#define	__iic_H
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"

//IO��������
#define SDA_IN()  {GPIOB->MODER&=0X0FFFFFFF;GPIOB->MODER|=(u32)0<<2;}
#define SDA_OUT() {GPIOB->MODER&=0X0FFFFFFF;GPIOB->MODER|=(u32)1<<2;}
 
//IO��������	 
#define IIC_SCL    PBout(0) //SCL
#define IIC_SDA    PBout(1) //SDA	 
#define READ_SDA   PBin(1)  //����SDA 
 
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
#endif
