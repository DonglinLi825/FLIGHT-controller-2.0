#ifndef __SPI_H_
#define __SPI_H_
#include "sys.h"
#include "delay.h"
#include "spl06.h"
#include "bmi088.h"



void SPI1_Init(void);
void SPI1_SetSpeed(uint8_t SpeedSet); //����SPI1�ٶ� 
uint8_t SPI1_ReadWriteOneByte(uint8_t TxData);//SPI1���߶�дһ���ֽ�
void SPI2_Init(void);
uint8_t SPI2_ReadWriteOneByte(uint8_t TxData);

#endif
