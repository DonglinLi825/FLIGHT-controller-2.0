#ifndef __SPI_H_
#define __SPI_H_
#include "sys.h"
#include "delay.h"
#include "spl06.h"
#include "bmi088.h"



void SPI1_Init(void);
void SPI1_SetSpeed(uint8_t SpeedSet); //设置SPI1速度 
uint8_t SPI1_ReadWriteOneByte(uint8_t TxData);//SPI1总线读写一个字节
void SPI2_Init(void);
uint8_t SPI2_ReadWriteOneByte(uint8_t TxData);

#endif
