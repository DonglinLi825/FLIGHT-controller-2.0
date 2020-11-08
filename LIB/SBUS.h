#ifndef _SBUS_H_
#define _SBUS_H_

#include "stm32f4xx.h"
#include "LED.h"
#include "usart.h"


extern uint16_t sbusIn[16];
extern uint8_t dataTemp[25];
	
#define SBUS_USART USART2


//
void SbusInit(void);
void sbusGetData(uint8_t data);
void USART2_IRQHandler(void);




#endif


