#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx.h"
#include "sys.h"
#include "myMath.h"


#define MOTORS 	4


void pwmOutInit(void);
void setPwm ( uint16_t pwm[MOTORS] );








#endif



