#ifndef __MYTASK_H
#define __MYTASK_H

#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h" 
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "usmart.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LED.h"
#include "SPI.h"
#include "iic.h"
#include "ak8975.h"
#include "bmi088.h"
#include "spl06.h"
#include "myTask.h"
#include "time5.h"
#include "filter.h"
#include "attribute.h"
#include "upper.h"
#include "GPS.h"
#include "SBUS.h"
#include "PWM.h"


extern TaskHandle_t startTask_Handle;      //任务句柄


#define START_TASK_PRIO		1       //任务优先级
#define START_STK_SIZE		128     //任务堆栈大小
void start_task(void *p_arg);

#define LED_TASK_PRIO		2       //任务优先级
#define LED_STK_SIZE		128     //任务堆栈大小            //任务句柄
void led_task(void *p_arg);


#endif


