#include "myTask.h"

TaskHandle_t startTask_Handle;      //任务句柄
TaskHandle_t ledTask_Handle;   

void start_task(void *pvParameters){
	taskENTER_CRITICAL();	
	xTaskCreate((TaskFunction_t	)led_task,			//任务函数
				(const char *	)"led_task",		//任务名字
				(uint16_t		)LED_STK_SIZE,		//堆栈大小
				(void *			)NULL,				//传递给任务的参数
				(UBaseType_t	)LED_TASK_PRIO,		//任务优先级
				(TaskHandle_t*	)&ledTask_Handle			//任务句柄
				);
	vTaskDelete(startTask_Handle);
	taskEXIT_CRITICAL();	//退出临界区域
}



void led_task(void *pvParameters){
	while(1)
	{
		vTaskDelay(1000);
		LED0=!LED0;
	}
}



