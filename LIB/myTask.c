#include "myTask.h"

TaskHandle_t startTask_Handle;      //������
TaskHandle_t ledTask_Handle;   

void start_task(void *pvParameters){
	taskENTER_CRITICAL();	
	xTaskCreate((TaskFunction_t	)led_task,			//������
				(const char *	)"led_task",		//��������
				(uint16_t		)LED_STK_SIZE,		//��ջ��С
				(void *			)NULL,				//���ݸ�����Ĳ���
				(UBaseType_t	)LED_TASK_PRIO,		//�������ȼ�
				(TaskHandle_t*	)&ledTask_Handle			//������
				);
	vTaskDelete(startTask_Handle);
	taskEXIT_CRITICAL();	//�˳��ٽ�����
}



void led_task(void *pvParameters){
	while(1)
	{
		vTaskDelay(1000);
		LED0=!LED0;
	}
}



