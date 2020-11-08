//******************************************************************************/
//
//					大连海事大学
//					信息科学技术学院
//					flight2.0
//  函数   	    : main.c
//  版本          	: v2.0
//  硬件作者   	: 李东霖
//  软件作者   	: 李东霖
//  工程创建时间  : 2020-0901
//  最近更新 	: 2020-0901
//	操作系统          ：FreeRTOS 9.0.0
//******************************************************************************/
#include "stm32f4xx.h" 
#include "sys.h"
#include "string.h"
#include "delay.h"
#include "usart.h"
#include "myTask.h"

/*			全局数据结构体定义			*/
/*			存储在静态存储区			*/
bmi088_gyroData_t bmi088_gyro_data;
bmi088_accelData_t bmi088_acc_data;
accData_t accData;
gyroData_t gyroData;
presData_t presData;
lpfData_t accLpfData[3];
lpfData_t gyroLpfData[3];
attribute_t attribute; 

uint8_t tempU8[10];
int32_t tempS32;
uint16_t pwm[4]={0};

uint8_t buffer[50];
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  				//中断优先级分组4
	delay_init(168); 
	uart_init(115200);												//USB转串口通道
	uart3_init();													//串口3 用于GPS通信
	SbusInit();														//接收机使用SBUS模式
	LED_Init();
	SPI1_Init();
	SPI2_Init();
	bmi088_init();
	spl06_init();
	imuLpfFilterInit(&accLpfData[0],&gyroLpfData[0]);					//低通滤波器
	TIME5_Init(1000,84);  //(1000,84) 对应解算频率1000hz
	pwmOutInit();
	buffer[0]=0x03;
	buffer[1]=0xFC;
	buffer[10]=0xFC;
	buffer[11]=0x03;
	setPwm(pwm);
	delay_ms(2000);
	pwm[0]=300;
	pwm[1]=300;
	pwm[2]=300;
	pwm[3]=300;
//	setPwm(pwm);
		while(1)
	{
//		bmi088_read(0X00,tempU8,2,ACC);
//		imuDataSend(&accData,&gyroData);
//		for(int i=0;i<25;i++)
		printf("%d\t",sbusIn[0]);
//		Usart1_Send((char *)buffer,12);
		delay_ms(300);
//		LED0=!LED0;
	}
	
	xTaskCreate((TaskFunction_t	)start_task,		//任务函数
				(const char *	)"start_task",		//任务名字
				(uint16_t		)START_STK_SIZE,	//堆栈大小
				(void *			)NULL,				//传递给任务的参数
				(UBaseType_t	)START_TASK_PRIO,	//任务优先级
				(TaskHandle_t*	)&startTask_Handle	//任务句柄
				);
	vTaskStartScheduler();                          //开始任务调度
}





