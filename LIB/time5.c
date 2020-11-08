#include "time5.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "adc.h"
#include "bmi088.h"
#include "filter.h"
#include "attribute.h"
#include "upper.h" 


extern bmi088_gyroData_t bmi088_gyro_data;
extern bmi088_accelData_t bmi088_acc_data;
extern accData_t accData;
extern gyroData_t gyroData;
extern presData_t presData;
extern lpfData_t accLpfData[3];
extern lpfData_t gyroLpfData[3];
extern attribute_t attribute; 
/*			通用定时器中断初始化
			这里时钟选择为APB2的2倍
			arr：自动重装值。
			psc：时钟预分频数（将168MHz的时钟分成指定频率，范围为1到65536）
			这里使用的是定时器5!							*/
unsigned char time5;
void TIME5_Init(u16 arr,u16 psc)           						// 溢出时间（us） = arr * psc / TIME5的输入时钟频率（84MHz）
{																// (ms) = arr / 此时的计数频率(KHz)
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 		//时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr-1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 				//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//设置时钟分割:一分频，TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 			//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM5, TIM_IT_Update ,ENABLE );   				//使能time5通用定时器的更新使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  			//TIM5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  		//从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure); 							//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM5, ENABLE);  									//使能TIMx外设
							 
}
extern uint8_t buffer[50];
void TIM5_IRQHandler(void)   //TIM5中断
{	
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 			//检查指定的TIM中断发生与否:TIM 中断源 
		{	
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update );  		//清除TIMx的中断待处理位:TIM 中断源 
			static uint8_t cnt=0;
			bmi088_dataRead(&bmi088_acc_data,&bmi088_gyro_data);
			bmi088_dataTransform(&bmi088_acc_data,&bmi088_gyro_data,&accData,&gyroData);
			imuDataFilter(&accData,&gyroData,&accLpfData[0],&gyroLpfData[0]);
			spl06_getPresData(&presData);
			cnt++;
			if(cnt==10)
			{
//				attributeDataSend(&attribute);
//				imuDataSend(&accData,&gyroData);
//				sensorTimeSend(&bmi088_acc_data);
				pressureDataSend(&presData);
				cnt=0;
			}
		
			attributeCal(&accData,&gyroData,&attribute);

			
			

		}
}

