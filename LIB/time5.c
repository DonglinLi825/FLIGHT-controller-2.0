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
/*			ͨ�ö�ʱ���жϳ�ʼ��
			����ʱ��ѡ��ΪAPB2��2��
			arr���Զ���װֵ��
			psc��ʱ��Ԥ��Ƶ������168MHz��ʱ�ӷֳ�ָ��Ƶ�ʣ���ΧΪ1��65536��
			����ʹ�õ��Ƕ�ʱ��5!							*/
unsigned char time5;
void TIME5_Init(u16 arr,u16 psc)           						// ���ʱ�䣨us�� = arr * psc / TIME5������ʱ��Ƶ�ʣ�84MHz��
{																// (ms) = arr / ��ʱ�ļ���Ƶ��(KHz)
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 		//ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr-1; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 				//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:һ��Ƶ��TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 			//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM5, TIM_IT_Update ,ENABLE );   				//ʹ��time5ͨ�ö�ʱ���ĸ���ʹ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  			//TIM5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//��ռ���ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  		//�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); 							//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM5, ENABLE);  									//ʹ��TIMx����
							 
}
extern uint8_t buffer[50];
void TIM5_IRQHandler(void)   //TIM5�ж�
{	
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 			//���ָ����TIM�жϷ������:TIM �ж�Դ 
		{	
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update );  		//���TIMx���жϴ�����λ:TIM �ж�Դ 
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

