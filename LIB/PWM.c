#include "PWM.h"


//����У׼������Ӧ����40%��80%
#define PWM_MIN	4000
#define PWM_MAX	8000

#define ACCURACY	10000
#define HZ		400

#define PWM_PERIOD LIMIT ( (ACCURACY*HZ) , 1 ,84000000 )
#define PWM_RATIO 	4


/*	��ʼ��PWM�ŵ���ѡ�ö�ʱ��4 ��·PWM			*/
void pwmOutInit()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM4, ENABLE );
	RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOB,ENABLE );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PB6789
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
/*	��������λTIM4����							*/	
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource6, GPIO_AF_TIM4 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource7, GPIO_AF_TIM4 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource8, GPIO_AF_TIM4 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource9, GPIO_AF_TIM4 );
	
	uint16_t prescalerValue=( uint16_t )( (SystemCoreClock / 2 ) / PWM_PERIOD ) - 1 ;
	
	//Ԥ��Ƶϵ��Ϊ21����Ƶ����ΪACCURACYΪ10000����PWMƵ��Ϊ400HZ��ռ�ձȿɵ�����Ϊ10000
	TIM_TimeBaseStructure.TIM_Period = ACCURACY - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM4, &TIM_TimeBaseStructure );
/*	��ʼ����ʱ�����ͨ��ģʽ1������Ϊ�ߣ���ʼ���ߵ�ƽʱ��Ϊ4000		*/	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PWM_MIN;
	//�ֱ��ʼ����ʱ��PWM���ĸ�ͨ��
    TIM_OC1PreloadConfig ( TIM4, TIM_OCPreload_Enable );	
    TIM_OC1Init ( TIM4, &TIM_OCInitStructure );

    TIM_OC2PreloadConfig ( TIM4, TIM_OCPreload_Enable );	
    TIM_OC2Init ( TIM4, &TIM_OCInitStructure );
	
    TIM_OC3PreloadConfig ( TIM4, TIM_OCPreload_Enable );	
    TIM_OC3Init ( TIM4, &TIM_OCInitStructure );
	
    TIM_OC4Init ( TIM4, &TIM_OCInitStructure );
    TIM_OC4PreloadConfig ( TIM4, TIM_OCPreload_Enable );	
	
    TIM_CtrlPWMOutputs ( TIM4, ENABLE );
    TIM_ARRPreloadConfig ( TIM4, ENABLE );
    TIM_Cmd ( TIM4, ENABLE );	
	
}

void setPwm ( uint16_t pwm[MOTORS] )
{
	//��PWM��������޷���������Ҫ��0��1000֮��
	pwm[0]=LIMIT( pwm[0] , 0 , 1000 );	
	pwm[1]=LIMIT( pwm[1] , 0 , 1000 );
	pwm[2]=LIMIT( pwm[2] , 0 , 1000 );
	pwm[3]=LIMIT( pwm[3] , 0 , 1000 );
	
    TIM4->CCR1 = PWM_RATIO * ( pwm[0] ) + PWM_MIN;				//1
    TIM4->CCR2 = PWM_RATIO * ( pwm[1] ) + PWM_MIN;				//2
    TIM4->CCR3 = PWM_RATIO * ( pwm[2] ) + PWM_MIN;				//3
    TIM4->CCR4 = PWM_RATIO * ( pwm[3] ) + PWM_MIN;				//4

}















