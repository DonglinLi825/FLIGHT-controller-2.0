#include "SPI.h"


void SPI1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);			//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);				//ʹ��SPI1ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	//PA5,6,7���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//��ʼ��
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);			//��ʼ��GPIOCʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//	
	GPIO_Init(GPIOC, &GPIO_InitStructure);							//��ʼ��PC4��ΪGyroƬѡ
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//��ʼ��PA4��ΪACC����Ƭѡ
	
	ACC_CS_HIGH;
	GYRO_CS_HIGH;
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);					//��λSPI1
	delay_ms(10);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);				//ֹͣ��λSPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16����Ӧ5Mbitps����
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure); 								//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

	SPI_Cmd(SPI1, ENABLE);  
	SPI1_ReadWriteOneByte(0xff);
}

void SPI1_SetSpeed(uint8_t SpeedSet) //����SPI1�ٶ� 
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SpeedSet));//�ж���Ч��
	SPI_Cmd(SPI1,DISABLE); 												//�ر�SPI1
	SPI1->CR1&=0xFFC7;													//λ3��5���㣬�������ò�����
	SPI1->CR1|=SpeedSet;												//����SPI1�ٶ� 
	SPI_Cmd(SPI1,ENABLE); 												//ʹ��SPI1
}

//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI1_ReadWriteOneByte(uint8_t TxData)
{	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		//�ȴ���������  
	SPI_I2S_SendData(SPI1, TxData); 									//ͨ������SPIx����һ��byte����
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}		//�ȴ�������һ��byte  
	return SPI_I2S_ReceiveData(SPI1); 									//����ͨ��SPIx������յ�����
}

void SPI2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);								//ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);									//ʹ��SPI2ʱ��
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;					//PAB13��14��15���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;										//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//��ʼ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;											//PB12��ΪSPL06��Ƭѡ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										//	
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//��ʼ��PB12��Ϊ��ѹ��Ƭѡ
	PRE_CS_HIGH;
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);								//
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);								//
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);								//

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);									//��λSPI2
	delay_ms(10);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);								//ֹͣ��λSPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  				//����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;										//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;									//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;											//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;										//����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;											//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;					//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16����Ӧ5Mbitps����
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;									//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;											//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  												//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

	SPI_Cmd(SPI2, ENABLE);  
	SPI2_ReadWriteOneByte(0xff);	//ʹ��SPI����
}

uint8_t SPI2_ReadWriteOneByte(uint8_t TxData)
{	
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}						//�ȴ���������  					
	SPI_I2S_SendData(SPI2, TxData); 													//ͨ������SPIx����һ��byte��
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}						//�ȴ�������һ��byte  
	return SPI_I2S_ReceiveData(SPI2); 													//����ͨ��SPIx������յ�����
}
