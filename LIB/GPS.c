#include "sys.h"
#include "GPS.h"
#include "usart.h"
#include "string.h"

//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos ʹ��	  
#include "task.h"
#endif

#define GPS_UART	USART3

GPS_INF gpsInformation;
/*				����GPS���������Լ�����				*/
const unsigned char gps_petrol_out_config[28]=
{
//	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9
	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42
};//���ò���������

const unsigned char gps_pvt_out_config[90]=
{
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1,
//	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x01,0x01,0x01,0x01,0x01,0x00,0x27,0x3D
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0xD5,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x29
};

const unsigned char gps_rate_out_config[16]=
{
	0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12  //len 14
};//������Ϣ��������

const unsigned char Enter_Send[]={0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};

void gps_baudrate_config(void)
{
	delay_ms(200);
//		UART_Write_D(gps_rate_out_config,14);
	Usart3_Send(gps_petrol_out_config,28);

	Usart3_Send(Enter_Send,sizeof(Enter_Send));
		delay_ms(20);
}

void gps_config(void)
{
	delay_ms(100);

	Usart3_Send(gps_pvt_out_config,90);
		delay_ms(20);

	Usart3_Send(gps_rate_out_config,14);
		delay_ms(20);

//	UART_Write_D(gps_petrol_out_config,28);
//		delay_ms(20);
	
	Usart3_Send(Enter_Send,sizeof(Enter_Send));
		delay_ms(20);
}

#if EN_UART3_RX   //���ʹ���˽���
//����3�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 UART3_RX_BUF[UART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 UART3_RX_STA=0;       //����״̬���	

//��ʼ��IO ����2
//bound:������
void uart3_init(){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10	 GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB10 11
/*			USART3�ж�����			*/	
#if EN_UART3_RX	
	USART_ITConfig(GPS_UART, USART_IT_RXNE, ENABLE);//��������ж�
	//USART3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
#endif

	USART_DeInit(GPS_UART);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate          =   9600;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config();
	
//	USART_Cmd(GPS_UART,DISABLE);
	USART_InitStructure.USART_BaudRate          =   38400;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config();


//	USART_Cmd(GPS_UART,DISABLE);
    USART_InitStructure.USART_BaudRate 			= 115200;       //����Ϊ���õ�115200
    USART_InitStructure.USART_WordLength 		= USART_WordLength_8b;  //8λ���ݳ���
    USART_InitStructure.USART_StopBits 			= USART_StopBits_1;   //1λֹͣλ
    USART_InitStructure.USART_Parity 			= USART_Parity_No;    //
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //
    USART_InitStructure.USART_Mode 				= USART_Mode_Tx | USART_Mode_Rx;  //
    USART_Init ( GPS_UART, &USART_InitStructure );
    USART_Cmd ( GPS_UART, ENABLE );		
	gps_config();	

    USART_ITConfig ( GPS_UART, USART_IT_RXNE, ENABLE );

}

void Usart3_Send(const unsigned char* Data,unsigned int Length)		//����3��������
{
	u16 i;
	while(USART_GetFlagStatus(GPS_UART,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
	for(i=0;i<Length;i++)
	{
		USART_SendData(GPS_UART,(uint8_t)Data[i]); 	 //�������ݵ�����3
		while(USART_GetFlagStatus(GPS_UART,USART_FLAG_TC)==RESET);  //�ȴ��ϴδ������ 
	}
}

uint8_t gpsBuffer[100];
uint8_t gpsCnt = 0;
void USART3_IRQHandler(void)                	//����3�жϷ������
{	
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	taskENTER_CRITICAL();	  
#endif

	unsigned char rxData;  

	if ( GPS_UART->SR & USART_SR_ORE ) //ORE??
    {
        rxData = GPS_UART->DR;
    }
	
	if(USART_GetITStatus(GPS_UART,USART_IT_RXNE)==SET)//USART_IT_RXNE:????  
	{ 	
		USART_ClearITPendingBit(GPS_UART,USART_IT_RXNE);  
		rxData=USART_ReceiveData(GPS_UART); 
		
	//	Usart1_Send(&rxData,1);
		
		if (gpsCnt == 0)
		{
			if (rxData == 0xB5)									//֡ͷ1
			{
				gpsBuffer[gpsCnt] = rxData;
				gpsCnt = 1;
			}
		}
		else if (gpsCnt == 1)
		{
			if (rxData == 0x62)									//֡ͷ2
			{
				gpsBuffer[gpsCnt] = rxData;
				gpsCnt = 2;
			}
			else
			{
				gpsCnt = 0;
			}
		}
		else
		{
			gpsBuffer[gpsCnt] = rxData;
			gpsCnt++;
			if (gpsCnt >= 100)
			{
				gpsCnt = 0;
				
				if (GPS_ubx_check_sum(gpsBuffer))			//GPS����У��
				{
					GPS_data_analysis();						//GPS���ݽ���
				}
			}
		}			
	}		
	
	
	
	
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	taskEXIT_CRITICAL();		
#endif
}
unsigned short len;
unsigned char GPS_ubx_check_sum(unsigned char *gpsBuffer)
{
	unsigned char CK_A = 0, CK_B = 0;
	unsigned short  i;
	
	len = gpsBuffer[4] + (gpsBuffer[5]<<8);
	if (len > 100)
	{
		return 0;
	}
	for (i = 2; i < len+6; i++)
	{
		CK_A = CK_A + gpsBuffer[i];
		CK_B = CK_B + CK_A;
	}
	if (CK_A == gpsBuffer[len+6] && CK_B == gpsBuffer[len+7])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void GPS_data_analysis(void)
{
	gpsInformation.last_N_vel = (float)gpsInformation.N_vel;							//��¼�ϴ��ϱ����ٶ�
	gpsInformation.last_E_vel = (float)gpsInformation.E_vel;							//��¼�ϴζ������ٶ�
	
	gpsInformation.satellite_num = gpsBuffer[29];									//��������
	gpsInformation.longitude = gpsBuffer[30] + (gpsBuffer[31]<<8) + (gpsBuffer[32]<<16) + (gpsBuffer[33]<<24);		//����
	gpsInformation.latitude  = gpsBuffer[34] + (gpsBuffer[35]<<8) + (gpsBuffer[36]<<16) + (gpsBuffer[37]<<24);		//γ��
	gpsInformation.N_vel	  = gpsBuffer[54] + (gpsBuffer[55]<<8) + (gpsBuffer[56]<<16) + (gpsBuffer[57]<<24);		//�ϱ����ٶ�
	gpsInformation.E_vel	  = gpsBuffer[58] + (gpsBuffer[59]<<8) + (gpsBuffer[60]<<16) + (gpsBuffer[61]<<24);		//�������ٶ�
	
	gpsInformation.N_vel /=10;								//��λ���� cm/s
	gpsInformation.E_vel /=10;								//��λ���� cm/s	

	if (gpsInformation.satellite_num >= 6 && gpsInformation.new_pos_get == 0)			//������������6��,�ҵ�һ�λ�ȡ��γ�ȵ�
	{
		gpsInformation.new_pos_get = 1;
		gpsInformation.start_longitude = gpsInformation.longitude;
		gpsInformation.start_latitude  = gpsInformation.latitude;
		gpsInformation.hope_latitude  = 0;
		gpsInformation.hope_longitude = 0;

	}
	
	if (gpsInformation.new_pos_get)
	{
		gpsInformation.latitude_offset  = gpsInformation.latitude  - gpsInformation.start_latitude;
		gpsInformation.longitude_offset = gpsInformation.longitude - gpsInformation.start_longitude;
	}

	gpsInformation.run_heart++;	
}






#endif	

 



