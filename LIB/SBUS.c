#include "SBUS.h"

void SbusInit(void)
{
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_StructInit(&GPIO_InitStructure);

	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART2, ENABLE ); //
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );
	

	GPIO_PinAFConfig ( GPIOA, GPIO_PinSource3, GPIO_AF_USART2 );//
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//GPIO_PuPd_UP;//
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
	
	USART_InitStructure.USART_BaudRate = 100000;       
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_2;   
    USART_InitStructure.USART_Parity = USART_Parity_Even;    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode = USART_Mode_Rx; 
    USART_Init ( SBUS_USART, &USART_InitStructure );
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );
    USART_ITConfig ( SBUS_USART, USART_IT_RXNE, ENABLE );	

    USART_Cmd ( SBUS_USART, ENABLE );
}


/*
sbus flags的结构如下所示：
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10) b: 0001 0000
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a

*/
uint16_t sbusIn[16];
uint8_t sbusFlag;
static uint8_t dataTemp[25];

void sbusGetData(uint8_t data)
{

	static u8 cnt = 0;
	
	dataTemp[cnt++] = data;
	
	if(cnt == 25)
	{
		if(dataTemp[0] == 0x0F && ((dataTemp[24] == 0x00)||(dataTemp[24] == 0x04)||(dataTemp[24] == 0x14)||(dataTemp[24] == 0x24)||(dataTemp[24] == 0x34)))
		{	
			LED0=!LED0;
			cnt = 0;
			sbusIn[0] = (int16_t)(dataTemp[2] & 0x07) << 8 | dataTemp[1];
			sbusIn[1] = (int16_t)(dataTemp[3] & 0x3f) << 5 | (dataTemp[2] >> 3);
			sbusIn[2] = (int16_t)(dataTemp[5] & 0x01) << 10 | ((int16_t)dataTemp[4] << 2) | (dataTemp[3] >> 6);
			sbusIn[3] = (int16_t)(dataTemp[6] & 0x0F) << 7 | (dataTemp[5] >> 1);
			sbusIn[4] = (int16_t)(dataTemp[7] & 0x7F) << 4 | (dataTemp[6] >> 4);
			sbusIn[5] = (int16_t)(dataTemp[9] & 0x03) << 9 | ((int16_t)dataTemp[8] << 1) | (dataTemp[7] >> 7);
			sbusIn[6] = (int16_t)(dataTemp[10] & 0x1F) << 6 | (dataTemp[9] >> 2);
			sbusIn[7] = (int16_t)dataTemp[11] << 3 | (dataTemp[10] >> 5);
			
			sbusIn[8] = (int16_t)(dataTemp[13] & 0x07) << 8 | dataTemp[12];
			sbusIn[9] = (int16_t)(dataTemp[14] & 0x3f) << 5 | (dataTemp[13] >> 3);
			sbusIn[10] = (int16_t)(dataTemp[16] & 0x01) << 10 | ((int16_t)dataTemp[15] << 2) | (dataTemp[14] >> 6);
			sbusIn[11] = (int16_t)(dataTemp[17] & 0x0F) << 7 | (dataTemp[16] >> 1);
			sbusIn[12] = (int16_t)(dataTemp[18] & 0x7F) << 4 | (dataTemp[17] >> 4);
			sbusIn[13] = (int16_t)(dataTemp[20] & 0x03) << 9 | ((int16_t)dataTemp[19] << 1) | (dataTemp[18] >> 7);
			sbusIn[14] = (int16_t)(dataTemp[21] & 0x1F) << 6 | (dataTemp[20] >> 2);
			sbusIn[15] = (int16_t)dataTemp[22] << 3 | (dataTemp[21] >> 5);
			sbusFlag = dataTemp[23];
			
			//user
			//
//			if(sbusFlag & 0x08)
//			{
//				//如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
//			}
//			else
//			{
//				//否则有数据就喂狗
//				for(u8 i = 0;i < 8;i++)//原RC接收程序只设计了8个通道
//				{
////					ch_watch_dog_feed(i);
//				}
//			}
		}
//		else
//		{
//			for(u8 i=0; i<24;i++)
//				dataTemp[i] = dataTemp[i+1];
//			cnt = 24;
//		}
	}
}

void USART2_IRQHandler(void)
{
	u8 rdata;
	
	if ( USART_GetITStatus ( SBUS_USART, USART_IT_RXNE ) )
    {
		USART_ClearITPendingBit ( SBUS_USART, USART_IT_RXNE ); 
		uint8_t err = 0;
		static uint8_t SBUS_RC_State = 0;	
		static unsigned char current_rc_channel = 0;
		static unsigned char rc_bit_count = 0;
		static unsigned short current_rc = 0;	
		#define SBUS_RC_Reset (SBUS_RC_State = current_rc_channel = rc_bit_count = current_rc = 0)  
		
        rdata = SBUS_USART->DR;
		


		sbusGetData(rdata);
//		if(err)
//			SBUS_RC_Reset;
//		
//		if(SBUS_RC_State == 0)
//		{
//			if(rdata == 0x0f)
//			{
//				++SBUS_RC_State;
//			}
//		}
//		else if(SBUS_RC_State<=22)
//		{
//			rc_bit_count += 8;
//			if(rc_bit_count >=11)
//			{
//				rc_bit_count -= 11;
//				unsigned char l_byte_count = 8 - rc_bit_count;
//				sbusIn[ current_rc_channel++ ] = 0.04885197850512945774303859306302f *
//							(float)( current_rc | ( ((unsigned char)( rdata << rc_bit_count )) << 3 ) );
//				current_rc = (unsigned char)( rdata >> l_byte_count );
//			}
//			else
//			{
//				current_rc |= rdata << (rc_bit_count - 8);	
//			}
//			++SBUS_RC_State;
//		}else if(SBUS_RC_State == 23)
//		{
//			//判断是否失控
//			#define SBUS_FAILSAFE_BIT	3
//			#define SBUS_FRAMELOST_BIT	2
////			failsafe = rdata & (1<<SBUS_FAILSAFE_BIT);
//			++SBUS_RC_State;
//		}
//		else
//		{
//			if( rdata == 0 )
//			{						
//				SBUS_RC_Reset;

//			}
//		}
			
		
		
		
//		Usart1_Send(&rdata,1);
		if(USART_GetFlagStatus(SBUS_USART,USART_FLAG_ORE))
		{
			USART_ClearFlag(SBUS_USART,USART_FLAG_ORE);
			USART_ReceiveData(USART2); 
		}
		
    }

}

