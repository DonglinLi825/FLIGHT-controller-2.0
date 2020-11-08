#include "SPI.h"


void SPI1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);			//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);				//使能SPI1时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	//PA5,6,7复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//初始化
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);			//初始化GPIOC时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//	
	GPIO_Init(GPIOC, &GPIO_InitStructure);							//初始化PC4作为Gyro片选
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//初始化PA4作为ACC部分片选
	
	ACC_CS_HIGH;
	GYRO_CS_HIGH;
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);					//复位SPI1
	delay_ms(10);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);				//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//定义波特率预分频的值:波特率预分频值为16，对应5Mbitps左右
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure); 								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI1, ENABLE);  
	SPI1_ReadWriteOneByte(0xff);
}

void SPI1_SetSpeed(uint8_t SpeedSet) //设置SPI1速度 
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SpeedSet));//判断有效性
	SPI_Cmd(SPI1,DISABLE); 												//关闭SPI1
	SPI1->CR1&=0xFFC7;													//位3到5清零，用来设置波特率
	SPI1->CR1|=SpeedSet;												//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); 												//使能SPI1
}

//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteOneByte(uint8_t TxData)
{	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}		//等待发送区空  
	SPI_I2S_SendData(SPI1, TxData); 									//通过外设SPIx发送一个byte数据
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}		//等待接收完一个byte  
	return SPI_I2S_ReceiveData(SPI1); 									//返回通过SPIx最近接收的数据
}

void SPI2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);								//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);									//使能SPI2时钟
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;					//PAB13，14，15复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;										//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//初始化
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;											//PB12作为SPL06的片选引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										//	
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//初始化PB12作为气压计片选
	PRE_CS_HIGH;
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);								//
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);								//
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);								//

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);									//复位SPI2
	delay_ms(10);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);								//停止复位SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  				//设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;										//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;									//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;											//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;										//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;											//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;					//定义波特率预分频的值:波特率预分频值为16，对应5Mbitps左右
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;									//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;											//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  												//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI2, ENABLE);  
	SPI2_ReadWriteOneByte(0xff);	//使能SPI外设
}

uint8_t SPI2_ReadWriteOneByte(uint8_t TxData)
{	
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}						//等待发送区空  					
	SPI_I2S_SendData(SPI2, TxData); 													//通过外设SPIx发送一个byte数
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}						//等待接收完一个byte  
	return SPI_I2S_ReceiveData(SPI2); 													//返回通过SPIx最近接收的数据
}
