#include "bmi088.h"

int8_t bmi088_init()
{
	uint8_t temp;
/*				���ٶȲ��ֳ�ʼ��							*/	
	delay_ms(1);
	bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);				//��ACC�л���SPIģʽ
	delay_us(1000);
	temp=bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);		//��ȡACC���ִ�����ID������Ƿ����
	if(temp!=BMI08X_ACCEL_CHIP_ID)								//�����������Ч������
		return 1;
	delay_us(100);
//	bmi088_write_byte(BMI08X_ACCEL_SOFTRESET_REG,0xb6,ACC);		//ACC���������λ
	delay_ms(2);
	bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);				//��ACC�л���SPIģʽ
	delay_us(100);
	bmi088_write_byte(BMI08X_ACCEL_PWR_CONF_REG,0x00,ACC);		//����ʹ��ģʽ
	delay_ms(5);
	bmi088_write_byte(BMI08X_ACCEL_PWR_CTRL_REG,0x04,ACC);		//�򿪼��ٶȼ�
	delay_ms(50);
	bmi088_write_byte(BMI08X_ACCEL_CONF_REG,
	(1<<7) | (0<<4) | (0xb<<0),ACC);							//���ٶȼ�ODR 800hz
	delay_us(200);
	bmi088_write_byte(BMI08X_ACCEL_RANGE_REG,ACC_RANGE,ACC);	//���ٶȼ�����+-24g
	delay_ms(100);
/*				���ٶȲ��ֳ�ʼ�����							*/	
/*				�����ǲ��ֳ�ʼ��								*/		
	
	temp=bmi088_read_byte(BMI08X_GYRO_CHIP_ID_REG,GYRO);		//��ȡGYRO���ִ�����ID������Ƿ����
	if(temp!=BMI08X_GYRO_CHIP_ID)								//�����������Ч������
		return 1;
	delay_us(10);
//	bmi088_write_byte(BMI08X_GYRO_SOFTRESET_REG,0xb6,GYRO);		//GYRO���������λ
	delay_ms(30);
	bmi088_write_byte(BMI08X_GYRO_LPM1_REG,0x00,GYRO);			//����ģʽ����
	delay_ms(30);
	bmi088_write_byte(BMI08X_GYRO_RANGE_REG,0x00,GYRO);			//����������2000��/s
	delay_us(10);
	bmi088_write_byte(BMI08X_GYRO_BANDWIDTH_REG,GYRO_RANGE,GYRO);//������ODR2000HZ �˲�������230HZ
	delay_us(10);
	return 0;
}
u8 txbuf[10]={0x80,0xff};
u8 rxbuf[10];

/*				BMI088���ַ��д��һ�ֽ�����				*/
uint8_t bmi088_write_byte(uint8_t reg,uint8_t val,uint8_t sensor)
{	
	uint8_t result; 
//	reg&=0x7F;	
	if(sensor)
		ACC_CS_LOW;
	else 
		GYRO_CS_LOW;
	SPI1_ReadWriteOneByte(reg);
	SPI1_ReadWriteOneByte(val);
	if(sensor)
		ACC_CS_HIGH;
	else 
		GYRO_CS_HIGH;
	return result;
	
}
/*				BMI088�ӵ�ַ�ڶ���һ�ֽ�����				*/
uint8_t bmi088_read_byte(uint8_t reg,uint8_t sensor)
{
	uint8_t result=0;
	reg|=0x80;	
	if(sensor)
		ACC_CS_LOW;
	else 
		GYRO_CS_LOW;
//	SPI1_TransmitReceive_Start(txbuf,rxbuf,2);	
	SPI1_ReadWriteOneByte(reg);
	result=SPI1_ReadWriteOneByte(0Xff);
	if(sensor)
		ACC_CS_HIGH;
	else 
		GYRO_CS_HIGH;
	return result;
}
/*				BMI088�ӵ�ַ�ڶ���һ����������				*/
void bmi088_read(uint8_t reg,uint8_t* buffer,uint8_t len,uint8_t sensor)
{
	reg|=0x80;	
	if(sensor)
		ACC_CS_LOW;
	else 
		GYRO_CS_LOW;
	SPI1_ReadWriteOneByte(reg);
	for(int i=0;i<len;i++)
	buffer[i]=SPI1_ReadWriteOneByte(0xff);
	if(sensor)
		ACC_CS_HIGH;
	else 
		GYRO_CS_HIGH;
}
/*				BMI088�������ݶ�ȡ���������ٶȲ���������ٶȣ��¶ȣ�������ʱ���			*/
/*				BMI088�������ݶ�ȡ�������ǲ���������������Ϣ								*/
/*				ʵ�ʲ��Թ�����ʹ��1000HZ230hz�������������Ϣ�ǳ��ȶ���ת��Ϊdeg/sΪ��λʱ
				����Ư�ƽ���1��10															*/
void bmi088_dataRead(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p bmi088_gyroData)
{
	bmi088_read(0x12,(uint8_t*)bmi088_accData,sizeof(bmi088_accelData_t),ACC);
	bmi088_read(0x02,(uint8_t*)bmi088_gyroData,sizeof(bmi088_gyroData_t),GYRO);
}
void bmi088_dataTransform(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p 
	bmi088_gyroData,accData_p accData,gyroData_p gyroData)
{
/*	��Ϊ�������ڷŷ������⣬�����Ҫ�����������귽����ת90��		*/
/*	�ο��ֲ�ʵ�����򣬴������¿���˳ʱ��ת��90��						*/	
	accData->accX =	-(float)(bmi088_accData->acc[1])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;		//��λ��1/1000�����������ٶ�G
	accData->accY = (float)(bmi088_accData->acc[0])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;		
	accData->accZ = (float)(bmi088_accData->acc[2])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;
	
	bmi088_accData->temperature=(BYTE1(bmi088_accData->temperature)<<8)|(BYTE2(bmi088_accData->temperature)>>5);
	accData->temperature=(float)(bmi088_accData->temperature>>5)/8+23;
	
	gyroData->gyroX_deg=-(float)(bmi088_gyroData->gyro[1])*2000/32678;						//��λ��deg/s
	gyroData->gyroY_deg=(float)(bmi088_gyroData->gyro[0])*2000/32678;
	gyroData->gyroZ_deg=(float)(bmi088_gyroData->gyro[2])*2000/32678;
/*					������Ϣ�������˲����и�ֵ����ʡ�˲�������								*/
//	gyroData->gyroX_rad=gyroData->gyroX_deg*pi/180;											//��λ��rad/s
//	gyroData->gyroY_rad=gyroData->gyroY_deg*pi/180;
//	gyroData->gyroZ_rad=gyroData->gyroZ_deg*pi/180;	
}

