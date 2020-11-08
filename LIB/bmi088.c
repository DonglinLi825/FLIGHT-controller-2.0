#include "bmi088.h"

int8_t bmi088_init()
{
	uint8_t temp;
/*				加速度部分初始化							*/	
	delay_ms(1);
	bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);				//将ACC切换到SPI模式
	delay_us(1000);
	temp=bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);		//读取ACC部分传感器ID，检测是否好用
	if(temp!=BMI08X_ACCEL_CHIP_ID)								//如果传感器无效，返回
		return 1;
	delay_us(100);
//	bmi088_write_byte(BMI08X_ACCEL_SOFTRESET_REG,0xb6,ACC);		//ACC部分软件复位
	delay_ms(2);
	bmi088_read_byte(BMI08X_ACCEL_CHIP_ID_REG,ACC);				//将ACC切换到SPI模式
	delay_us(100);
	bmi088_write_byte(BMI08X_ACCEL_PWR_CONF_REG,0x00,ACC);		//进入使能模式
	delay_ms(5);
	bmi088_write_byte(BMI08X_ACCEL_PWR_CTRL_REG,0x04,ACC);		//打开加速度计
	delay_ms(50);
	bmi088_write_byte(BMI08X_ACCEL_CONF_REG,
	(1<<7) | (0<<4) | (0xb<<0),ACC);							//加速度计ODR 800hz
	delay_us(200);
	bmi088_write_byte(BMI08X_ACCEL_RANGE_REG,ACC_RANGE,ACC);	//加速度计量程+-24g
	delay_ms(100);
/*				加速度部分初始化完成							*/	
/*				陀螺仪部分初始化								*/		
	
	temp=bmi088_read_byte(BMI08X_GYRO_CHIP_ID_REG,GYRO);		//读取GYRO部分传感器ID，检测是否好用
	if(temp!=BMI08X_GYRO_CHIP_ID)								//如果传感器无效，返回
		return 1;
	delay_us(10);
//	bmi088_write_byte(BMI08X_GYRO_SOFTRESET_REG,0xb6,GYRO);		//GYRO部分软件复位
	delay_ms(30);
	bmi088_write_byte(BMI08X_GYRO_LPM1_REG,0x00,GYRO);			//功率模式正常
	delay_ms(30);
	bmi088_write_byte(BMI08X_GYRO_RANGE_REG,0x00,GYRO);			//陀螺仪量程2000°/s
	delay_us(10);
	bmi088_write_byte(BMI08X_GYRO_BANDWIDTH_REG,GYRO_RANGE,GYRO);//陀螺仪ODR2000HZ 滤波器带宽230HZ
	delay_us(10);
	return 0;
}
u8 txbuf[10]={0x80,0xff};
u8 rxbuf[10];

/*				BMI088向地址内写入一字节数据				*/
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
/*				BMI088从地址内读出一字节数据				*/
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
/*				BMI088从地址内读出一定长度数据				*/
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
/*				BMI088基本数据读取，包括加速度部分三轴加速度，温度，传感器时间等			*/
/*				BMI088基本数据读取，陀螺仪部分三轴陀螺仪信息								*/
/*				实际测试过程中使用1000HZ230hz带宽的陀螺仪信息非常稳定，转化为deg/s为单位时
				上下漂移仅仅1到10															*/
void bmi088_dataRead(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p bmi088_gyroData)
{
	bmi088_read(0x12,(uint8_t*)bmi088_accData,sizeof(bmi088_accelData_t),ACC);
	bmi088_read(0x02,(uint8_t*)bmi088_gyroData,sizeof(bmi088_gyroData_t),GYRO);
}
void bmi088_dataTransform(bmi088_accelData_p bmi088_accData,bmi088_gyroData_p 
	bmi088_gyroData,accData_p accData,gyroData_p gyroData)
{
/*	因为传感器摆放方向问题，因此需要将传感器坐标方向旋转90度		*/
/*	参考手册实例方向，从上向下看，顺时针转动90°						*/	
	accData->accX =	-(float)(bmi088_accData->acc[1])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;		//单位是1/1000倍的重力加速度G
	accData->accY = (float)(bmi088_accData->acc[0])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;		
	accData->accZ = (float)(bmi088_accData->acc[2])*1000/32768*pow(2,ACC_RANGE+1)*1.5f;
	
	bmi088_accData->temperature=(BYTE1(bmi088_accData->temperature)<<8)|(BYTE2(bmi088_accData->temperature)>>5);
	accData->temperature=(float)(bmi088_accData->temperature>>5)/8+23;
	
	gyroData->gyroX_deg=-(float)(bmi088_gyroData->gyro[1])*2000/32678;						//单位是deg/s
	gyroData->gyroY_deg=(float)(bmi088_gyroData->gyro[0])*2000/32678;
	gyroData->gyroZ_deg=(float)(bmi088_gyroData->gyro[2])*2000/32678;
/*					弧度信息可以在滤波器中赋值，节省滤波器数量								*/
//	gyroData->gyroX_rad=gyroData->gyroX_deg*pi/180;											//单位是rad/s
//	gyroData->gyroY_rad=gyroData->gyroY_deg*pi/180;
//	gyroData->gyroZ_rad=gyroData->gyroZ_deg*pi/180;	
}

