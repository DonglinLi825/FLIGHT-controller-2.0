#include "upper.h"

/*		����λ�����ͷɻ���ǰIMU����						*/
void imuDataSend(const accData_p accData,const gyroData_p gyroData)
{
	
	u8 _cnt=0;
	vs16 _temp;
	u8 data_to_send[50];
/*			�̶���ʽ��Э��֡ͷ		*/
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0XFF;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
/*			��Ҫ����Ĵ���������	*/	
	_temp = accData->accX;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	_temp = accData->accY;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	_temp = accData->accZ;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	
	_temp = gyroData->gyroX_deg;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	_temp = gyroData->gyroY_deg;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	_temp = gyroData->gyroZ_deg;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);

	data_to_send[_cnt++]=1;

	data_to_send[3] = _cnt-4;
	
	u8 check_sum1 = 0, \
	check_sum2 = 0;
	for(u8 i=0;i<_cnt;i++)
	{
		check_sum1 += data_to_send[i];
		check_sum2 += check_sum1;
	}
	data_to_send[_cnt++] = check_sum1;
	data_to_send[_cnt++] = check_sum2;
	Usart1_Send(data_to_send,_cnt);
	

		
}
/*		����λ�����ͷɻ���ǰ��ѹ�ƴ���������			*/
void pressureDataSend(const presData_t* presData)
{
	uint8_t buffer[20];
	uint8_t sumCheck = 0;
	uint8_t addCheck = 0;
	/*		�̶�֡ͷ��ʽ								*/
	buffer[0]=0xaa;
	buffer[1]=0xff;
	buffer[2]=0x02;
	buffer[3]=14;
	/*		�����ơ���ѹ������							*/
	buffer[4]=(int16_t)(0)&0xff;		
	buffer[5]=((int16_t)(0)>>8);	
	buffer[6]=(int16_t)(0)&0xff;
	buffer[7]=((int16_t)(0)>>8);	
	buffer[8]=(int16_t)(0)&0xff;
	buffer[9]=((int16_t)(0)>>8);	
	buffer[10]=BYTE1(presData->high);
	buffer[11]=BYTE2(presData->high);
	buffer[12]=BYTE3(presData->high);
	buffer[13]=BYTE4(presData->high);
	buffer[14]=BYTE1(presData->temperature * 10);
	buffer[15]=BYTE2(presData->temperature * 10);
	buffer[16]=(int8_t)(1)&0xff;
	buffer[17]=(int8_t)(1)&0xff;	
	for(uint8_t i=0;i < (buffer[3]+4); i++)
	{
		sumCheck += buffer[i]; 							//��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addCheck += sumCheck; 							//ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	buffer[18]=sumCheck;
	buffer[19]=addCheck;
	Usart1_Send(buffer,20);
}


void attributeDataSend(const attribute_p attribute) 
{
	uint8_t buffer[20];
	uint8_t sumCheck = 0;
	uint8_t addCheck = 0;
	/*		�̶�֡ͷ��ʽ								*/
	buffer[0]=0xaa;
	buffer[1]=0xff;
	buffer[2]=0x03;
	buffer[3]=7;

	/*		��ǰ��̬����								*/
	buffer[4]=BYTE1((attribute->pitch) * 100);
	buffer[5]=BYTE2((attribute->pitch) * 100);	
	buffer[6]=BYTE1((attribute->roll) * 100);
	buffer[7]=BYTE2((attribute->roll) * 100);	
	buffer[8]=BYTE1(-(attribute->yaw) * 100);
	buffer[9]=BYTE2(-(attribute->yaw) * 100);
	//��ΪBMI088������YAW������Ϊ˳ʱ��Ϊ������������λ���෴����˼Ӹ���
	buffer[10]=(int8_t)(0)&0xff;
	for(uint8_t i=0;i < (buffer[3]+4); i++)
	{
		sumCheck += buffer[i]; 							//��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
		addCheck += sumCheck; 							//ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
	}
	buffer[11]=sumCheck;
	buffer[12]=addCheck;
	Usart1_Send(buffer,13);

}


void sensorTimeSend(const bmi088_accelData_p bmi088_accelData)
{
	int8_t buffer[8];
	buffer[0]=0x03;
	buffer[1]=0xfc;	
	buffer[2]=BYTE1(bmi088_accelData->sensor_time);
	buffer[3]=BYTE2(bmi088_accelData->sensor_time);
	buffer[4]=BYTE3(bmi088_accelData->sensor_time);
	buffer[5]=0;
	buffer[6]=0xfc;
	buffer[7]=0x03;
	Usart1_Send(buffer,8);
}







