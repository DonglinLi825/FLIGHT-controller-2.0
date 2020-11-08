#include "AK8975.h"







uint8_t ak8975_read_byte(uint8_t addr)
{
	uint8_t result=0;
	IIC_Start();
	IIC_Send_Byte((AK8975_ADDR<<1));
	if(IIC_Wait_Ack())
		return 0;
	IIC_Send_Byte(addr);
	if(IIC_Wait_Ack())
		return 0;
	IIC_Start();
	IIC_Send_Byte((AK8975_ADDR<<1)|1);
	if(IIC_Wait_Ack())
		return 0;
	result=IIC_Read_Byte(0);
	IIC_Stop();
	return result;
}






