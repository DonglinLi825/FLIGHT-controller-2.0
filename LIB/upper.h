#ifndef __UPPER_H
#define __UPPER_H

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "attribute.h"

void imuDataSend(const accData_p accData,const gyroData_p gyroData);
void pressureDataSend(const presData_t* presData);
void attributeDataSend(const attribute_p attribute);
void sensorTimeSend(const bmi088_accelData_p bmi088_accelData);



#endif 


