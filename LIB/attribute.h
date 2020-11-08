#ifndef __ATTRIBUTE_H
#define __ATTRIBUTE_H

#include "sys.h"
#include "bmi088.h"
#include "spl06.h"
#include "math.h"
#include "arm_math.h" 
#include "myMath.h"

#define pi ((float)3.1415926f)

typedef struct attribute_t
{
	double pitch;
	double roll;
	double yaw;
}attribute_t;
typedef attribute_t* attribute_p;

void attributeCal(accData_p accData,gyroData_p gyroData,attribute_p attribute);


#endif 

