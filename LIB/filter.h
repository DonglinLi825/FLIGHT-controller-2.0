#ifndef __FILTER_H
#define __FILTER_H
#include "sys.h"
#include "bmi088.h"
#include "spl06.h"
#include "arm_math.h" 

#define M_PI_F ((double)3.1415926f)
#define SAMPLE_FREQ 1000
#define ACC_CUTOFF_FREQ 30
#define GYRO_CUTOFF_FREQ 300

typedef struct 
{
	double a1;
	double a2;
	double b0;
	double b1;
	double b2;
	double delay_element_1;
	double delay_element_2;
} lpfData_t;

void imuLpfFilterInit(lpfData_t* accLpf,lpfData_t* gyroLpf);
void lpfInit(lpfData_t* lpfData, double sample_freq, double cutoff_freq);
void lpfSetCutOffFreq(lpfData_t* lpfData, double sample_freq, double cutoff_freq);
double lpfApply(lpfData_t* lpfData, double input);
void imuDataFilter(accData_p accData,gyroData_p gyroData,lpfData_t* accLpfData,lpfData_t* gyroLpfData);



#endif



