#include "filter.h"

/*			使用二阶IIR滤波器						*/

void imuLpfFilterInit(lpfData_t* accLpf,lpfData_t* gyroLpf)
{
	for(int i=0;i<3;i++)
	{
	lpfInit(&accLpf[i],SAMPLE_FREQ,ACC_CUTOFF_FREQ);
	lpfInit(&gyroLpf[i],SAMPLE_FREQ,GYRO_CUTOFF_FREQ);	
	}

}

/* 			二阶低通滤波参数初始化					*/
void lpfInit(lpfData_t* lpfData, double sample_freq, double cutoff_freq)
{
	if (lpfData == 0 || cutoff_freq <= 0.0f) 
	{	
		return;
	}

	lpfSetCutOffFreq(lpfData,sample_freq,cutoff_freq);

}


/*			根据采样频率和截止频率设置参数			*/
void lpfSetCutOffFreq(lpfData_t* lpfData, double sample_freq, double cutoff_freq)
{
	double fr = sample_freq/cutoff_freq;
	double ohm = tanf(M_PI_F/fr);
	double c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
	lpfData->b0 = ohm*ohm/c;
	lpfData->b1 = 2.0f*lpfData->b0;
	lpfData->b2 = lpfData->b0;
	lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
	lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	lpfData->delay_element_1 = 0.0f;
	lpfData->delay_element_2 = 0.0f;
	
}

/*			滤波器函数滤波函数						*/
double lpfApply(lpfData_t* lpfData, double input)
{
	double delay_element_0=input-lpfData->delay_element_1*lpfData->a1 
							-lpfData->delay_element_2*lpfData->a2;
	if (!isfinite(delay_element_0)) 
	{	
		delay_element_0=input;						// don't allow bad values to propigate via the filter
	}

	double output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
	return output;
}
/*			利用初始化的结构体对加速度计和陀螺仪滤波*/
void imuDataFilter(accData_p accData,gyroData_p gyroData,lpfData_t* accLpfData,lpfData_t* gyroLpfData)	
{	
	accData->accX=lpfApply(&accLpfData[0],accData->accX);
	accData->accY=lpfApply(&accLpfData[1],accData->accY);	
	accData->accZ=lpfApply(&accLpfData[2],accData->accZ);
	
	gyroData->gyroX_deg=lpfApply(&gyroLpfData[0],gyroData->gyroX_deg);
	gyroData->gyroY_deg=lpfApply(&gyroLpfData[1],gyroData->gyroY_deg);	
	gyroData->gyroZ_deg=lpfApply(&gyroLpfData[2],gyroData->gyroZ_deg);
	
	gyroData->gyroX_rad=gyroData->gyroX_deg*M_PI_F/180;
	gyroData->gyroY_rad=gyroData->gyroY_deg*M_PI_F/180;
	gyroData->gyroZ_rad=gyroData->gyroZ_deg*M_PI_F/180;
											
}









