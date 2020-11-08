#include "attribute.h"

#define ATTRIBUTE_KP	5.0f
#define ATTRIBUTE_KI	0.1f
#define ATTRIBUTE_KD	0.01f
#define CAL_PERIOD		0.001f

float QE_b[3][3];//��Ԫ����Ч��ת����
/*			�㷨�ο�����
			https://blog.csdn.net/guanjianhe/article/details/95608801
			�����ߵ��㷨����ϵ���								
			��Ԫ����ŷ���Ƿ�									
			���׽������û��ʹ��STM32��DSP����					*/
void attributeCal(accData_p accData,gyroData_p gyroData,attribute_p attribute)
{
	static float 	q0=1,	q1=0,	q2=0,	q3=0;					//��Ԫ�����壬��̬����
	static float errorXInit=0,errorYInit=0,errorZInit=0;		//����������
	float 	vX,vY,vZ;											//��������ϵ����������
	float 	errorX,errorY,errorZ;								//�����������
	float	norm;												//��һ���õ���ʱ����		
	

	
	float q0_last = q0;										//����Ԫ������									
	float q1_last = q1;
	float q2_last = q2;
	float q3_last = q3;
		

	
/*			���������������һ��								
			������ٶȼƹ�һ��									
			Ϊ������ٶȣ����ó˷�								*/
	norm=1/sqrt(accData->accX * accData->accX + accData->accY * 
				accData->accY + accData->accZ * accData->accZ);
	accData->accX *= norm;
	accData->accY *= norm;
	accData->accZ *= norm;
/*			��������������ڶ���								
			��������ϵ����������								*/
	vX = QE_b[2][0];
	vY = QE_b[2][1];
	vZ = QE_b[2][2];
/*			�������������������								
			���ù�һ�����ٶȺ�������������õ���̬���			*/
	errorX = accData->accY * vZ - accData->accZ * vY;
	errorY = accData->accZ * vX - accData->accX * vZ;
	errorZ = accData->accX * vY - accData->accY * vX;
/*			����������������Ĳ�								
			������											*/
	errorXInit += errorX * ATTRIBUTE_KI * CAL_PERIOD;
	errorYInit += errorY * ATTRIBUTE_KI * CAL_PERIOD;
	errorZInit += errorZ * ATTRIBUTE_KI * CAL_PERIOD;
/*			����������������岽								
			�����˲�											*/
	gyroData->gyroX_rad += ATTRIBUTE_KP * ( errorXInit + errorX );
	gyroData->gyroY_rad += ATTRIBUTE_KP * ( errorYInit + errorY );
	gyroData->gyroZ_rad += ATTRIBUTE_KP * ( errorZInit + errorZ );
/*			��Ԫ�������һ��									
			������Ԫ��											*/
	q0=q0_last+(-gyroData->gyroX_rad * q1_last - gyroData->gyroY_rad * q2_last - gyroData->gyroZ_rad * q3_last) * CAL_PERIOD/2;
	q1=q1_last+( gyroData->gyroX_rad * q0_last - gyroData->gyroY_rad * q3_last + gyroData->gyroZ_rad * q2_last) * CAL_PERIOD/2;
	q2=q2_last+( gyroData->gyroX_rad * q3_last + gyroData->gyroY_rad * q0_last - gyroData->gyroZ_rad * q1_last) * CAL_PERIOD/2;
	q3=q3_last+(-gyroData->gyroX_rad * q2_last + gyroData->gyroY_rad * q1_last + gyroData->gyroZ_rad * q0_last) * CAL_PERIOD/2;
/*			��Ԫ������ڶ���									
			��һ����Ԫ��										*/	
	norm=1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
/*			������Ԫ����﷽ʽ����ת����				*/	
	
	float q0q0 = q0*q0;								//������Ԫ�����
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
/*			������Ԫ����﷽ʽ����ת����				*/
    QE_b[0][0] = 1 - (2*q2q2 + 2*q3q3);
    QE_b[0][1] = 2*q1q2 - 2*q0q3;
    QE_b[0][2] = 2*q1q3 + 2*q0q2;
		
    QE_b[1][0] = 2*q1q2 + 2*q0q3;
    QE_b[1][1] = 1 - (2*q1q1 + 2*q3q3);
    QE_b[1][2] = 2*q2q3 - 2*q0q1;

    QE_b[2][0] = 2*q1q3 - 2*q0q2;
    QE_b[2][1] = 2*q2q3 + 2*q0q1;
    QE_b[2][2] = 1 - (2*q1q1 + 2*q2q2);
/*			��Ԫ�����������									
			���ŷ���ǣ���Ҫ����ʵ��������Ӹ��ţ�										*/
	attribute->pitch = -asin( QE_b[2][0] ) * 57.2957795f;
	attribute->roll = atan2( QE_b[2][1] , QE_b[2][2] ) * 57.2957795f;
	attribute->yaw =  atan2( QE_b[1][0] , QE_b[0][0] ) * 57.2957795f;	
	//yaw��������λ�����岻ͬ��BMI088��Ϊ��ʱ��Ϊ����������λ��Э��˳ʱ��Ϊ����
}

























