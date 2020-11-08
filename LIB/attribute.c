#include "attribute.h"

#define ATTRIBUTE_KP	5.0f
#define ATTRIBUTE_KI	0.1f
#define ATTRIBUTE_KD	0.01f
#define CAL_PERIOD		0.001f

float QE_b[3][3];//四元数等效旋转矩阵
/*			算法参考链接
			https://blog.csdn.net/guanjianhe/article/details/95608801
			捷联惯导算法与组合导航								
			四元数解欧拉角法									
			本套结算代码没有使用STM32的DSP加速					*/
void attributeCal(accData_p accData,gyroData_p gyroData,attribute_p attribute)
{
	static float 	q0=1,	q1=0,	q2=0,	q3=0;					//四元数定义，静态变量
	static float errorXInit=0,errorYInit=0,errorZInit=0;		//三轴误差积分
	float 	vX,vY,vZ;											//物体坐标系下重力分量
	float 	errorX,errorY,errorZ;								//三轴误差向量
	float	norm;												//归一化用的临时变量		
	

	
	float q0_last = q0;										//旧四元数更新									
	float q1_last = q1;
	float q2_last = q2;
	float q3_last = q3;
		

	
/*			陀螺仪误差消除第一步								
			三轴加速度计归一化									
			为了提高速度，多用乘法								*/
	norm=1/sqrt(accData->accX * accData->accX + accData->accY * 
				accData->accY + accData->accZ * accData->accZ);
	accData->accX *= norm;
	accData->accY *= norm;
	accData->accZ *= norm;
/*			陀螺仪误差消除第二步								
			物体坐标系下重力分量								*/
	vX = QE_b[2][0];
	vY = QE_b[2][1];
	vZ = QE_b[2][2];
/*			陀螺仪误差消除第三步								
			利用归一化加速度和重力分量叉积得到姿态误差			*/
	errorX = accData->accY * vZ - accData->accZ * vY;
	errorY = accData->accZ * vX - accData->accX * vZ;
	errorZ = accData->accX * vY - accData->accY * vX;
/*			陀螺仪误差消除第四步								
			误差积分											*/
	errorXInit += errorX * ATTRIBUTE_KI * CAL_PERIOD;
	errorYInit += errorY * ATTRIBUTE_KI * CAL_PERIOD;
	errorZInit += errorZ * ATTRIBUTE_KI * CAL_PERIOD;
/*			陀螺仪误差消除第五步								
			互补滤波											*/
	gyroData->gyroX_rad += ATTRIBUTE_KP * ( errorXInit + errorX );
	gyroData->gyroY_rad += ATTRIBUTE_KP * ( errorYInit + errorY );
	gyroData->gyroZ_rad += ATTRIBUTE_KP * ( errorZInit + errorZ );
/*			四元数解算第一步									
			更新四元数											*/
	q0=q0_last+(-gyroData->gyroX_rad * q1_last - gyroData->gyroY_rad * q2_last - gyroData->gyroZ_rad * q3_last) * CAL_PERIOD/2;
	q1=q1_last+( gyroData->gyroX_rad * q0_last - gyroData->gyroY_rad * q3_last + gyroData->gyroZ_rad * q2_last) * CAL_PERIOD/2;
	q2=q2_last+( gyroData->gyroX_rad * q3_last + gyroData->gyroY_rad * q0_last - gyroData->gyroZ_rad * q1_last) * CAL_PERIOD/2;
	q3=q3_last+(-gyroData->gyroX_rad * q2_last + gyroData->gyroY_rad * q1_last + gyroData->gyroZ_rad * q0_last) * CAL_PERIOD/2;
/*			四元数解算第二步									
			归一化四元数										*/	
	norm=1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
/*			更新四元数表达方式的旋转矩阵				*/	
	
	float q0q0 = q0*q0;								//常用四元数组合
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
/*			更新四元数表达方式的旋转矩阵				*/
    QE_b[0][0] = 1 - (2*q2q2 + 2*q3q3);
    QE_b[0][1] = 2*q1q2 - 2*q0q3;
    QE_b[0][2] = 2*q1q3 + 2*q0q2;
		
    QE_b[1][0] = 2*q1q2 + 2*q0q3;
    QE_b[1][1] = 1 - (2*q1q1 + 2*q3q3);
    QE_b[1][2] = 2*q2q3 - 2*q0q1;

    QE_b[2][0] = 2*q1q3 - 2*q0q2;
    QE_b[2][1] = 2*q2q3 + 2*q0q1;
    QE_b[2][2] = 1 - (2*q1q1 + 2*q2q2);
/*			四元数解算第三步									
			求解欧拉角，需要根据实际情况增加负号，										*/
	attribute->pitch = -asin( QE_b[2][0] ) * 57.2957795f;
	attribute->roll = atan2( QE_b[2][1] , QE_b[2][2] ) * 57.2957795f;
	attribute->yaw =  atan2( QE_b[1][0] , QE_b[0][0] ) * 57.2957795f;	
	//yaw与匿名上位机定义不同，BMI088认为逆时针为正，匿名上位机协议顺时针为正。
}

























