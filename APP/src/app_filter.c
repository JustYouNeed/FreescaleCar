/**
  *******************************************************************************************************
  * File Name: app_filter.c
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-2
  * Brief: 本文件提供了各种滤波函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-3-2
	*			Mod: 建立文件
	*
	*		2.Author: Vector
	*			Data: 2018-4-20
	*			Mod: 增加卡尔曼滤波函数
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"

/*
*********************************************************************************************************
*                            filter_SildingAverage              
*
* Description: 滑动均值滤波
*							 优点: 1.对周期性干扰性有良好的抵制作用,平滑度高,适用于调频振荡的系统
*              缺点: 1.灵敏度低
*										 2.对偶然出现的脉冲性干扰的抵制作用较差
*										 3.不易消除由于脉冲干扰引起的采样值偏差
*										 4.不适用于脉冲干扰比较严重的场合
*										 5.比较浪费RAM
* Arguments  : 1> Array[]: 数据缓存区
*              2> Average: 滤波后的值
*              3> Length: 数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length)
{
	uint16_t cnt = 0;
	uint32_t sum = 0;
	
	/*  计算和  */
	for(; cnt < Length; cnt ++)
	{
		sum += Array[cnt];
	}
	*Average = (uint16_t)(sum / Length);
}
/*
*********************************************************************************************************
*                                   filter_Kalman1Dim_Init       
*
* Description: 初始化卡尔曼结构体参数
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalam_struct, double Q, double R)
{
	Kalam_struct->Kg = 0;              //卡尔曼增益
	Kalam_struct->Output = 0;          //输出量
	Kalam_struct->P = 0;               //协方差
	Kalam_struct->Q = Q;               //系统过程噪声的协方差/预测值置信度
	Kalam_struct->R = R;               //系统测量噪声的协方差/测量值置信度
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input)
{
	Kalam_Struct->P += Kalam_Struct->Q;
	
	Kalam_Struct->Output += Kalam_Struct->Kg*(input - Kalam_Struct->Output);
	
	Kalam_Struct->Kg = Kalam_Struct->P/(Kalam_Struct->P + Kalam_Struct->R);
	
	Kalam_Struct->P *= (1-Kalam_Struct->Kg);
}

/*
*********************************************************************************************************
*                       filter_KanlmanInit                   
*
* Description: 初始化卡尔曼滤波结构体
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_KanlmanInit(Kalman_TypeDef *Kalman)
{
	/*  初始化滤波器输出为0  */
	Kalman->X[0] = 0;
	Kalman->X[1] = 0;
	
	/*  滤波器采样周期  */
	Kalman->dt = 0.005;
	
	/*  滤波器状态转移矩阵  */
	Kalman->A[0][0] = 1;
	Kalman->A[0][1] = -Kalman->dt;
	Kalman->A[1][0] = 0;
	Kalman->A[1][1] = 1;
	
	/*  控制输入转移矩阵  */
	Kalman->B[0] = Kalman->dt;
	Kalman->B[1] = 0;
	
	/*  协方差矩阵  */
	Kalman->P[0][0] = 1;
	Kalman->P[0][1] = 0;
	Kalman->P[1][0] = 0;
	Kalman->P[1][1] = 1;
	
	/*  预测值的置信度  */
	Kalman->Q[0][0] = 0.001;
	Kalman->Q[0][1] = 0;
	Kalman->Q[1][0] = 0;
	Kalman->Q[1][1] = 0.003;
	
	/*  测量过程噪声  */
	Kalman->R = 0.5;
}

/*
*********************************************************************************************************
*                       filter_KalmanFilter                   
*
* Description: 卡尔曼滤波函数
*             
* Arguments  : 1.Kalman: Kalman控制结构体
*							 2.Gyro: 测量到的角速度
*							 3.AccAngle: 测量到的加速度计算出的角度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_KalmanFilter(Kalman_TypeDef *Kalman, double Gyro, double AccAngle)
{	
	/*  公式1,X(k|k-1) = AX(k-1|k-1) + BU(k)  X, A, B, 都为矩阵, 进行先验估计  */
	Kalman->X[0] = (Kalman->A[0][0] * Kalman->X[0] + Kalman->A[0][1] * Kalman->X[1]) + Gyro * Kalman->B[0];
	Kalman->X[1] = (Kalman->A[1][0] * Kalman->X[0] + Kalman->A[1][1] * Kalman->X[1]) + Gyro * Kalman->B[1];
	
	/*  公式2, P(k|k-1) = AP(k-1|k-1)A_T + Q */
	Kalman->P[0][0] = Kalman->P[0][0] - Kalman->P[1][0]*Kalman->dt - Kalman->P[0][1]*Kalman->dt + Kalman->P[0][0] * Kalman->dt * Kalman->dt + Kalman->Q[0][0];
	Kalman->P[0][1] = Kalman->P[0][1] - Kalman->P[1][1] * Kalman->dt + Kalman->Q[0][1];
	Kalman->P[1][0] = Kalman->P[1][0] - Kalman->P[1][1] * Kalman->dt + Kalman->Q[1][0];
	Kalman->P[1][1] = Kalman->P[1][1] + Kalman->Q[1][1];
	
	/*  公式3, Kg(k) = P(k|k-1)H_T/(HP(k|k-1)H_T + R), H为系数矩阵 H = | 1 0 |  */
	Kalman->Kg[0] = Kalman->P[0][0] / (Kalman->P[0][0] + Kalman->R);
	Kalman->Kg[1] = Kalman->P[1][0] / (Kalman->P[0][0] + Kalman->R);
	
	/*  公式4, X(k|k) = X(k|k-1) + Kg(k)(Z(k) - H*X(k|k-1)), Z(k)为系统测量输入  */
	Kalman->X[0] = Kalman->X[0] + Kalman->Kg[0] * (AccAngle - Kalman->X[0]);
//	Kalman->X[1] = Kalman->X[1] + Kalman->Kg[1] * (Gyro - Kalman->X[1]);
	Kalman->Gyro = Gyro - Kalman->X[1];
	
	/*  公式5, P(k|k) = (I - Kg(k)*H)P(k|k-1)  */
	Kalman->P[0][0] = Kalman->P[0][0] * (1 - Kalman->Kg[0]);
	Kalman->P[0][1] = Kalman->P[0][1] * (1 - Kalman->Kg[0]);
	Kalman->P[1][0] = Kalman->P[1][0] - Kalman->P[0][0] * Kalman->Kg[1];
	Kalman->P[1][1] = Kalman->P[1][1] - Kalman->P[0][1] * Kalman->Kg[1];
}



/********************************************  END OF FILE  *******************************************/

