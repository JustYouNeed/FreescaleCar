/**
  *******************************************************************************************************
  * File Name: app_filter.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 本文件声明了各种滤波函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __APP_FILTER_H
# define __APP_FILTER_H

/*  一维卡尔曼滤波结构体  */
typedef struct 
{
	double Q;
	double R;
	double P;
	double Kg;
	double Output;
}Kalman1Dim_TypeDef;


typedef struct
{
	double X[2];			/*  卡尔曼的状态X,对于姿态解算来说,X[0]就是姿态角  */
	int16_t Gyro;			/*  最优角速度  */
	double A[2][2];		/*  A为系统为k-1到k时刻的状态转移矩阵  */
	double B[2];			/*  B为系统参数  */
	double P[2][2];		/*  P为系统协方差  */
	double Q[2][2];		/*  Q为系统噪声  */
	double R;					/*  测量噪声  */
	double Kg[2];			/*  Kg为系统增益  */	
	double dt;				/*  滤波器采样时间  */
}Kalman_TypeDef;

void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length);
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalman_struct, double Q, double R);
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input);
void filter_KanlmanInit(Kalman_TypeDef *Kalman);
void filter_KalmanFilter(Kalman_TypeDef *Kalman, double Gyro, double AccAngle);
# endif

/********************************************  END OF FILE  *******************************************/

