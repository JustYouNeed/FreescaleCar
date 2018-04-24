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
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  一维卡尔曼滤波结构体  */
typedef struct 
{
	double Q;
	double R;
	double P;
	double Kg;
	double Output;
}Kalman1Dim_TypeDef;

void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length);
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalman_struct, double Q, double R);
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input);

# endif

/********************************************  END OF FILE  *******************************************/

