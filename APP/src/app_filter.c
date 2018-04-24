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
# include "app_filter.h"

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
	Kalam_struct->Kg = 0;
	Kalam_struct->Output = 0;
	Kalam_struct->P = 0;
	Kalam_struct->Q = Q;
	Kalam_struct->R = R;
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



/********************************************  END OF FILE  *******************************************/

