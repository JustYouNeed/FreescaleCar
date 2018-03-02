/**
  *******************************************************************************************************
  * File Name: drv_pit.h
  * Author: Vector
  * Version: V1.1.1
  * Date: 2018-2-1
  * Brief: 该文件对与PIT相关的外设进行了声明,同时声明了操作PIT的函数、结构体、枚举变量等
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *		  Author: Vector
  *		  Mod: 建立文件
	*		
	*		1.Date: 2018-2-27
	*			Author:	Vector
	*			Mod: 1.为PIT_InitTypeDef结构体添加新变量,PIT_IRQPriority
						 2.改正定时器通道命名错误
  *
  *******************************************************************************************************
  */


# ifndef __DRV_PIT_H
# define __DRV_PIT_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

/*  函数指针  */
typedef void (*_cb_PIT_IRQHandler)(void);


/*  PIT定时器初始化结构体  */
typedef struct 
{
	uint8_t PIT_Channel;		/*  通道  */
	uint8_t PIT_IRQCmd;			/*  中断使能  */
	uint32_t PIT_Period;		/*  定时周期  */
	uint8_t PIT_IRQPriority;	/*  中断优先级  */
	_cb_PIT_IRQHandler PIT_Callback;		/*  中断回调函数指针  */
}PIT_InitTypeDef;

/*  PIT定时器通道枚举变量  */
typedef enum
{
	PIT_Channel_0 = 0,
	PIT_Channel_1 = 1,
}PITChannel_TypeDef;


/*  供外部使用的PIT操作函数  */
void drv_pit_Init(PIT_InitTypeDef *PIT_InitStruct);
_cb_PIT_IRQHandler drv_pit_SetCallback(uint8_t PITx, _cb_PIT_IRQHandler PIT_Callback);
uint32_t drv_pit_GetCounter(uint8_t PITx);
void drv_pit_Stop(uint8_t PITx);

# endif

/********************************************  END OF FILE  *******************************************/
