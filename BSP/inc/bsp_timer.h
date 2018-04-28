/**
  *******************************************************************************************************
  * File Name: bsp_timer.h
  * Author: Vector
  * Version: V1.1.1
  * Date: 2018-2-11
  * Brief: 本文件声明了有关软件定时器的一些变量
  *******************************************************************************************************
  * History
	*		1.Date: 2018-2-11
	* 		Author: Vector
  *			Mod: 建立文件
	*			
	*		2.Date: 2018-2-27
	*			Author: Vector
	*			Mod: 1.修改函数名bsp_tim_CreateSoftTimer为bsp_tim_CreateSoftTimer
	*					 2.修改函数名bsp_tim_DeleteTimer为bsp_tim_DeleteSoftTimer
	*					 3.新增函数bsp_tim_CreateHardTimer声明,创建硬件定时器函数,硬件定时器采用PIT定时器
	*				   4.新增函数bsp_tim_DeleteHardTimer声明,删除硬件定时器
	*          4.新增结构体HardTimer_Str,硬件定时器管理结构体
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_TIMER_H
# define __BSP_TIMER_H

# include "derivative.h"

# define SOFT_TIMER_COUNT			6

typedef void (*_cbTimerCallBack)(void);		/*  软件定时器回调函数数据类型  */

/*  软件定时器控制结构体  */
typedef struct	
{
	volatile uint8_t v_ucMode;		/*  模式  */
	volatile uint8_t v_ucFlag;		/*  定时到达标志  */
	volatile uint32_t v_uiCount;	/*  定时计数器  */
	volatile uint32_t v_uiPreLoad;	/*  重装载值  */
	
	uint8_t ucUsed;					/*  是否已经使用  */
	_cbTimerCallBack _cbTimer;	/*  回调函数  */
	
}SoftTimer_TypeDef;

typedef struct
{
	uint8_t ucUsed;							/*  是否已经使用  */
	_cbTimerCallBack _cbTimer;	/*  回调函数  */
}HardTimer_TypeDef;


typedef struct
{
	uint16_t Hours, Minutes, Seconds;
	uint8_t Years, Months, Days;
}Time_TypeDef;

/*  软件定时器模式枚举变量  */
typedef enum
{
	TIMER_MODE_ONCE = 0x00,
	TIMER_MODE_AUTO
}TIMER_MODE_ENUM;

extern Time_TypeDef SysTime;

void bsp_tim_SoftConfig(void);  /* 初始化软件定时器 */
int8_t bsp_tim_CreateSoftTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer, TIMER_MODE_ENUM eMode);
int8_t bsp_tim_CreateHardTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer);
int8_t bsp_tim_DeleteSoftTimer(uint8_t ucTimerId);
int8_t bsp_tim_DeleteHardTimer(uint8_t ucTimerId);
_cbTimerCallBack bsp_tim_SetTimerCB(uint8_t TimerId, _cbTimerCallBack  _cbTimer);
int8_t bsp_tim_TimerCheck(uint8_t ucTimerId);
int32_t bsp_tim_GetRunTime(void);
int32_t bsp_tim_CheckRunTime(int32_t iLastTime);
_cbTimerCallBack bsp_tim_GetCB(uint8_t ucTimerId);

void bsp_tim_DelayMs(uint16_t ui_nMs);
void bsp_tim_DelayUs(uint16_t ui_nUs);

# endif

/********************************************  END OF FILE  *******************************************/

