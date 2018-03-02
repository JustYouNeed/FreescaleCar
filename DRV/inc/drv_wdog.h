/**
  *******************************************************************************************************
  * File Name: drv_wdog.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-27
  * Brief: 本文件声明了有关看门狗操作的函数、变量以及宏定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-17
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __DRV_WDOG_H
# define __DRV_WDOG_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

# define WDOG_UpdateEnable		((uint8_t)0x01)
# define WDOG_UpdateDisable		((uint8_t)0x00)

# define WDOG_DebugEnable			((uint8_t)0x01)
# define WDOG_DebugDisable		((uint8_t)0x00)

# define WDOG_WaitEnable  		((uint8_t)0x01)
# define WDOG_WaitDisable 		((uint8_t)0x00)

# define WDOG_StopEnable	  	((uint8_t)0x01)
# define WDOG_StopDisable 		((uint8_t)0x00)

# define WDOG_PrescalerEnable	    ((uint8_t)0x01)
# define WDOG_PrescalerDisable		((uint8_t)0x00)

# define WDOG_ClockSource_BusClk	((uint8_t)0x00)
# define WDOG_ClockSource_LPOClk	((uint8_t)0x01)
# define WDOG_ClockSource_ICSClk	((uint8_t)0x02)
# define WDOG_ClockSource_ExtClk	((uint8_t)0x03)

# define WDOG_WorkMode_Window			  ((uint8_t)0x01)
# define WDOG_WorkMode_Independent	((uint8_t)0x00)

typedef struct
{
	uint8_t WDOG_UpdateCmd;		/*  是否允许更新  */
	uint8_t WDOG_DebugCmd;		/*  Debug模式使能  */
	uint8_t WDOG_WaitCmd;			/*  Wait模式使能  */
	uint8_t WDOG_StopCmd;			/*  Stop模式使能  */
	uint8_t WDOG_WorkMode;		/*  工作模式  */
	uint8_t WDOG_PrescalerCmd;/*  是否开启预分频  */
	uint8_t WDOG_ClockSource;	/*  时钟源  */
	uint8_t WDOG_Period;			/*  周期  */
}WDOG_InitTypeDef;

void drv_wdog_StdInit(WDOG_InitTypeDef *WDOG_InitStruct);
void drv_wdog_SampleInit(uint16_t period);
void drv_wdog_Feed(void);
void drv_wdog_Enable(void);
void drv_wdog_IRQCmd(FunctionalState NewState);


# endif


/********************************************  END OF FILE  *******************************************/
