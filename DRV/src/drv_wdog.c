/**
  *******************************************************************************************************
  * File Name: drv_wdog.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: 本文件定义了有关操作看门狗的函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-17
	*			Mod: 建立文件,测试未通过
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_wdog.h"


/*
*********************************************************************************************************
*                           drv_wdog_StdInit               
*
* Description: 看门狗标准初始化函数
*             
* Arguments  : 1> WDOG_InitStruct: 看门狗初始化结构体
*
* Reutrn     : None.
*
* Note(s)    : 该函数和drv_wdog_SampleInit不同,有更为全面的设置,同时调用该函数后需要使能看门狗才能启动
*********************************************************************************************************
*/
void drv_wdog_StdInit(WDOG_InitTypeDef *WDOG_InitStruct)
{
	uint16_t cnt = 0;
	uint8_t reg = 0;
	
	DISABLE_INT();
	
//	WDOG_CNT = 0x20c5;
//	WDOG_CNT = 0x28D9;
	
	/*  计算装载值  */
	switch(WDOG_InitStruct->WDOG_ClockSource)
	{
		case WDOG_ClockSource_BusClk: cnt = (SystemBusClock / 1000) * WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_LPOClk: cnt = WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_ICSClk: cnt = 32 * WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_ExtClk: cnt = 10000;break;
	}
	
	/*  写入计数值  */
	WDOG_TOVAL = cnt;
	
	/*  窗口看门狗需要写入窗口值  */
	if(WDOG_InitStruct->WDOG_WorkMode == WDOG_WorkMode_Window)
	{
		cnt = (uint16_t)((cnt * 2) / 3);
		WDOG_WIN = cnt;
	}
	
	
	reg |= WDOG_InitStruct->WDOG_WorkMode << WDOG_CS2_WIN_SHIFT;
	reg |= WDOG_InitStruct->WDOG_PrescalerCmd << WDOG_CS2_PRES_SHIFT;
	reg |= WDOG_InitStruct->WDOG_ClockSource << WDOG_CS2_CLK_SHIFT;
	WDOG_CS2 = reg;
	
	reg = 0;
	reg |= WDOG_CS1_EN_MASK;
	reg |= WDOG_InitStruct->WDOG_UpdateCmd << WDOG_CS1_UPDATE_SHIFT;
	reg |= WDOG_InitStruct->WDOG_DebugCmd << WDOG_CS1_DBG_SHIFT;
	reg |= WDOG_InitStruct->WDOG_WaitCmd << WDOG_CS1_WAIT_SHIFT;
	reg |= WDOG_InitStruct->WDOG_StopCmd << WDOG_CS1_STOP_SHIFT;
	WDOG_CS1 = reg;
	
	ENABLE_INT();
}

/*
*********************************************************************************************************
*                                drv_wdog_SampleInit          
*
* Description: 简单初始化看门狗
*             
* Arguments  : 1> period: 喂狗周期
*
* Reutrn     : None.
*
* Note(s)    : 该函数只能初始化看门狗为独立看门狗
*********************************************************************************************************
*/
void drv_wdog_SampleInit(uint16_t period)
{
	WDOG_InitTypeDef WDOG_InitStruct;
	
	WDOG_InitStruct.WDOG_ClockSource = WDOG_ClockSource_BusClk;
	WDOG_InitStruct.WDOG_DebugCmd = WDOG_DebugEnable;
	WDOG_InitStruct.WDOG_StopCmd = WDOG_StopDisable;
	WDOG_InitStruct.WDOG_UpdateCmd = WDOG_UpdateDisable;
	WDOG_InitStruct.WDOG_WaitCmd = WDOG_WaitDisable;
	WDOG_InitStruct.WDOG_PrescalerCmd = WDOG_PrescalerDisable;
	WDOG_InitStruct.WDOG_Period = period;
	WDOG_InitStruct.WDOG_WorkMode = WDOG_WorkMode_Independent;
	drv_wdog_StdInit(&WDOG_InitStruct);
	
	drv_wdog_Enable();
}

/*
*********************************************************************************************************
*                              drv_wdog_Feed            
*
* Description: 喂狗
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_wdog_Feed(void)
{
	DISABLE_INT();
	
	WDOG_CNT = 0x02a6;
	WDOG_CNT = 0x80b4;
	
	ENABLE_INT();
}

/*
*********************************************************************************************************
*                         drv_wdog_Enable                 
*
* Description: 使能看门狗
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 使能看门狗后,只能复位才能停止看门狗
*********************************************************************************************************
*/
void drv_wdog_Enable(void)
{
	WDOG_CS1 |= WDOG_CS1_EN_MASK;
}

/*
*********************************************************************************************************
*                          drv_wdog_IRQCmd                
*
* Description: 看门狗中断开关函数
*             
* Arguments  : 1> NewState: ENABLE, or DISABLE
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_wdog_IRQCmd(FunctionalState NewState)
{
	WDOG_CS1 |= WDOG_CS1_INT_MASK;
}


/********************************************  END OF FILE  *******************************************/

