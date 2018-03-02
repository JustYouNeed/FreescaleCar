/**
  *******************************************************************************************************
  * File Name: drv_wdog.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: ���ļ��������йز������Ź��ĺ���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-17
	*			Mod: �����ļ�,����δͨ��
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
* Description: ���Ź���׼��ʼ������
*             
* Arguments  : 1> WDOG_InitStruct: ���Ź���ʼ���ṹ��
*
* Reutrn     : None.
*
* Note(s)    : �ú�����drv_wdog_SampleInit��ͬ,�и�Ϊȫ�������,ͬʱ���øú�������Ҫʹ�ܿ��Ź���������
*********************************************************************************************************
*/
void drv_wdog_StdInit(WDOG_InitTypeDef *WDOG_InitStruct)
{
	uint16_t cnt = 0;
	uint8_t reg = 0;
	
	DISABLE_INT();
	
//	WDOG_CNT = 0x20c5;
//	WDOG_CNT = 0x28D9;
	
	/*  ����װ��ֵ  */
	switch(WDOG_InitStruct->WDOG_ClockSource)
	{
		case WDOG_ClockSource_BusClk: cnt = (SystemBusClock / 1000) * WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_LPOClk: cnt = WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_ICSClk: cnt = 32 * WDOG_InitStruct->WDOG_Period;break;
		case WDOG_ClockSource_ExtClk: cnt = 10000;break;
	}
	
	/*  д�����ֵ  */
	WDOG_TOVAL = cnt;
	
	/*  ���ڿ��Ź���Ҫд�봰��ֵ  */
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
* Description: �򵥳�ʼ�����Ź�
*             
* Arguments  : 1> period: ι������
*
* Reutrn     : None.
*
* Note(s)    : �ú���ֻ�ܳ�ʼ�����Ź�Ϊ�������Ź�
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
* Description: ι��
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
* Description: ʹ�ܿ��Ź�
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ʹ�ܿ��Ź���,ֻ�ܸ�λ����ֹͣ���Ź�
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
* Description: ���Ź��жϿ��غ���
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

