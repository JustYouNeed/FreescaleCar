/**
  *******************************************************************************************************
  * File Name: bsp_timer.c
  * Author: Vector
  * Version: V1.2.2
  * Date: 2018-2-11
  * Brief: 本文件对滴答定时器进行了一定程度的封装,同时建立了软件定时器,每个定时器可有一个回调函数
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: 建立文件
	*		
	*		2.Author: Vector
	*			Date:	2018-2-17
	*			Mod:	1.修复定时器标志位问题	
	*						2.修复定时器创建失败问题
	*						
	*		3.Author: Vector
	*			Date: 2018-2-27
	*			Mod: 1.修改函数名bsp_tim_CreateSoftTimer为bsp_tim_CreateSoftTimer
	*					 2.修改函数名bsp_tim_DeleteTimer为bsp_tim_DeleteSoftTimer
	*					 3.优化部分函数,防止出现数组越界的可能
	*          4.新增函数bsp_tim_CreateHardTimer定义,创建硬件定时器函数,硬件定时器采用PIT定时器
	*          5.新增函数bsp_tim_DeleteHardTimer定义,删除硬件定时器
	*          6.新增HardTimer[]硬件定时器管理数组
	*
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_timer.h"

static volatile uint32_t s_uiDelayCount = 0;	/*  该文件私有变量,用于延时  */
static volatile uint8_t s_ucTimeOutFlag = 0;	/*  该文件私有变量,用于统计运行时间  */

__IO int32_t g_iRunTime = 0;		/*  该文件私有变量,运行时间  */


SoftTimer_Str SoftTimer[SOFT_TIMER_COUNT];	/*  软件定时器组  */
HardTimer_Str HardTimer[2];									/*  由于采用PIT定时器,因此硬件定时器最多只能有两个  */

/*
*********************************************************************************************************
*                               bsp_tim_SoftDec           
*
* Description: 递减每个软件定时器的计数器
*             
* Arguments  : 1> pTimer:SoftTimer_Str结构体指针，指向一个软件定时器
*
* Reutrn     : None.
*
* Note(s)    : 该函数为本文件私有函数
*********************************************************************************************************
*/
void bsp_tim_SoftDec(SoftTimer_Str * pTimer)
{
	if(pTimer->v_uiCount >0)		/*  只有在计数器的值大于零才需要递减  */
	{
		if(--pTimer->v_uiCount ==0 )  /*  时间到后标志置位  */
		{
			pTimer->v_ucFlag = 1;
			
			
			if(pTimer->v_ucMode == TIMER_MODE_AUTO)			/*  定时器模式为自动重装载时重装计数器  */
			{
				pTimer->v_uiCount = pTimer->v_uiPreLoad;
				pTimer->v_ucFlag = 0;
			}
			
			if(pTimer->_cbTimer)			/*  设置回调函数时调用回调函数  */
			{
				pTimer->_cbTimer();
			}
		}/*  end  if(--pTimer->v_uiCount ==0 ) */
	}	/*  end if(pTimer->v_uiCount >0)  */
}

/*
*********************************************************************************************************
*                                   SysTick_ISR       
*
* Description: 滴答定时器中断服务函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void SysTick_ISR(void)
{
# if OS_SUPPORT < 1u
	uint8_t i = 0;
# endif
	
	if(s_uiDelayCount > 0)		/*  延时计数器  */
	{
		if(-- s_uiDelayCount == 0) 
			s_ucTimeOutFlag = 1;			/*  延时完成  */
	}
	
	g_iRunTime ++;				/*  运行时间计数器  */
	if(g_iRunTime == 0x7fffffff) g_iRunTime = 0;			/*  运行时间计数器为32位，最大值为 0x7fffffff */
	
# if OS_SUPPORT > 0u			/*  如果需要支持操作系统  */
	if(OS_RUNNING == 1)
	{
		OSIntEnter();						//进入中断
		OSTimeTick();       				//调用ucos的时钟服务程序               
		OSIntExit();       	 				//触发任务切换软中断
	}
# else
	for(i = 0; i < SOFT_TIMER_COUNT; i++)		/*  递减软件定时器的值  */
	{
		bsp_tim_SoftDec(&SoftTimer[i]);
	}
# endif
	
}

/*
*********************************************************************************************************
*                                SysTick_Handler          
*
* Description: 滴答定时器中断
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void SysTick_Handler(void)
{
	SysTick_ISR();
}


/*
*********************************************************************************************************
*                                 bsp_tim_SoftConfig         
*
* Description: 初始化软件定时器
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_tim_SoftConfig(void)
{
	uint8_t i = 0;
	
	for(i = 0; i < SOFT_TIMER_COUNT; i++)
	{
		SoftTimer[i].v_uiCount = 0;
		SoftTimer[i].v_uiPreLoad = 0;
		SoftTimer[i].v_ucFlag = 0;
		SoftTimer[i].v_ucMode = 0;
		SoftTimer[i]._cbTimer = 0;
		SoftTimer[i].ucUsed = 0;
	}
	
	SysTick_Config(SystemCoreClock / 1000);
}


/*
*********************************************************************************************************
*                             bsp_tim_CreateSoftTimer             
*
* Description: 创建一个软件定时器
*             
* Arguments  : 1> ucTimerId:软件定时器ID
*              2> uiPeriod:定时周期
*              3> _cbTimer:定时器回调函数
*              4> eMode:定时器模式
*
* Reutrn     : 1> 0: 成功
*              2> 其他: 错误代码
*
* Note(s)    : 1.软件定时器个数有限制，当传入ID超过最大定时器数量时将会创建失败
*              2.软件定时器不能覆盖已经使用的定时器，需要将原来的删除后才能创建
*							 3.定时器回调函数执行时间应尽可能短,且不能用延时函数
*********************************************************************************************************
*/
int8_t bsp_tim_CreateSoftTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer, TIMER_MODE_ENUM eMode)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;				/*  超过定时器数量  */
	if(SoftTimer[ucTimerId].ucUsed == 1) return -2;		/*  已经使用了的定时器不能再次创建  */
	
	SoftTimer[ucTimerId].v_uiCount = uiPeriod;		/*  设置定时周期  */
	SoftTimer[ucTimerId].v_ucFlag = 0;		/*  清除定时标志位  */
	SoftTimer[ucTimerId].v_ucMode = eMode;		/*  定时模式  */
	SoftTimer[ucTimerId].ucUsed = 1;	/*  已经使用  */
	SoftTimer[ucTimerId].v_uiPreLoad = uiPeriod;	/*  软件重载值  */
	SoftTimer[ucTimerId]._cbTimer = _cbTimer;		/*  定时器回调函数  */
	
	return 0;
}

/*
*********************************************************************************************************
*                                   bsp_tim_DeleteSoftTimer       
*
* Description: 删除一个软件定时器
*             
* Arguments  : 1> ucTimerId:软件定时器ID
*
* Reutrn     : 1> 0: 删除成功
*              2> 其他: 错误代码
*
* Note(s)    : 未使用的定时器不能被删除
*********************************************************************************************************
*/
int8_t bsp_tim_DeleteSoftTimer(uint8_t ucTimerId)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;
	if(SoftTimer[ucTimerId].ucUsed == 0)  return -2;	
	
	SoftTimer[ucTimerId].v_ucMode = 0;
	SoftTimer[ucTimerId].v_uiPreLoad = 0;
	SoftTimer[ucTimerId].v_uiCount = 0;
		
	return 0;
}

/*
*********************************************************************************************************
*                             bsp_tim_CreateHardTimer             
*
* Description: 创建一个硬件定时器
*             
* Arguments  : 1> ucTimerId: 硬件定时器ID 
*              2> uiPeriod: 定时周期
*              3> _cbTimer: 定时器回调函数
*
* Reutrn     : 1> 0: 成功
*              2> 其他: 错误代码
*
* Note(s)    : 1.软件定时器个数有限制，当传入ID超过最大定时器数量时将会创建失败
*              2.软件定时器不能覆盖已经使用的定时器，需要将原来的删除后才能创建
*							 3.定时器回调函数执行时间应尽可能短,且不能用延时函数
*********************************************************************************************************
*/
int8_t bsp_tim_CreateHardTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer)
{
	PIT_InitTypeDef PIT_InitStruct;
	
	if(ucTimerId > 2) return -1;		/*  最多只能有两个定时器  */
	if(HardTimer[ucTimerId].ucUsed == 1)	return -2;	/*  该定时器已经被使用了  */
	
	PIT_InitStruct.PIT_Channel = ucTimerId;
	PIT_InitStruct.PIT_IRQCmd = ENABLE;
	PIT_InitStruct.PIT_IRQPriority = 0x03;
	PIT_InitStruct.PIT_Period = uiPeriod;
	PIT_InitStruct.PIT_Callback = _cbTimer;
	drv_pit_Init(&PIT_InitStruct);
	
	HardTimer[ucTimerId].ucUsed = 1;
	HardTimer[ucTimerId]._cbTimer = _cbTimer;
	return 0;
}


/*
*********************************************************************************************************
*                                 bsp_tim_DeleteHardTimer         
*
* Description: 删除一个硬件定时器
*             
* Arguments  : 1> ucTimerId: 定时器ID
*
* Reutrn     : 1> 0: 成功
*              2> -1: 无效定时器ID
*              3> -2: 该定时器未使用
*
* Note(s)    : None.
*********************************************************************************************************
*/
int8_t bsp_tim_DeleteHardTimer(uint8_t ucTimerId)
{
	if(ucTimerId > 2) return -1;		/*  无效定时器ID  */
	if(HardTimer[ucTimerId].ucUsed == 0) return -2;	/*  该定时器未使用  */
	
	drv_pit_Stop(ucTimerId);
	
	/*  清零定时器  */
	HardTimer[ucTimerId].ucUsed = 0;
	HardTimer[ucTimerId]._cbTimer = 0;
	
	return 0;
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
_cbTimerCallBack  bsp_tim_SetTimerCB(uint8_t TimerId, _cbTimerCallBack  _cbTimer)
{
	_cbTimerCallBack _cbTimerTemp;
	_cbTimerTemp = SoftTimer[TimerId]._cbTimer;
	
	SoftTimer[TimerId]._cbTimer = _cbTimer;
	
	return _cbTimerTemp;
}

/*
*********************************************************************************************************
*                              bsp_tim_TimerCheck            
*
* Description: 检查软件定时器是否到达定时时间
*             
* Arguments  : 1> ucTimerId:定时器ID
*
* Reutrn     : 1> 0: 定时时间到
*              2> 1: 未到定时时间
*
* Note(s)    : None.
*********************************************************************************************************
*/
int8_t bsp_tim_TimerCheck(uint8_t ucTimerId)
{
	if(SoftTimer[ucTimerId].ucUsed == 0) return -1;
	
	if(ucTimerId > SOFT_TIMER_COUNT) return -2;
	
	return (SoftTimer[ucTimerId].v_ucFlag == 1)?0:1;
}


/*
*********************************************************************************************************
*                              bsp_tim_GetRunTime            
*
* Description: 获取系统运行时间
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int32_t bsp_tim_GetRunTime(void)
{	
	return g_iRunTime;
}
int32_t bsp_tim_CheckRunTime(int32_t iLastTime)
{
	int32_t nowTime;
	int32_t timeDiff;
		
	nowTime = g_iRunTime;
	
	if (nowTime >= iLastTime)
	{
		timeDiff = nowTime - iLastTime;
	}
	else
	{
		timeDiff = 0x7FFFFFFF - iLastTime + nowTime;
	}
	
	return timeDiff;
}


/*
*********************************************************************************************************
*                                  bsp_tim_DelayMs        
*
* Description: 毫秒级延时函数
*             
* Arguments  : 1> ui_nMs:要延时的时间
*
* Reutrn     : None.
*
* Note(s)    :  延时采用滴答定时器实现，因为滴答定时器为16位定时器，所以该最大延时时间为65534
*********************************************************************************************************
*/
void bsp_tim_DelayMs(uint16_t ui_nMs)
{
	if(ui_nMs == 0) return ;
	else if(ui_nMs == 1) ui_nMs = 2;
		
	s_uiDelayCount = ui_nMs;
	s_ucTimeOutFlag = 0;
		
	while(1)
	{
		if(s_ucTimeOutFlag == 1) break;
	}
}


/*
*********************************************************************************************************
*                                 bsp_tim_DelayUs         
*
* Description: 微秒级延时函数
*             
* Arguments  : 1> ui_nUs:延时时长，微秒
*
* Reutrn     : None.
*
* Note(s)    : 最大延时为65535微秒
*********************************************************************************************************
*/
void bsp_tim_DelayUs(uint16_t ui_nUs)
{
		uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
		reload = SysTick->LOAD;                
    ticks = ui_nUs * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* 刚进入时的计数器值 */

    while (1)
    {
			tnow = SysTick->VAL;    
			if (tnow != told)
			{    
				/* SYSTICK是一个递减的计数器 */    
				if (tnow < told)
				{
						tcnt += told - tnow;    
				}
				/* 重新装载递减 */
				else
				{
						tcnt += reload - tnow + told;    
				}        
				told = tnow;

				/* 时间超过/等于要延迟的时间,则退出 */
				if (tcnt >= ticks)
				{
					break;
				}
			}  
    }
}

/********************************************  END OF FILE  *******************************************/


