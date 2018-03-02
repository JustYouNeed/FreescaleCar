/**
  *******************************************************************************************************
  * File Name: drv_pit.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-1
  * Brief: KEA128芯片PIT定时器底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *     Author: Vector
  *     Mod: 建立文件,添加基本函数
	*	
	*		2.Date: 2018-2-27
	*			Author: Vector
	*			Mod: 修复定时器设置失败BUG
  *
  *******************************************************************************************************
  */

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_pit.h"

/*  静态变量,如果开启PIT定时器的时候有设置回调函数,
		则会存储在该回调函数指针数组里面,并在中断发生
		的时候执行.
*/
static _cb_PIT_IRQHandler PIT_Handler[2]; 

/*
*********************************************************************************************************
*                                     drv_pit_Init     
*
* Description: 初始化一个PIT定时器
*             
* Arguments  : 1> PIT_InitStruct: PIT定时器初始化结构体指针
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_pit_Init(PIT_InitTypeDef *PIT_InitStruct)
{
	uint32_t cnt = 0;
	
	drv_rcc_ClockCmd(RCC_PeriphClock_PIT, ENABLE);

	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	cnt = (SystemBusClock / 1000 ) * PIT_InitStruct->PIT_Period;
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].LDVAL = cnt;	/*  设置溢出中断时间  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TFLG |= PIT_TFLG_TIF_MASK;					/*  清除中断标志位  */
	
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL &= ~ PIT_TCTRL_TEN_MASK;       	/*  禁止PITx定时器,用于清空计数器  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL &= ~PIT_TCTRL_TIE_MASK;         /*  先关闭中断  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL  |= PIT_TCTRL_TEN_MASK;	/*  开启PITx定时器  */
	
	if(PIT_InitStruct->PIT_IRQCmd == ENABLE)		/*  如果开启了中断则配置相应位  */
	{
		PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL  |= PIT_TCTRL_TIE_MASK;		/*  开启中断  */
		PIT_Handler[PIT_InitStruct->PIT_Channel] = PIT_InitStruct->PIT_Callback;	/*  配置中断函数  */
		
		/*  配置相关中断  */
		switch(PIT_InitStruct->PIT_Channel)
		{
			case PIT_Channel_0: 
			{
				NVIC_EnableIRQ(PIT_CH0_IRQn);		/*  开启中断  */
				NVIC_SetPriority(PIT_CH0_IRQn, PIT_InitStruct->PIT_IRQPriority);	/*  设置中断优先级  */
			}break;
			case PIT_Channel_1: 
			{
				NVIC_EnableIRQ(PIT_CH1_IRQn); /*  开启中断  */
				NVIC_SetPriority(PIT_CH1_IRQn, PIT_InitStruct->PIT_IRQPriority); /*  设置中断优先级  */
			}break;
		}
	}
}

/*
*********************************************************************************************************
*                                   drv_pit_SetCallback       
*
* Description: 设置PIT定时器的中断回调函数
*             
* Arguments  : 1> PITx: 定时器编号,该芯片只有两个通道,因此该值只能为0或者1
*              2> PIT_Callback: 回调函数
*
* Reutrn     : None.
*
* Note(s)    : PITx只能取0或者1
*********************************************************************************************************
*/
_cb_PIT_IRQHandler drv_pit_SetCallback(uint8_t PITx, _cb_PIT_IRQHandler PIT_Callback)
{
	_cb_PIT_IRQHandler _cb_Last = PIT_Handler[PITx];	/*  用于返回之前的回调函数  */
	
	if(PITx > 1) return 0;		/*  定时器通道不存在  */
		
	PIT_Handler[PITx] = PIT_Callback;		/*  保存好回调函数  */
	
	return _cb_Last;
}

/*
*********************************************************************************************************
*                                    drv_pit_GetCounter      
*
* Description: 获取定时器计数器的值
*             
* Arguments  : 1> PITx: 定时器编号,该芯片只有两个通道,因此该值只能为0或者1
*
* Reutrn     : 计数器的值
*
* Note(s)    : PITx只能取0或者1
*********************************************************************************************************
*/
uint32_t drv_pit_GetCounter(uint8_t PITx)
{
	uint32_t val;
	val = (uint32_t)(~0) - PIT->CHANNEL[PITx].CVAL;
	if(PIT->CHANNEL[PITx].TFLG &  PIT_TFLG_TIF_MASK)	/*  判断时间是否超时  */
	{
		PIT->CHANNEL[PITx].TFLG |= PIT_TFLG_TIF_MASK;		/*  清除中断标志位  */
		PIT->CHANNEL[PITx].TCTRL &= ~ PIT_TCTRL_TEN_MASK;		/*  禁止定时器  */
		return 0XFFFFFFFF;									
	}
	return val;
}

/*
*********************************************************************************************************
*                                      drv_pit_Stop    
*
* Description: 停止一个定时器
*             
* Arguments  : 1> PITx: 定时器编号,该芯片只有两个通道,因此该值只能为0或者1
*
* Reutrn     : None.
*
* Note(s)    : PITx只能取0或者1
*********************************************************************************************************
*/
void drv_pit_Stop(uint8_t PITx)
{
	if(PITx > 1) return ;		/*  定时器通道不存在  */
	
	PIT->CHANNEL[PITx].TFLG |= PIT_TFLG_TIF_MASK;		/*  清除中断标志位  */
	PIT->CHANNEL[PITx].TCTRL &= ~ PIT_TCTRL_TEN_MASK;		/*  禁止定时器  */
}


/*
*********************************************************************************************************
*                                     PIT_CH0_IRQHandler     
*
* Description: PIT定时器通道0中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void PIT_CH0_IRQHandler(void)
{
	if(PIT->CHANNEL[PIT_Channel_0].TFLG & 0x01)		/*  发生了中断  */
	{
		if(PIT_Handler[PIT_Channel_0])	/*  如果设置了中断回调函数  */
			PIT_Handler[PIT_Channel_0]();			/*  调用中断回调函数  */
	}
	PIT->CHANNEL[PIT_Channel_0].TFLG |= PIT_TFLG_TIF_MASK; /*  清除中断标志位  */
}


/*
*********************************************************************************************************
*                                     PIT_CH1_IRQHandler     
*
* Description: PIT定时器通道1中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void PIT_CH1_IRQHandler(void)
{
	if(PIT->CHANNEL[PIT_Channel_1].TFLG & 0x01)		/*  发生了中断  */
	{
		if(PIT_Handler[PIT_Channel_1])	/*  如果设置了中断回调函数  */
			PIT_Handler[PIT_Channel_1]();			/*  调用中断回调函数  */
	}
	PIT->CHANNEL[PIT_Channel_1].TFLG |= PIT_TFLG_TIF_MASK; /*  清除中断标志位  */
}


/********************************************  END OF FILE  *******************************************/

