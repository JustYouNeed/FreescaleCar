/**
  *******************************************************************************************************
  * File Name: drv_kbi.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-2-28
  * Brief: 本文件定义了有关操作KBI外设的底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*     Date: 2018-2-28
	*     Mod: 建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 更改KBI初始化逻辑,改为采用结构体初始化
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_kbi.h"

/*  判断中断是否发生  */
# define IS_KBI0_IRQ(channel)				(KBI0->SP & channel)
# define IS_KBI1_IRQ(channel)				(KBI1->SP & channel)

/*  KBI管理数组  */
KBI_ManageDef KBI_INFO[58];

/*  KBI使用计数器  */
static uint8_t KBI_UsedCount = 0;

/*  KBI数组  */
static KBI_Type * const KBIX[] = KBI_BASES;

/*  KBI初始化标志  */
static uint8_t KBI0_IRQ_Init = 0;
static uint8_t KBI1_IRQ_Init = 0;
static uint8_t KBI0_Clock_Init = 0;
static uint8_t KBI1_Clock_Init = 0;
/*
*********************************************************************************************************
*                               drv_kbi_Init           
*
* Description: 初始化KBI
*             
* Arguments  : 1> KBI_InitStruct: KBI初始化结构体指针
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_kbi_Init(KBI_InitTypeDef *KBI_InitStruct)
{
	uint8_t kbix = 0;
	uint32_t channel = 0, i = 0; 
	
	kbix = (KBI_InitStruct->KBI_Channel & 0xf0000000) ? 1 : 0;
	channel = KBI_InitStruct->KBI_Channel & 0x0fffffff;
	
	
	/*  初始化相应时钟,同时标记该时钟已经初始化,防止下次再次初始化  */
	if(kbix == 0 && KBI0_Clock_Init == 0)
	{
		SIM->SCGC |= SIM_SCGC_KBI0_MASK;
		KBI0_Clock_Init = 1;
	}
	if(kbix == 1 && KBI1_Clock_Init == 0)
	{
		SIM->SCGC |= SIM_SCGC_KBI1_MASK;
		KBI1_Clock_Init = 1;
	}
	
	KBIX[kbix]->SC &= ~(uint32_t)KBI_SC_KBIE_MASK;		/*  设置前先关闭中断  */
	
	/*  设置触发模式  */
	if(KBI_InitStruct->KBI_TrigMode == KBI_TrigFalling || KBI_InitStruct->KBI_TrigMode == KBI_TringFallingLow)
		KBIX[kbix]->ES &= ~(uint32_t)channel;
	else
		KBIX[kbix]->ES |= (uint32_t)channel;
	
	/*  开启上拉  */
	for(; i < 32; i++)
	{
		if((channel >> i) & 0x1)
		{
			if(KBI_InitStruct->KBI_PuPdCmd != DISABLE)		/*  设置了开启上拉电阻  */
				drv_gpio_PullCmd((GPIOPin_TypeDef)((uint8_t)(i + kbix * 32)), ENABLE);
			
			/*  填写管理表  */
			if(KBI_INFO[KBI_UsedCount].KBI_Used == 0)
			{
				KBI_INFO[KBI_UsedCount].KBI_Used = 1;
				KBI_INFO[KBI_UsedCount].KBI_Channel = (uint32_t)(1 << i);
				KBI_UsedCount++;
			}
		}
	}
	
	/*  使能KBI引脚  */
	KBIX[kbix]->PE |= (uint32_t)channel;
	
	/*  清除标志位  */
	if(kbix == 0) KBI0->SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
	else if(kbix == 1) KBI1->SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
	
	/*  设置触发模式  */
	if(KBI_InitStruct->KBI_TrigMode == KBI_TrigFalling || KBI_InitStruct->KBI_TrigMode == KBI_TrigRising)
		KBIX[kbix]->SC &= ~(1 << 0);
	else 
		KBIX[kbix]->SC |= KBI_SC_KBMOD_MASK;
	
	
	KBIX[kbix]->SC |= ( 0
										| KBI_SC_KBIE_MASK    //KBI中断使能
										| KBI_SC_RSTKBSP_MASK
										| KBI_SC_KBSPEN_MASK  //使能KBI_SP寄存器
										);
	/*  使能KBI中断  */
	if(kbix == 0 && KBI0_IRQ_Init == 0) 
	{
		NVIC_EnableIRQ(KBI0_IRQn);
		NVIC_SetPriority(KBI0_IRQn, 1);
		KBI0_IRQ_Init = 1;
	}
	else if(kbix == 1 && KBI1_IRQ_Init == 0) 
	{
		NVIC_EnableIRQ(KBI1_IRQn);
		NVIC_SetPriority(KBI1_IRQn, 2);
		KBI1_IRQ_Init = 1;
	}
	

}

/*
*********************************************************************************************************
*                              drv_kbi_SetCallback            
*
* Description: 设置KBI通道的回调函数
*             
* Arguments  : 1> KBI_Channel: KBI通道,范围在drv_kbi.h中定义
*              2> _cb: 回调函数
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_kbi_SetCallback(uint32_t KBI_Channel, _cbKBICallBack _cb)
{
	uint8_t cnt = 0;
	
	for(; cnt < KBI_UsedCount; cnt ++)		/*  查找管理表  */
	{
		if(KBI_Channel == KBI_INFO[cnt].KBI_Channel) break;		/*  找到当前通道  */
	}
	
	if(cnt == KBI_UsedCount) return;		/*  该通道没有使用,不能设置回调函数  */
	
	KBI_INFO[cnt]._cb = _cb;		/*  保存回调函数  */
}

/*
*********************************************************************************************************
*                           KBI0_IRQHandler               
*
* Description: KBI0中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void KBI0_IRQHandler(void)
{
	uint8_t cnt = 0;
	
	if(KBI0_SC & KBI_SC_KBF_MASK)
	{
		for(; cnt < KBI_UsedCount; cnt ++)		/*  查找管理表  */
		{
			if(IS_KBI0_IRQ(KBI_INFO[cnt].KBI_Channel) && KBI_INFO[cnt]._cb)  /*  中断发生,且有设置回调函数  */
			{
				KBI_INFO[cnt]._cb();		/*  执行回调函数  */
			}
		}
	}
	KBI0_SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
}


/*
*********************************************************************************************************
*                           KBI1_IRQHandler               
*
* Description: KBI1中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void KBI1_IRQHandler(void)
{
	uint8_t cnt = 0;
	
	if(KBI1_SC & KBI_SC_KBF_MASK)
	{
		for(; cnt < KBI_UsedCount; cnt ++)		/*  查找管理表  */
		{
			if(IS_KBI1_IRQ(KBI_INFO[cnt].KBI_Channel) && KBI_INFO[cnt]._cb)  /*  中断发生,且有设置回调函数  */
			{
				KBI_INFO[cnt]._cb();		/*  执行回调函数  */
			}
		}
	}
	KBI1_SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
}



/********************************************  END OF FILE  *******************************************/

