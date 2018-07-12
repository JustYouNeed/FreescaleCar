/**
  *******************************************************************************************************
  * File Name: drv_kbi.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-2-28
  * Brief: ���ļ��������йز���KBI����ĵײ���������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*     Date: 2018-2-28
	*     Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-1
	*			Mod: ����KBI��ʼ���߼�,��Ϊ���ýṹ���ʼ��
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv.h"

/*  �ж��ж��Ƿ���  */
# define IS_KBI0_IRQ(channel)				(KBI0->SP & channel)
# define IS_KBI1_IRQ(channel)				(KBI1->SP & channel)

/*  KBI��������  */
KBI_ManageDef KBI_INFO[58];

/*  KBIʹ�ü�����  */
static uint8_t KBI_UsedCount = 0;

/*  KBI����  */
static KBI_Type * const KBIX[] = KBI_BASES;

/*  KBI��ʼ����־  */
static uint8_t KBI0_IRQ_Init = 0;
static uint8_t KBI1_IRQ_Init = 0;
static uint8_t KBI0_Clock_Init = 0;
static uint8_t KBI1_Clock_Init = 0;
/*
*********************************************************************************************************
*                               drv_kbi_Init           
*
* Description: ��ʼ��KBI
*             
* Arguments  : 1> KBI_InitStruct: KBI��ʼ���ṹ��ָ��
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
	
	
	/*  ��ʼ����Ӧʱ��,ͬʱ��Ǹ�ʱ���Ѿ���ʼ��,��ֹ�´��ٴγ�ʼ��  */
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
	
	KBIX[kbix]->SC &= ~(uint32_t)KBI_SC_KBIE_MASK;		/*  ����ǰ�ȹر��ж�  */
	
	/*  ���ô���ģʽ  */
	if(KBI_InitStruct->KBI_TrigMode == KBI_TrigFalling || KBI_InitStruct->KBI_TrigMode == KBI_TringFallingLow)
		KBIX[kbix]->ES &= ~(uint32_t)channel;
	else
		KBIX[kbix]->ES |= (uint32_t)channel;
	
	/*  ��������  */
	for(; i < 32; i++)
	{
		if((channel >> i) & 0x1)
		{
			if(KBI_InitStruct->KBI_PuPdCmd != DISABLE)		/*  �����˿�����������  */
				drv_gpio_PullCmd((GPIOPin_TypeDef)((uint8_t)(i + kbix * 32)), ENABLE);
			
			/*  ��д�����  */
			if(KBI_INFO[KBI_UsedCount].KBI_Used == 0)
			{
				KBI_INFO[KBI_UsedCount].KBI_Used = 1;
				KBI_INFO[KBI_UsedCount].KBI_Channel = (uint32_t)(1 << i);
				KBI_UsedCount++;
			}
		}
	}
	
	/*  ʹ��KBI����  */
	KBIX[kbix]->PE |= (uint32_t)channel;
	
	/*  �����־λ  */
	if(kbix == 0) KBI0->SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
	else if(kbix == 1) KBI1->SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
	
	/*  ���ô���ģʽ  */
	if(KBI_InitStruct->KBI_TrigMode == KBI_TrigFalling || KBI_InitStruct->KBI_TrigMode == KBI_TrigRising)
		KBIX[kbix]->SC &= ~(1 << 0);
	else 
		KBIX[kbix]->SC |= KBI_SC_KBMOD_MASK;
	
	
	KBIX[kbix]->SC |= ( 0
										| KBI_SC_KBIE_MASK    //KBI�ж�ʹ��
										| KBI_SC_RSTKBSP_MASK
										| KBI_SC_KBSPEN_MASK  //ʹ��KBI_SP�Ĵ���
										);
	/*  ʹ��KBI�ж�  */
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
* Description: ����KBIͨ���Ļص�����
*             
* Arguments  : 1> KBI_Channel: KBIͨ��,��Χ��drv_kbi.h�ж���
*              2> _cb: �ص�����
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_kbi_SetCallback(uint32_t KBI_Channel, _cbKBICallBack _cb)
{
	uint8_t cnt = 0;
	
	for(; cnt < KBI_UsedCount; cnt ++)		/*  ���ҹ����  */
	{
		if(KBI_Channel == KBI_INFO[cnt].KBI_Channel) break;		/*  �ҵ���ǰͨ��  */
	}
	
	if(cnt == KBI_UsedCount) return;		/*  ��ͨ��û��ʹ��,�������ûص�����  */
	
	KBI_INFO[cnt]._cb = _cb;		/*  ����ص�����  */
}

/*
*********************************************************************************************************
*                           KBI0_IRQHandler               
*
* Description: KBI0�жϺ���
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
		for(; cnt < KBI_UsedCount; cnt ++)		/*  ���ҹ����  */
		{
			if(IS_KBI0_IRQ(KBI_INFO[cnt].KBI_Channel) && KBI_INFO[cnt]._cb)  /*  �жϷ���,�������ûص�����  */
			{
				KBI_INFO[cnt]._cb();		/*  ִ�лص�����  */
			}
		}
	}
	KBI0_SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
}


/*
*********************************************************************************************************
*                           KBI1_IRQHandler               
*
* Description: KBI1�жϺ���
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
		for(; cnt < KBI_UsedCount; cnt ++)		/*  ���ҹ����  */
		{
			if(IS_KBI1_IRQ(KBI_INFO[cnt].KBI_Channel) && KBI_INFO[cnt]._cb)  /*  �жϷ���,�������ûص�����  */
			{
				KBI_INFO[cnt]._cb();		/*  ִ�лص�����  */
			}
		}
	}
	KBI1_SC |= KBI_SC_KBACK_MASK | KBI_SC_RSTKBSP_MASK;
}



/********************************************  END OF FILE  *******************************************/

