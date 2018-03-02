/**
  *******************************************************************************************************
  * File Name: drv_pit.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-1
  * Brief: KEA128оƬPIT��ʱ���ײ���������
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *     Author: Vector
  *     Mod: �����ļ�,��ӻ�������
	*	
	*		2.Date: 2018-2-27
	*			Author: Vector
	*			Mod: �޸���ʱ������ʧ��BUG
  *
  *******************************************************************************************************
  */

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_pit.h"

/*  ��̬����,�������PIT��ʱ����ʱ�������ûص�����,
		���洢�ڸûص�����ָ����������,�����жϷ���
		��ʱ��ִ��.
*/
static _cb_PIT_IRQHandler PIT_Handler[2]; 

/*
*********************************************************************************************************
*                                     drv_pit_Init     
*
* Description: ��ʼ��һ��PIT��ʱ��
*             
* Arguments  : 1> PIT_InitStruct: PIT��ʱ����ʼ���ṹ��ָ��
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
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].LDVAL = cnt;	/*  ��������ж�ʱ��  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TFLG |= PIT_TFLG_TIF_MASK;					/*  ����жϱ�־λ  */
	
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL &= ~ PIT_TCTRL_TEN_MASK;       	/*  ��ֹPITx��ʱ��,������ռ�����  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL &= ~PIT_TCTRL_TIE_MASK;         /*  �ȹر��ж�  */
	PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL  |= PIT_TCTRL_TEN_MASK;	/*  ����PITx��ʱ��  */
	
	if(PIT_InitStruct->PIT_IRQCmd == ENABLE)		/*  ����������ж���������Ӧλ  */
	{
		PIT->CHANNEL[PIT_InitStruct->PIT_Channel].TCTRL  |= PIT_TCTRL_TIE_MASK;		/*  �����ж�  */
		PIT_Handler[PIT_InitStruct->PIT_Channel] = PIT_InitStruct->PIT_Callback;	/*  �����жϺ���  */
		
		/*  ��������ж�  */
		switch(PIT_InitStruct->PIT_Channel)
		{
			case PIT_Channel_0: 
			{
				NVIC_EnableIRQ(PIT_CH0_IRQn);		/*  �����ж�  */
				NVIC_SetPriority(PIT_CH0_IRQn, PIT_InitStruct->PIT_IRQPriority);	/*  �����ж����ȼ�  */
			}break;
			case PIT_Channel_1: 
			{
				NVIC_EnableIRQ(PIT_CH1_IRQn); /*  �����ж�  */
				NVIC_SetPriority(PIT_CH1_IRQn, PIT_InitStruct->PIT_IRQPriority); /*  �����ж����ȼ�  */
			}break;
		}
	}
}

/*
*********************************************************************************************************
*                                   drv_pit_SetCallback       
*
* Description: ����PIT��ʱ�����жϻص�����
*             
* Arguments  : 1> PITx: ��ʱ�����,��оƬֻ������ͨ��,��˸�ֵֻ��Ϊ0����1
*              2> PIT_Callback: �ص�����
*
* Reutrn     : None.
*
* Note(s)    : PITxֻ��ȡ0����1
*********************************************************************************************************
*/
_cb_PIT_IRQHandler drv_pit_SetCallback(uint8_t PITx, _cb_PIT_IRQHandler PIT_Callback)
{
	_cb_PIT_IRQHandler _cb_Last = PIT_Handler[PITx];	/*  ���ڷ���֮ǰ�Ļص�����  */
	
	if(PITx > 1) return 0;		/*  ��ʱ��ͨ��������  */
		
	PIT_Handler[PITx] = PIT_Callback;		/*  ����ûص�����  */
	
	return _cb_Last;
}

/*
*********************************************************************************************************
*                                    drv_pit_GetCounter      
*
* Description: ��ȡ��ʱ����������ֵ
*             
* Arguments  : 1> PITx: ��ʱ�����,��оƬֻ������ͨ��,��˸�ֵֻ��Ϊ0����1
*
* Reutrn     : ��������ֵ
*
* Note(s)    : PITxֻ��ȡ0����1
*********************************************************************************************************
*/
uint32_t drv_pit_GetCounter(uint8_t PITx)
{
	uint32_t val;
	val = (uint32_t)(~0) - PIT->CHANNEL[PITx].CVAL;
	if(PIT->CHANNEL[PITx].TFLG &  PIT_TFLG_TIF_MASK)	/*  �ж�ʱ���Ƿ�ʱ  */
	{
		PIT->CHANNEL[PITx].TFLG |= PIT_TFLG_TIF_MASK;		/*  ����жϱ�־λ  */
		PIT->CHANNEL[PITx].TCTRL &= ~ PIT_TCTRL_TEN_MASK;		/*  ��ֹ��ʱ��  */
		return 0XFFFFFFFF;									
	}
	return val;
}

/*
*********************************************************************************************************
*                                      drv_pit_Stop    
*
* Description: ֹͣһ����ʱ��
*             
* Arguments  : 1> PITx: ��ʱ�����,��оƬֻ������ͨ��,��˸�ֵֻ��Ϊ0����1
*
* Reutrn     : None.
*
* Note(s)    : PITxֻ��ȡ0����1
*********************************************************************************************************
*/
void drv_pit_Stop(uint8_t PITx)
{
	if(PITx > 1) return ;		/*  ��ʱ��ͨ��������  */
	
	PIT->CHANNEL[PITx].TFLG |= PIT_TFLG_TIF_MASK;		/*  ����жϱ�־λ  */
	PIT->CHANNEL[PITx].TCTRL &= ~ PIT_TCTRL_TEN_MASK;		/*  ��ֹ��ʱ��  */
}


/*
*********************************************************************************************************
*                                     PIT_CH0_IRQHandler     
*
* Description: PIT��ʱ��ͨ��0�жϺ���
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
	if(PIT->CHANNEL[PIT_Channel_0].TFLG & 0x01)		/*  �������ж�  */
	{
		if(PIT_Handler[PIT_Channel_0])	/*  ����������жϻص�����  */
			PIT_Handler[PIT_Channel_0]();			/*  �����жϻص�����  */
	}
	PIT->CHANNEL[PIT_Channel_0].TFLG |= PIT_TFLG_TIF_MASK; /*  ����жϱ�־λ  */
}


/*
*********************************************************************************************************
*                                     PIT_CH1_IRQHandler     
*
* Description: PIT��ʱ��ͨ��1�жϺ���
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
	if(PIT->CHANNEL[PIT_Channel_1].TFLG & 0x01)		/*  �������ж�  */
	{
		if(PIT_Handler[PIT_Channel_1])	/*  ����������жϻص�����  */
			PIT_Handler[PIT_Channel_1]();			/*  �����жϻص�����  */
	}
	PIT->CHANNEL[PIT_Channel_1].TFLG |= PIT_TFLG_TIF_MASK; /*  ����жϱ�־λ  */
}


/********************************************  END OF FILE  *******************************************/

