/**
  *******************************************************************************************************
  * File Name: bsp_timer.c
  * Author: Vector
  * Version: V1.2.2
  * Date: 2018-2-11
  * Brief: ���ļ��Եδ�ʱ��������һ���̶ȵķ�װ,ͬʱ�����������ʱ��,ÿ����ʱ������һ���ص�����
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: �����ļ�
	*		
	*		2.Author: Vector
	*			Date:	2018-2-17
	*			Mod:	1.�޸���ʱ����־λ����	
	*						2.�޸���ʱ������ʧ������
	*						
	*		3.Author: Vector
	*			Date: 2018-2-27
	*			Mod: 1.�޸ĺ�����bsp_tim_CreateSoftTimerΪbsp_tim_CreateSoftTimer
	*					 2.�޸ĺ�����bsp_tim_DeleteTimerΪbsp_tim_DeleteSoftTimer
	*					 3.�Ż����ֺ���,��ֹ��������Խ��Ŀ���
	*          4.��������bsp_tim_CreateHardTimer����,����Ӳ����ʱ������,Ӳ����ʱ������PIT��ʱ��
	*          5.��������bsp_tim_DeleteHardTimer����,ɾ��Ӳ����ʱ��
	*          6.����HardTimer[]Ӳ����ʱ����������
	*
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_timer.h"

static volatile uint32_t s_uiDelayCount = 0;	/*  ���ļ�˽�б���,������ʱ  */
static volatile uint8_t s_ucTimeOutFlag = 0;	/*  ���ļ�˽�б���,����ͳ������ʱ��  */

__IO int32_t g_iRunTime = 0;		/*  ���ļ�˽�б���,����ʱ��  */


SoftTimer_TypeDef SoftTimer[SOFT_TIMER_COUNT];	/*  �����ʱ����  */
HardTimer_TypeDef HardTimer[2];									/*  ���ڲ���PIT��ʱ��,���Ӳ����ʱ�����ֻ��������  */
Time_TypeDef SysTime;
/*
*********************************************************************************************************
*                               bsp_tim_SoftDec           
*
* Description: �ݼ�ÿ�������ʱ���ļ�����
*             
* Arguments  : 1> pTimer:SoftTimer_TypeDef�ṹ��ָ�룬ָ��һ�������ʱ��
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ�˽�к���
*********************************************************************************************************
*/
void bsp_tim_SoftDec(SoftTimer_TypeDef * pTimer)
{
	if(pTimer->v_uiCount >0)		/*  ֻ���ڼ�������ֵ���������Ҫ�ݼ�  */
	{
		if(--pTimer->v_uiCount ==0 )  /*  ʱ�䵽���־��λ  */
		{
			pTimer->v_ucFlag = 1;
			
			
			if(pTimer->v_ucMode == TIMER_MODE_AUTO)			/*  ��ʱ��ģʽΪ�Զ���װ��ʱ��װ������  */
			{
				pTimer->v_uiCount = pTimer->v_uiPreLoad;
				pTimer->v_ucFlag = 0;
			}
			
			if(pTimer->_cbTimer)			/*  ���ûص�����ʱ���ûص�����  */
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
* Description: �δ�ʱ���жϷ�����
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
	
	if(s_uiDelayCount > 0)		/*  ��ʱ������  */
	{
		if(-- s_uiDelayCount == 0) 
			s_ucTimeOutFlag = 1;			/*  ��ʱ���  */
	}
	
	g_iRunTime ++;				/*  ����ʱ�������  */
	if(g_iRunTime == 0x7fffffff) g_iRunTime = 0;			/*  ����ʱ�������Ϊ32λ�����ֵΪ 0x7fffffff */
	
	if(g_iRunTime % 990 == 0)
		SysTime.Seconds++;

	if(SysTime.Seconds > 59) 
	{
		SysTime.Seconds = 0;
		SysTime.Minutes ++;
	}
	
	if(SysTime.Minutes > 59)
	{
		SysTime.Minutes = 0;
		SysTime.Hours++;
	}
	if(SysTime.Hours > 23)
	{
		SysTime.Hours = 0;
	}
# if OS_SUPPORT > 0u			/*  �����Ҫ֧�ֲ���ϵͳ  */
	if(OS_RUNNING == 1)
	{
		OSIntEnter();						//�����ж�
		OSTimeTick();       				//����ucos��ʱ�ӷ������               
		OSIntExit();       	 				//���������л����ж�
	}
# else
	for(i = 0; i < SOFT_TIMER_COUNT; i++)		/*  �ݼ������ʱ����ֵ  */
	{
		bsp_tim_SoftDec(&SoftTimer[i]);
	}
# endif
	
}

/*
*********************************************************************************************************
*                                SysTick_Handler          
*
* Description: �δ�ʱ���ж�
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
* Description: ��ʼ�������ʱ��
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
	SysTime.Hours = 0;
	SysTime.Minutes = 0;
	SysTime.Seconds = 0;
	
	SysTick_Config(SystemCoreClock / 1000);
}


/*
*********************************************************************************************************
*                             bsp_tim_CreateSoftTimer             
*
* Description: ����һ�������ʱ��
*             
* Arguments  : 1> ucTimerId:�����ʱ��ID
*              2> uiPeriod:��ʱ����
*              3> _cbTimer:��ʱ���ص�����
*              4> eMode:��ʱ��ģʽ
*
* Reutrn     : 1> 0: �ɹ�
*              2> ����: �������
*
* Note(s)    : 1.�����ʱ�����������ƣ�������ID�������ʱ������ʱ���ᴴ��ʧ��
*              2.�����ʱ�����ܸ����Ѿ�ʹ�õĶ�ʱ������Ҫ��ԭ����ɾ������ܴ���
*							 3.��ʱ���ص�����ִ��ʱ��Ӧ�����ܶ�,�Ҳ�������ʱ����
*********************************************************************************************************
*/
int8_t bsp_tim_CreateSoftTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer, TIMER_MODE_ENUM eMode)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;				/*  ������ʱ������  */
	if(SoftTimer[ucTimerId].ucUsed == 1) return -2;		/*  �Ѿ�ʹ���˵Ķ�ʱ�������ٴδ���  */
	
	SoftTimer[ucTimerId].v_uiCount = uiPeriod;		/*  ���ö�ʱ����  */
	SoftTimer[ucTimerId].v_ucFlag = 0;		/*  �����ʱ��־λ  */
	SoftTimer[ucTimerId].v_ucMode = eMode;		/*  ��ʱģʽ  */
	SoftTimer[ucTimerId].ucUsed = 1;	/*  �Ѿ�ʹ��  */
	SoftTimer[ucTimerId].v_uiPreLoad = uiPeriod;	/*  �������ֵ  */
	SoftTimer[ucTimerId]._cbTimer = _cbTimer;		/*  ��ʱ���ص�����  */
	
	return 0;
}

/*
*********************************************************************************************************
*                                   bsp_tim_DeleteSoftTimer       
*
* Description: ɾ��һ�������ʱ��
*             
* Arguments  : 1> ucTimerId:�����ʱ��ID
*
* Reutrn     : 1> 0: ɾ���ɹ�
*              2> ����: �������
*
* Note(s)    : δʹ�õĶ�ʱ�����ܱ�ɾ��
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
* Description: ����һ��Ӳ����ʱ��
*             
* Arguments  : 1> ucTimerId: Ӳ����ʱ��ID 
*              2> uiPeriod: ��ʱ����
*              3> _cbTimer: ��ʱ���ص�����
*
* Reutrn     : 1> 0: �ɹ�
*              2> ����: �������
*
* Note(s)    : 1.�����ʱ�����������ƣ�������ID�������ʱ������ʱ���ᴴ��ʧ��
*              2.�����ʱ�����ܸ����Ѿ�ʹ�õĶ�ʱ������Ҫ��ԭ����ɾ������ܴ���
*							 3.��ʱ���ص�����ִ��ʱ��Ӧ�����ܶ�,�Ҳ�������ʱ����
*********************************************************************************************************
*/
int8_t bsp_tim_CreateHardTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer)
{
	PIT_InitTypeDef PIT_InitStruct;
	
	if(ucTimerId > 2) return -1;		/*  ���ֻ����������ʱ��  */
	if(HardTimer[ucTimerId].ucUsed == 1)	return -2;	/*  �ö�ʱ���Ѿ���ʹ����  */
	
	PIT_InitStruct.PIT_Channel = ucTimerId;
	PIT_InitStruct.PIT_IRQCmd = ENABLE;
	PIT_InitStruct.PIT_IRQPriority = 0x03;
	PIT_InitStruct.PIT_Period = uiPeriod;
	PIT_InitStruct.PIT_Callback = _cbTimer;
	drv_pit_Init(&PIT_InitStruct);
	
	HardTimer[ucTimerId].ucUsed = 1;
	HardTimer[ucTimerId]._cbTimer = _cbTimer;
	
	NVIC_SetPriority(PIT_CH0_IRQn, 1);
	NVIC_SetPriority(PIT_CH1_IRQn, 3);
	return 0;
}


/*
*********************************************************************************************************
*                                 bsp_tim_DeleteHardTimer         
*
* Description: ɾ��һ��Ӳ����ʱ��
*             
* Arguments  : 1> ucTimerId: ��ʱ��ID
*
* Reutrn     : 1> 0: �ɹ�
*              2> -1: ��Ч��ʱ��ID
*              3> -2: �ö�ʱ��δʹ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
int8_t bsp_tim_DeleteHardTimer(uint8_t ucTimerId)
{
	if(ucTimerId > 2) return -1;		/*  ��Ч��ʱ��ID  */
	if(HardTimer[ucTimerId].ucUsed == 0) return -2;	/*  �ö�ʱ��δʹ��  */
	
	drv_pit_Stop(ucTimerId);
	
	/*  ���㶨ʱ��  */
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
* Description: ��������ʱ���Ƿ񵽴ﶨʱʱ��
*             
* Arguments  : 1> ucTimerId:��ʱ��ID
*
* Reutrn     : 1> 0: ��ʱʱ�䵽
*              2> 1: δ����ʱʱ��
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
* Description: ��ȡϵͳ����ʱ��
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
* Description: ���뼶��ʱ����
*             
* Arguments  : 1> ui_nMs:Ҫ��ʱ��ʱ��
*
* Reutrn     : None.
*
* Note(s)    :  ��ʱ���õδ�ʱ��ʵ�֣���Ϊ�δ�ʱ��Ϊ16λ��ʱ�������Ը������ʱʱ��Ϊ65534
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
* Description: ΢�뼶��ʱ����
*             
* Arguments  : 1> ui_nUs:��ʱʱ����΢��
*
* Reutrn     : None.
*
* Note(s)    : �����ʱΪ65535΢��
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
    ticks = ui_nUs * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

    while (1)
    {
			tnow = SysTick->VAL;    
			if (tnow != told)
			{    
				/* SYSTICK��һ���ݼ��ļ����� */    
				if (tnow < told)
				{
						tcnt += told - tnow;    
				}
				/* ����װ�صݼ� */
				else
				{
						tcnt += reload - tnow + told;    
				}        
				told = tnow;

				/* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
				if (tcnt >= ticks)
				{
					break;
				}
			}  
    }
}

/********************************************  END OF FILE  *******************************************/


