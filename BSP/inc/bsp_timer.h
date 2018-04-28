/**
  *******************************************************************************************************
  * File Name: bsp_timer.h
  * Author: Vector
  * Version: V1.1.1
  * Date: 2018-2-11
  * Brief: ���ļ��������й������ʱ����һЩ����
  *******************************************************************************************************
  * History
	*		1.Date: 2018-2-11
	* 		Author: Vector
  *			Mod: �����ļ�
	*			
	*		2.Date: 2018-2-27
	*			Author: Vector
	*			Mod: 1.�޸ĺ�����bsp_tim_CreateSoftTimerΪbsp_tim_CreateSoftTimer
	*					 2.�޸ĺ�����bsp_tim_DeleteTimerΪbsp_tim_DeleteSoftTimer
	*					 3.��������bsp_tim_CreateHardTimer����,����Ӳ����ʱ������,Ӳ����ʱ������PIT��ʱ��
	*				   4.��������bsp_tim_DeleteHardTimer����,ɾ��Ӳ����ʱ��
	*          4.�����ṹ��HardTimer_Str,Ӳ����ʱ������ṹ��
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_TIMER_H
# define __BSP_TIMER_H

# include "derivative.h"

# define SOFT_TIMER_COUNT			6

typedef void (*_cbTimerCallBack)(void);		/*  �����ʱ���ص�������������  */

/*  �����ʱ�����ƽṹ��  */
typedef struct	
{
	volatile uint8_t v_ucMode;		/*  ģʽ  */
	volatile uint8_t v_ucFlag;		/*  ��ʱ�����־  */
	volatile uint32_t v_uiCount;	/*  ��ʱ������  */
	volatile uint32_t v_uiPreLoad;	/*  ��װ��ֵ  */
	
	uint8_t ucUsed;					/*  �Ƿ��Ѿ�ʹ��  */
	_cbTimerCallBack _cbTimer;	/*  �ص�����  */
	
}SoftTimer_TypeDef;

typedef struct
{
	uint8_t ucUsed;							/*  �Ƿ��Ѿ�ʹ��  */
	_cbTimerCallBack _cbTimer;	/*  �ص�����  */
}HardTimer_TypeDef;


typedef struct
{
	uint16_t Hours, Minutes, Seconds;
	uint8_t Years, Months, Days;
}Time_TypeDef;

/*  �����ʱ��ģʽö�ٱ���  */
typedef enum
{
	TIMER_MODE_ONCE = 0x00,
	TIMER_MODE_AUTO
}TIMER_MODE_ENUM;

extern Time_TypeDef SysTime;

void bsp_tim_SoftConfig(void);  /* ��ʼ�������ʱ�� */
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

