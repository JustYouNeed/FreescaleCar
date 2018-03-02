/**
  *******************************************************************************************************
  * File Name: drv_pit.h
  * Author: Vector
  * Version: V1.1.1
  * Date: 2018-2-1
  * Brief: ���ļ�����PIT��ص��������������,ͬʱ�����˲���PIT�ĺ������ṹ�塢ö�ٱ�����
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *		  Author: Vector
  *		  Mod: �����ļ�
	*		
	*		1.Date: 2018-2-27
	*			Author:	Vector
	*			Mod: 1.ΪPIT_InitTypeDef�ṹ������±���,PIT_IRQPriority
						 2.������ʱ��ͨ����������
  *
  *******************************************************************************************************
  */


# ifndef __DRV_PIT_H
# define __DRV_PIT_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

/*  ����ָ��  */
typedef void (*_cb_PIT_IRQHandler)(void);


/*  PIT��ʱ����ʼ���ṹ��  */
typedef struct 
{
	uint8_t PIT_Channel;		/*  ͨ��  */
	uint8_t PIT_IRQCmd;			/*  �ж�ʹ��  */
	uint32_t PIT_Period;		/*  ��ʱ����  */
	uint8_t PIT_IRQPriority;	/*  �ж����ȼ�  */
	_cb_PIT_IRQHandler PIT_Callback;		/*  �жϻص�����ָ��  */
}PIT_InitTypeDef;

/*  PIT��ʱ��ͨ��ö�ٱ���  */
typedef enum
{
	PIT_Channel_0 = 0,
	PIT_Channel_1 = 1,
}PITChannel_TypeDef;


/*  ���ⲿʹ�õ�PIT��������  */
void drv_pit_Init(PIT_InitTypeDef *PIT_InitStruct);
_cb_PIT_IRQHandler drv_pit_SetCallback(uint8_t PITx, _cb_PIT_IRQHandler PIT_Callback);
uint32_t drv_pit_GetCounter(uint8_t PITx);
void drv_pit_Stop(uint8_t PITx);

# endif

/********************************************  END OF FILE  *******************************************/
