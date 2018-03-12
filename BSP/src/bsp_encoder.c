/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-2-28
  * Brief: ���ļ��ṩ���йر������Ļ�����������,���ʼ������ȡ������ֵ��,�����������жϷ�ʽ����
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-2-28
	*     Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.��ԭ�������ı������������ϵ�Car�ṹ����,���ڹ���
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_encoder.h"
# include "bsp_timer.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                                 _cbLeftEncoder         
*
* Description: ��߱������ص�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void _cbLeftEncoder(void)
{
	Car.Motor.LeftEncoder ++;
}

/*
*********************************************************************************************************
*                                 _cbRightEncoder         
*
* Description: �ұ߱������ص�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void _cbRightEncoder(void)
{
	Car.Motor.RightEncoder ++;
}


/*
*********************************************************************************************************
*                            bsp_encoder_Config              
*
* Description: ��ʼ��������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	KBI_InitTypeDef KBI_InitStruct;
	
	/*  ��ʼ����������������  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = LEFTENCONDER_DIR_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  �ұ߱�������������  */
	GPIO_InitStruct.GPIO_Pin = RIGHTENCONDER_DIR_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  ��ʼ��������ͨ��  */
	KBI_InitStruct.KBI_Channel = LEFTENCONDER_CHANNEL | RIGHTENCONDER_CHANNEL;
	KBI_InitStruct.KBI_EdgeOnlyCmd = ENABLE;
	KBI_InitStruct.KBI_TrigMode = KBI_TrigFalling;
	drv_kbi_Init(&KBI_InitStruct);
	
	drv_kbi_SetCallback(LEFTENCONDER_CHANNEL, _cbLeftEncoder);	/*  ע��ص�����  */	

	drv_kbi_SetCallback(RIGHTENCONDER_CHANNEL, _cbRightEncoder);	/*  ע��ص�����  */
}

/*
*********************************************************************************************************
*                              bsp_encoder_SpeedCalc            
*
* Description: �ɱ�������ֵ�������ٶ�
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ӧ�ö�ʱ����,������ٶȼ��㾫��,ͬʱ�ú������������ֵֻ����Ϊ�ο�ֵ,���Ǻܾ�ȷ
*********************************************************************************************************
*/
extern uint8_t TimerTaskRunMutexSignal;
void bsp_encoder_SpeedCalc(void)
{
<<<<<<< HEAD
=======
	static uint32_t LastLeftEncoder,LastRightEncoder;
>>>>>>> origin/Mr-He
	static uint32_t LastTime;
	int32_t runtime;

	/*  ����������жϺ�������ִ��,��ֱ�ӷ���  */
	if(TimerTaskRunMutexSignal == 1) return ;
	
	/*  �궨��ʱ��������������  */
	TimerTaskRunMutexSignal = 1;
	
	/*  �����ٶ�  */
	runtime = bsp_tim_GetRunTime() - LastTime;
	Car.Motor.LeftSpeed = (int32_t)(Car.Motor.LeftEncoder - 0) / runtime;
	Car.Motor.RightSpeed = (int32_t)(Car.Motor.RightEncoder - 0) / runtime;
	
	/*  ��ձ�����ֵ  */
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  ����ʱ��,Ϊ�´μ�����׼��  */
	LastTime = 	bsp_tim_GetRunTime();
	
	/*  �����������  */
	TimerTaskRunMutexSignal = 0;
}

/********************************************  END OF FILE  *******************************************/


