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
//static void _cbLeftEncoder(void)
//{
//	Car.Motor.LeftEncoder ++;
//}

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
//static void _cbRightEncoder(void)
//{
//	Car.Motor.RightEncoder ++;
//}


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
//	KBI_InitTypeDef KBI_InitStruct;
	

//	
//	/*  ��ʼ��������ͨ��  */
//	KBI_InitStruct.KBI_Channel = LEFTENCONDER_CHANNEL | RIGHTENCONDER_CHANNEL;
//	KBI_InitStruct.KBI_EdgeOnlyCmd = ENABLE;
//	KBI_InitStruct.KBI_TrigMode = KBI_TrigFalling;
//	drv_kbi_Init(&KBI_InitStruct);
//	
//	drv_kbi_SetCallback(LEFTENCONDER_CHANNEL, _cbLeftEncoder);	/*  ע��ص�����  */	

//	drv_kbi_SetCallback(RIGHTENCONDER_CHANNEL, _cbRightEncoder);	/*  ע��ص�����  */
//	
//	NVIC_SetPriority(KBI0_IRQn, 0);


	SIM->SCGC |= SIM_SCGC_FTM0_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS(1);
//	FTM0->MODE |= (1 << 2);
	
	FTM0->SC &= ~(3);
	FTM0->SC |= (3 << 3);
	FTM0->CNT = 0;
//	FTM0->MODE &= ~(1 << 2);
	
	SIM->SCGC |= SIM_SCGC_FTM1_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM1CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM1CLKPS(2);
//	FTM1->MODE |= (1 << 2);
	FTM1->SC |= FTM_SC_PS(0);
	FTM1->SC |= FTM_SC_CLKS(3);  
//	FTM1->MODE &= ~(1 << 2);
	FTM1->CNT = 0;
	
	
//	drv_gpio_PullCmd(GPIO_Pin_E0, ENABLE);
//	drv_gpio_PullCmd(GPIO_Pin_E7, ENABLE);
	
		/*  ��ʼ����������������  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = LEFTENCONDER_DIR_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  �ұ߱�������������  */
	GPIO_InitStruct.GPIO_Pin = RIGHTENCONDER_DIR_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
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
	static uint32_t LastTime;
	static uint16_t LastLeftEnconder, LastRightEnconder;
	int32_t runtime;

	/*  ����������жϺ�������ִ��,��ֱ�ӷ���  */
//	if(TimerTaskRunMutexSignal == 1) return ;
//	
//	/*  �궨��ʱ��������������  */
//	TimerTaskRunMutexSignal = 1;
	
	Car.Motor.LeftEncoder = (uint16_t)FTM0->CNT;
	Car.Motor.RightEncoder = (uint16_t)FTM1->CNT;
	
	FTM0->CNT = 0;
	FTM1->CNT = 0;
	
//	if(Car.Motor.LeftEncoder - LastLeftEnconder > 100 || Car.Motor.LeftEncoder - LastLeftEnconder < -100)
//		Car.Motor.LeftEncoder = LastLeftEnconder;
//	
//	if(Car.Motor.RightEncoder - LastRightEnconder > 100 || Car.Motor.RightEncoder - LastRightEnconder < -100)
//		Car.Motor.RightEncoder = LastRightEnconder;
	
	/*  �����ٶ�  */
//	runtime = bsp_tim_GetRunTime() - LastTime;
	Car.Motor.LeftSpeed = (Car.Motor.LeftEncoder - 0);
	Car.Motor.RightSpeed = (Car.Motor.RightEncoder - 0);
	
		if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	
	LastLeftEnconder = Car.Motor.LeftEncoder;
	LastRightEnconder = Car.Motor.RightEncoder;
	
//	Car.Motor.LeftEncoder = 0;
//	Car.Motor.RightEncoder = 0;
	
	/*  ����ʱ��,������ֵ,Ϊ�´μ�����׼��  */
//	LastTime = 	bsp_tim_GetRunTime();
//	LastLeftEncoder = Car.Motor.LeftEncoder;
//	LastRightEncoder = Car.Motor.RightEncoder;
	
	
//	TimerTaskRunMutexSignal = 0;
}

/********************************************  END OF FILE  *******************************************/


