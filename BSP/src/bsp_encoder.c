/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V2.1.0
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
	*		3.Author: Vector
	*			Date: 2018-3-16
	*			Mod: 1.��������������ʽ��KBI�жϸ�ΪFTM������
	*					 2.ɾ������������жϻص�����
	*				   3.�޸ĵ���ٶȼ��㷽ʽ,�ɼ���������ֵ���Ϊ��ȡ�����������㷽ʽ
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

	/*  ��ʼ����߱�����,����FTM0,����E0  */
	SIM->SCGC |= SIM_SCGC_FTM0_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS(1);
	FTM0->SC &= ~(3);
	FTM0->SC |= (3 << 3);
	FTM0->CNT = 0;
	
	/*  ��ʼ����߱�����,����FTM1,����E7  */
	SIM->SCGC |= SIM_SCGC_FTM1_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM1CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM1CLKPS(2);
	FTM1->SC |= FTM_SC_PS(0);
	FTM1->SC |= FTM_SC_CLKS(3);  
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
<<<<<<< HEAD
void bsp_encoder_SpeedCalc(void)
<<<<<<< HEAD
{
//<<<<<<< HEAD
//=======
	static uint32_t LastLeftEncoder,LastRightEncoder;
//<<<<<<< HEAD
//>>>>>>> origin/Mr-He
//=======
//>>>>>>> Mr-He
//>>>>>>> d476e22040494988d81fb0d65879a545a2623703
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
	
//<<<<<<< HEAD
	/*  ��ձ�����ֵ  */
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  ����ʱ��,Ϊ�´μ�����׼��  */
//=======
	
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  ����ʱ��,������ֵ,Ϊ�´μ�����׼��  */
//>>>>>>> Mr-He
	LastTime = 	bsp_tim_GetRunTime();
	
	/*  �����������  */
	TimerTaskRunMutexSignal = 0;
=======
=======
void bsp_encoder_ReadCounter(void)
>>>>>>> Mr-He
{	
	/*  ��ȡFTM������ֵ  */
	Car.Motor.LeftEncoder += (uint16_t)FTM0->CNT;
	Car.Motor.RightEncoder += (uint16_t)FTM1->CNT;
	
	/*  ��ռ�����  */
	FTM0->CNT = 0;
	FTM1->CNT = 0;
	
<<<<<<< HEAD
//	if(Car.Motor.LeftEncoder - LastLeftEnconder > 100 || Car.Motor.LeftEncoder - LastLeftEnconder < -100)
//		Car.Motor.LeftEncoder = LastLeftEnconder;
//	
//	if(Car.Motor.RightEncoder - LastRightEnconder > 100 || Car.Motor.RightEncoder - LastRightEnconder < -100)
//		Car.Motor.RightEncoder = LastRightEnconder;
	
	/*  �����ٶ�  */
	Car.Motor.LeftSpeed = (Car.Motor.LeftEncoder - 0);
	Car.Motor.RightSpeed = (Car.Motor.RightEncoder - 0);
	
	/*  �ɷ����ƽ���жϵ���ٶȷ���,�������ߵ��������ת��180��,���Է�����180����λ��  */
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
>>>>>>> Mr-He
=======

//	/*  �ɷ����ƽ���жϵ���ٶȷ���,�������ߵ��������ת��180��,���Է�����180����λ��  */
//	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
//	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
>>>>>>> Mr-He
}

/********************************************  END OF FILE  *******************************************/


