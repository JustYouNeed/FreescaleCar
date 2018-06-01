/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件提供了车子电机控制的基本函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: 初始化电机控制引脚.PWM
*             
* Arguments  : None
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;	
	PWM_InitTypeDef PWM_InitStruct;
	
	/*  初始化电机驱动使能引脚  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = DRV_EN_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  先关闭驱动  */
	DRV_DISABLE();
	
	/*  初始化电机PWM  */
	PWM_InitStruct.PWM_Channel = DRV_PWM1_CHANNEL;
	PWM_InitStruct.PWM_Frequency = 13;
	PWM_InitStruct.PWM_Pulse = 0;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM2_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM3_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM4_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
}

/*
*********************************************************************************************************
*                          bsp_motor_SetPwm                
*
* Description: 设置电机PWM
*             
* Arguments  : 1> LeftPwm: 左边电机PWM
*							 2> RightPwm: 右边电机PWM
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_SetPwm(int16_t LeftPwm, int16_t RightPwm)
{
	DRV_DISABLE();		/*  关闭驱动  */	
	/*  设置左边PWM  */
	if(LeftPwm >= 0)	/*  电机正转  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, LeftPwm);
		drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, 0);
	}
	else		/*  反转  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, 0);
		drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, -LeftPwm);
	}
	
	/*  设置右边PWM  */
	if(RightPwm >= 0)	/*  电机正转  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, RightPwm);
		drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, 0);
	}
	else		/*  反转  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, 0);
		drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, -RightPwm);
	}
	
	DRV_ENABLE();
}

/*
*********************************************************************************************************
*                          bsp_motor_Stop                
*
* Description: 停止电机
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_Stop(void)
{
	DRV_DISABLE();	/*  关闭驱动  */
	
	/*  PWM设置占空比为0  */
	drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, 0);
}
	
	

/********************************************  END OF FILE  *******************************************/

