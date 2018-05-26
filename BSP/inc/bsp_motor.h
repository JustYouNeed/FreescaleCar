/**
  *******************************************************************************************************
  * File Name: bsp_motor.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 本文件声明了有关电机控制的函数以及变量
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_MOTOR_H
# define __BSP_MOTOR_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

/*  电机PWM通道宏定义  */
# define DRV_PWM1_CHANNEL		PWM_Channel0_H0
# define DRV_PWM2_CHANNEL		PWM_Channel1_H1
# define DRV_PWM3_CHANNEL		PWM_Channel3_G5
# define DRV_PWM4_CHANNEL		PWM_Channel4_G6

/*  电机驱动使能引脚  */
# define DRV_EN_PIN			GPIO_Pin_E5
# define DRV_ENABLE()		drv_gpio_WritePin(DRV_EN_PIN, GPIO_PIN_SET)
# define DRV_DISABLE()	drv_gpio_WritePin(DRV_EN_PIN, GPIO_PIN_RESET)

/*  电机控制结构体定义  */
typedef struct
{	
 	int16_t LeftPwm;				/*  左边电机PWM  */
	int16_t RightPwm;				/*  右边电机PWM  */
	
	int32_t LeftEncoder;		/*  左边电机编码器  */
	int32_t RightEncoder;		/*  右边电机编码器  */
	
	float LeftSpeed;			/*  左边电机转速  */
	float RightSpeed;			/*  右边电机转速  */
}Motor_TypeDef;


/*
  *******************************************************************************************************
  *                              FUNCTION DECLARE
  *******************************************************************************************************
*/
void bsp_motor_Config(void);
void bsp_motor_SetPwm(int16_t LeftPwm, int16_t RightPwm);
void bsp_motor_Stop(void);

# endif

/********************************************  END OF FILE  *******************************************/

