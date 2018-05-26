/**
  *******************************************************************************************************
  * File Name: bsp_motor.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��������йص�����Ƶĺ����Լ�����
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
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

/*  ���PWMͨ���궨��  */
# define DRV_PWM1_CHANNEL		PWM_Channel0_H0
# define DRV_PWM2_CHANNEL		PWM_Channel1_H1
# define DRV_PWM3_CHANNEL		PWM_Channel3_G5
# define DRV_PWM4_CHANNEL		PWM_Channel4_G6

/*  �������ʹ������  */
# define DRV_EN_PIN			GPIO_Pin_E5
# define DRV_ENABLE()		drv_gpio_WritePin(DRV_EN_PIN, GPIO_PIN_SET)
# define DRV_DISABLE()	drv_gpio_WritePin(DRV_EN_PIN, GPIO_PIN_RESET)

/*  ������ƽṹ�嶨��  */
typedef struct
{	
 	int16_t LeftPwm;				/*  ��ߵ��PWM  */
	int16_t RightPwm;				/*  �ұߵ��PWM  */
	
	int32_t LeftEncoder;		/*  ��ߵ��������  */
	int32_t RightEncoder;		/*  �ұߵ��������  */
	
	float LeftSpeed;			/*  ��ߵ��ת��  */
	float RightSpeed;			/*  �ұߵ��ת��  */
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

