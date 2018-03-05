/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: ���ļ��ṩ�˳��ӵ�����ƵĻ�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_motor.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: ��ʼ�������������.PWM
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
	
	/*  ��ʼ���������ʹ������  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = DRV_EN_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  �ȹر�����  */
	DRV_DISABLE();
	
	/*  ��ʼ�����PWM  */
	PWM_InitStruct.PWM_Channel = DRV_PWM1_CHANNEL;
	PWM_InitStruct.PWM_Frequency = 10;
	PWM_InitStruct.PWM_Pulse = 0;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM2_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM3_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
	
	PWM_InitStruct.PWM_Channel = DRV_PWM4_CHANNEL;
	drv_ftm_PWMInit(&PWM_InitStruct);
}
extern uint16_t period[3];
void ftm_pwm_duty(uint8_t ftmn, uint8_t ch, uint32_t duty)
{
    uint32_t cv;
    //ռ�ձ� = (CnV-CNTIN)/(MOD-CNTIN+1)
    switch(ftmn)
    {
    case 0:
        cv = (duty * (period[ftmn] - 0 + 1)) / 1000;
        break;

    case 1:
        cv = (duty * (period[ftmn] - 0 + 1)) / 1000;
        break;

    case 2:
        cv = (duty * (period[ftmn] - 0 + 1)) / 1000;
        break;

    default:
        break;
    }
    
    FTM2->CONTROLS[ch].CnV = cv;      //����ռ�ձ�
}
/*
*********************************************************************************************************
*                          bsp_motor_SetPwm                
*
* Description: ���õ��PWM
*             
* Arguments  : 1> LeftPwm: ��ߵ��PWM
*							 2> RightPwm: �ұߵ��PWM
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_SetPwm(int16_t LeftPwm, int16_t RightPwm)
{
	DRV_ENABLE();		/*  ��������  */
	
	/*  �������PWM  */
	if(RightPwm > 0)	/*  �����ת  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, RightPwm+3);
//		ftm_pwm_duty(2, 1, 0 + 3);
		drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, 3);
	}
	else		/*  ��ת  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, 0+3);
//		ftm_pwm_duty(2, 1, -(RightPwm + 3));
		drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, -(RightPwm-3));
	}
	
	/*  �������PWM  */
	if(LeftPwm > 0)	/*  �����ת  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, LeftPwm+3);
//		
		drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, 0+3);
	}
	else		/*  ��ת  */
	{
		drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, 0+3);
		drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, -(LeftPwm+3));
	}
}

/*
*********************************************************************************************************
*                          bsp_motor_Stop                
*
* Description: ֹͣ���
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
	DRV_DISABLE();	/*  �ر�����  */
	
	/*  PWM����ռ�ձ�Ϊ0  */
	drv_ftm_PWMSetDuty(DRV_PWM1_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM2_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM3_CHANNEL, 0);
	drv_ftm_PWMSetDuty(DRV_PWM4_CHANNEL, 0);
}
	
	

/********************************************  END OF FILE  *******************************************/

