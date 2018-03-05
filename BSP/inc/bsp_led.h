/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-25
  * Brief: ���ļ��������йذ弶LED�Ƶı�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-2-25
	*			Mod:	�����ļ� 
	*
	*		2.Author:
	*			Date: 2018-3-1
	*			Mod: LED��ID�Ķ����ɺ궨���Ϊö�ٱ���
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_LED_H
# define __BSP_LED_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

/*  ���尴������ʱ���ŵ�״̬  */
# define LED_OFF				GPIO_PIN_RESET
# define LED_ON					GPIO_PIN_SET

/*  LED�����Ŷ���  */
# define LED1_PIN			GPIO_Pin_D0
# define LED2_PIN			GPIO_Pin_D1

/*  LED IDö�ٱ���  */
typedef enum
{
	LED_ALL = 0x0,
	LED1,
	LED2,
}LED_IDType;


void bsp_led_Config(void);
void bsp_led_ON(uint8_t LEDx);
void bsp_led_OFF(uint8_t LEDx);
void bsp_led_Toggle(uint8_t LEDx);

# endif

/********************************************  END OF FILE  *******************************************/

