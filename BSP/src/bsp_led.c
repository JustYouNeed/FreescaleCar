/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: ���ļ�Ϊ�弶LED������
  *******************************************************************************************************
  * History
	*		1.Author:	Vector
	*			Date:	2018-2-17
  *			Mod:	�����ļ�
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_led.h"


/*
*********************************************************************************************************
*                                   bsp_led_Config       
*
* Description: ��ʼ��LED����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = LED1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	drv_gpio_Init(&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = LED2_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	
	bsp_led_OFF(LED_ALL);
}


/*
*********************************************************************************************************
*                                          bsp_led_ON
*
* Description: ��һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�򿪵�LED�Ʊ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_ON(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			drv_gpio_WritePin(LED1_PIN, LED_ON);
			drv_gpio_WritePin(LED2_PIN, LED_ON);
		}break;
		case LED1: drv_gpio_WritePin(LED1_PIN, LED_ON);break;
		case LED2: drv_gpio_WritePin(LED2_PIN, LED_ON);break;
		default: break;
	}
}

/*
*********************************************************************************************************
*                                          bsp_led_OFF
*
* Description: �ر�һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�رյ�LED�Ʊ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_OFF(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			drv_gpio_WritePin(LED1_PIN, LED_OFF);
			drv_gpio_WritePin(LED2_PIN, LED_OFF);
		}break;
		case LED1: drv_gpio_WritePin(LED1_PIN, LED_OFF);break;
		case LED2: drv_gpio_WritePin(LED2_PIN, LED_OFF);break;
		default: break;
	}
}


/*
*********************************************************************************************************
*                                       bsp_led_Toggle   
*
* Description: �л�LED�Ƶ�״̬
*             
* Arguments  : 1> LEDx:	Ҫ�л���LED�Ʊ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Toggle(uint8_t LEDx)
{
	switch(LEDx)
	{
		case LED_ALL: 
		{
			drv_gpio_TogglePin(LED1_PIN);
			drv_gpio_TogglePin(LED2_PIN);
		}break;
		case LED1: drv_gpio_TogglePin(LED1_PIN);break;
		case LED2: drv_gpio_TogglePin(LED2_PIN);break;
		default: break;
	}
}


/********************************************  END OF FILE  *******************************************/

