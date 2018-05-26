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
# include "bsp.h"


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
	
	GPIO_InitStruct.GPIO_Pin = LED_RED_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	drv_gpio_Init(&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = LED_BLUE_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	
	bsp_led_OFF(LED_ALL);
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void bsp_led_WritePin(LED_EnumTypeDef LEDx, uint8_t LEDState)
{
	switch(LEDx)
	{
		case LED_ALL:	drv_gpio_WritePin(LED_RED_PIN, LEDState); drv_gpio_WritePin(LED_BLUE_PIN, LEDState);break;
		case LED_RED: drv_gpio_WritePin(LED_RED_PIN, LEDState);break;
		case LED_BLUE: drv_gpio_WritePin(LED_BLUE_PIN, LEDState);break;
		default:break;
	}
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void bsp_led_TogglePin(LED_EnumTypeDef LEDx)
{
	switch(LEDx)
	{
		case LED_ALL:	drv_gpio_TogglePin(LED_RED_PIN); drv_gpio_TogglePin(LED_BLUE_PIN);break;
		case LED_RED: drv_gpio_TogglePin(LED_RED_PIN);break;
		case LED_BLUE: drv_gpio_TogglePin(LED_BLUE_PIN);break;
		default:break;
	}
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
		case LED_ALL: bsp_led_WritePin(LED_ALL, LED_ON);break;
		case LED_RED: bsp_led_WritePin(LED_RED, LED_ON);break;
		case LED_BLUE: bsp_led_WritePin(LED_BLUE, LED_ON);break;
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
		case LED_ALL: bsp_led_WritePin(LED_ALL, LED_OFF);break;
		case LED_RED: bsp_led_WritePin(LED_RED, LED_OFF);break;
		case LED_BLUE: bsp_led_WritePin(LED_BLUE, LED_OFF);break;
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
		case LED_ALL: bsp_led_TogglePin(LED_ALL);break;
		case LED_RED: bsp_led_TogglePin(LED_RED);break;
		case LED_BLUE: bsp_led_TogglePin(LED_BLUE);break;
		default: break;
	}
}


/********************************************  END OF FILE  *******************************************/

