/**
  *******************************************************************************************************
  * File Name: bsp_switch.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 本文件提供了主控上拔码开关的基本操作函数
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

# include "bsp_switch.h"

/*
*********************************************************************************************************
*                     bsp_switch_Config                     
*
* Description: 初始化主控上的拔码开关
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_switch_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = BIT1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT1_PIN, ENABLE);
	drv_gpio_WritePin(BIT1_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT2_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT2_PIN, ENABLE);
	drv_gpio_WritePin(BIT2_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT3_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT3_PIN, ENABLE);
	drv_gpio_WritePin(BIT3_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT4_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT4_PIN, ENABLE);
	drv_gpio_WritePin(BIT4_PIN, GPIO_PIN_SET);
}


/*
*********************************************************************************************************
*                         bsp_switch_GetValue                 
*
* Description: 获取拔码开关的状态
*             
* Arguments  : None.
*
* Reutrn     : 1> 拔码开关的状态
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_switch_GetValue(void)
{
	uint8_t value = 0;
	
//	value |= (drv_gpio_ReadPin(BIT1_PIN) == 1) ? (1 << 0 ) : (value);
//	value |= (drv_gpio_ReadPin(BIT2_PIN) == 1) ? (1 << 1) : (value);
//	value |= (drv_gpio_ReadPin(BIT3_PIN) == 1) ? (1 << 2) : (value);
//	value |= (drv_gpio_ReadPin(BIT4_PIN) == 1) ? (1 << 3) : (value);
	value |= (uint8_t)((drv_gpio_ReadPin(BIT1_PIN) << 0) | (drv_gpio_ReadPin(BIT2_PIN) << 1));
	value |= (uint8_t)((drv_gpio_ReadPin(BIT3_PIN) << 2) | (drv_gpio_ReadPin(BIT4_PIN) << 3));
	
	return value;
}

/********************************************  END OF FILE  *******************************************/

