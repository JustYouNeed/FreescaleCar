/**
  *******************************************************************************************************
  * File Name: bsp_battery.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-25
  * Brief: 本文件提供了小车主板电池电量测量功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-25
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

# include "bsp_battery.h"

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
void bsp_bat_Config(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Channel = BAT_CHANNEL;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStruct.ADC_ChannelCount = 1;
	ADC_InitStruct.ADC_ClockSource = ADC_ClockSource_BusClock;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_IRQCmd = DISABLE;
	ADC_InitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_InitStruct.ADC_RefSource = ADC_RefSource_VDD;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	drv_adc_Init(&ADC_InitStruct);
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
float bsp_bat_GetVol(void)
{
	uint8_t adc = 0;
	float vol = 0.0f;
	
	adc = drv_adc_ConvOnce(BAT_CHANNEL, ADC_Resolution_8b);
	vol = (adc* 5.0f/ 255) / (R2 / (R1 + R2)) ;
	
	return vol;
}

/********************************************  END OF FILE  *******************************************/

