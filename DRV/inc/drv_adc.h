/**
  *******************************************************************************************************
  * File Name: drv_adc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件为ADC外设提供了底层驱动函数.初始化函数等,同时定义了有关ADC的变量类型,宏定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __DRV_ADC_H
# define __DRV_ADC_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

/*  ADC采样通道  */
# define ADC_Channel_A0                               ((uint32_t)0x00000001)
# define ADC_Channel_A1                               ((uint32_t)0x00000002)
# define ADC_Channel_A6                               ((uint32_t)0x00000004)
# define ADC_Channel_A7                               ((uint32_t)0x00000008)
# define ADC_Channel_B0                               ((uint32_t)0x00000010)
# define ADC_Channel_B1                               ((uint32_t)0x00000020)
# define ADC_Channel_B2                               ((uint32_t)0x00000040)
# define ADC_Channel_B3                               ((uint32_t)0x00000080)
# define ADC_Channel_C1                               ((uint32_t)0x00000100)
# define ADC_Channel_C0                               ((uint32_t)0x00000200)
# define ADC_Channel_C2                               ((uint32_t)0x00000400)
# define ADC_Channel_C3                               ((uint32_t)0x00000800)
# define ADC_Channel_F4                               ((uint32_t)0x00001000)
# define ADC_Channel_F5                               ((uint32_t)0x00002000)
# define ADC_Channel_F6                               ((uint32_t)0x00004000)
# define ADC_Channel_F7                               ((uint32_t)0x00008000)
# define ADC_Channel_VSS						  	  						((uint32_t)0x00010000)
# define ADC_Channel_Temperature					  					((uint32_t)0x00020000)



/*  ADC时钟分频系数  */
# define ADC_Prescaler_Div1                         ((uint8_t)0x00)
# define ADC_Prescaler_Div2                         ((uint8_t)0x20)
# define ADC_Prescaler_Div3                         ((uint8_t)0x40)
# define ADC_Prescaler_Div4                         ((uint8_t)0x60)

/*  ADC采样位宽  */
# define ADC_Resolution_12b                         ((uint8_t)0x08)
# define ADC_Resolution_10b                         ((uint8_t)0x04)
# define ADC_Resolution_8b                          ((uint8_t)0x00)

/*  ADC时钟源  */
# define ADC_ClockSource_BusClock				    				((uint8_t)0x00)
# define ADC_ClockSource_BusClockDiv2								((uint8_t)0x01)
# define ADC_ClockSource_ALTClock										((uint8_t)0x10)
# define ADC_ClockSource_ADACKClock				 					((uint8_t)0x11)

/*  ADC参考电压源  */
# define ADC_RefSource_VREF													((uint8_t)0x00)
# define ADC_RefSource_VDD													((uint8_t)0x01)

/*  ADC初始化结构体定义  */
typedef struct
{
	uint32_t ADC_Channel;							/*  ADC 通道  */
	uint8_t ADC_ChannelCount;						/*  ADC通道数,也就是FIFO深度  */
	uint8_t ADC_IRQCmd;								/*  是否开启中断  */
	uint8_t ADC_Resolution;							/*  ADC数据位数  */
	FunctionalState ADC_ScanConvMode;				/*  扫描模式  */
	FunctionalState ADC_ContinuousConvMode;			/*  连续转换  */
	uint8_t ADC_Prescaler;							/*  ADC时钟分频  */
	uint8_t ADC_ClockSource;						/*  ADC时钟源  */
	uint8_t ADC_RefSource;							/*  ADC参考基准源  */
}ADC_InitTypeDef;


/*
  *******************************************************************************************************
  *                              FUNCTION DECLARE
  *******************************************************************************************************
*/
void drv_adc_Init(ADC_InitTypeDef *ADC_InitStruct);
void drv_adc_GetMultiADCResult(uint16_t *ADCBuff);
uint16_t drv_adc_GetSingleADCResult(uint32_t ADC_Channel);
uint16_t drv_adc_ConvOnce(uint32_t ADC_Channel, uint8_t ADC_Resolution);
void drv_adc_StartConv(void);

# endif

