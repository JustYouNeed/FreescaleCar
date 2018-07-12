/**
  *******************************************************************************************************
  * File Name: drv_adc.h
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-1
  * Brief: ���ļ�ΪADC�����ṩ�˵ײ���������.��ʼ��������,ͬʱ�������й�ADC�ı�������,�궨��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: �����ļ�
  *
	*		2.Author:Vector
	*			Date:2018-5-3
	*			Mod: �޸�ADCͨ��8��������
	*
  *******************************************************************************************************
  */	

# ifndef __DRV_ADC_H
# define __DRV_ADC_H

/*  ADC����ͨ��  */
# define ADC_Channel_A0                               ((uint32_t)0x00000001)
# define ADC_Channel_A1                               ((uint32_t)0x00000002)
# define ADC_Channel_A6                               ((uint32_t)0x00000004)
# define ADC_Channel_A7                               ((uint32_t)0x00000008)
# define ADC_Channel_B0                               ((uint32_t)0x00000010)
# define ADC_Channel_B1                               ((uint32_t)0x00000020)
# define ADC_Channel_B2                               ((uint32_t)0x00000040)
# define ADC_Channel_B3                               ((uint32_t)0x00000080)
# define ADC_Channel_C0                               ((uint32_t)0x00000100)
# define ADC_Channel_C1                               ((uint32_t)0x00000200)
# define ADC_Channel_C2                               ((uint32_t)0x00000400)
# define ADC_Channel_C3                               ((uint32_t)0x00000800)
# define ADC_Channel_F4                               ((uint32_t)0x00001000)
# define ADC_Channel_F5                               ((uint32_t)0x00002000)
# define ADC_Channel_F6                               ((uint32_t)0x00004000)
# define ADC_Channel_F7                               ((uint32_t)0x00008000)
# define ADC_Channel_VSS						  	  						((uint32_t)0x00010000)
# define ADC_Channel_Temperature					  					((uint32_t)0x00020000)



/*  ADCʱ�ӷ�Ƶϵ��  */
# define ADC_Prescaler_Div1                         ((uint8_t)0x00)
# define ADC_Prescaler_Div2                         ((uint8_t)0x20)
# define ADC_Prescaler_Div3                         ((uint8_t)0x40)
# define ADC_Prescaler_Div4                         ((uint8_t)0x60)

/*  ADC����λ��  */
# define ADC_Resolution_12b                         ((uint8_t)0x02)
# define ADC_Resolution_10b                         ((uint8_t)0x01)
# define ADC_Resolution_8b                          ((uint8_t)0x00)

/*  ADCʱ��Դ  */
# define ADC_ClockSource_BusClock				    				((uint8_t)0x00)
# define ADC_ClockSource_BusClockDiv2								((uint8_t)0x01)
# define ADC_ClockSource_ALTClock										((uint8_t)0x10)
# define ADC_ClockSource_ADACKClock				 					((uint8_t)0x11)

/*  ADC�ο���ѹԴ  */
# define ADC_RefSource_VREF													((uint8_t)0x00)
# define ADC_RefSource_VDD													((uint8_t)0x01)

/*  ADC��ʼ���ṹ�嶨��  */
typedef struct
{
	uint32_t ADC_Channel;							/*  ADC ͨ��  */
	uint8_t ADC_ChannelCount;						/*  ADCͨ����,Ҳ����FIFO���  */
	uint8_t ADC_IRQCmd;								/*  �Ƿ����ж�  */
	uint8_t ADC_Resolution;							/*  ADC����λ��  */
	FunctionalState ADC_ScanConvMode;				/*  ɨ��ģʽ  */
	FunctionalState ADC_ContinuousConvMode;			/*  ����ת��  */
	uint8_t ADC_Prescaler;							/*  ADCʱ�ӷ�Ƶ  */
	uint8_t ADC_ClockSource;						/*  ADCʱ��Դ  */
	uint8_t ADC_RefSource;							/*  ADC�ο���׼Դ  */
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

