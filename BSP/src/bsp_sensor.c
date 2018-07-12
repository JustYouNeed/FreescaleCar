/**
  *******************************************************************************************************
  * File Name: bsp_sensor.c
  * Author: Vector
  * Version: V1.4.0
  * Date: 2018-3-1
  * Brief: ���ļ�Ϊ��Ŵ������ṩ�˻����Ĳ�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.����º���bsp_sensor_DataNormalized,bsp_sensor_DataCopy,bsp_sensor_Calibration
	*					 2.�������Ĵ������������ϵ���Car�ṹ����,���ڹ���
	*
	*		3.Author: Vector
	*			Date: 2018-3-26
	*			Mod: ���������ݲɼ����жϸ�Ϊ��ѯ��ʽ
	*
	*		4.Author: Vector
	*			Date: 2018-3-27
	*			Mod: ��ˮƽ�ͱȲ��봹ֱ�ͱȲ�������Ŵ�100��,���ں���ʹ��
	*
	*		5.Author: Vector
	*			Date: 2018-6-12
	*			Mod: ���Ӻ�������ˮƽ���,ͬʱ���Ĵ��������ݲɼ��߼�
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "app.h"
# include "FreescaleCar.h"

# define S_F_H_L_CH		ADC_Channel_B3
# define S_F_H_R_CH		ADC_Channel_C3
# define S_B_H_L_CH		ADC_Channel_C0
# define S_B_H_R_CH		ADC_Channel_C1
# define S_V_L_CH			ADC_Channel_F6
# define S_V_R_CH			ADC_Channel_C2
# define S_M_CH				ADC_Channel_F7	

/*
*********************************************************************************************************
*                         bsp_sensor_Config                 
*
* Description: ��ʼ����Ŵ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_sensor_Config(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Channel = S_F_H_L_CH | S_F_H_R_CH | S_B_H_L_CH | S_B_H_R_CH | S_V_L_CH | S_V_R_CH | S_M_CH;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStruct.ADC_ChannelCount = 1;
	ADC_InitStruct.ADC_ClockSource = ADC_ClockSource_BusClockDiv2;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_IRQCmd = DISABLE;
	ADC_InitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_InitStruct.ADC_RefSource = ADC_RefSource_VDD;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	drv_adc_Init(&ADC_InitStruct);
}


/*
*********************************************************************************************************
*                                   bsp_sensor_GetSensorChannel       
*
* Description: ��ȡ��������ADCͨ��
*             
* Arguments  : 1>SensorID: ���������
*
* Reutrn     : ������ADCͨ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t bsp_sensor_GetSensorChannel(uint8_t SensorID)
{
	uint32_t channel = 0;
	
	if(SensorID > SENSOR_COUNT) return 0;
	
	switch(SensorID)
	{
		case S_F_H_L: channel = S_F_H_L_CH;break;
		case S_F_H_R: channel = S_F_H_R_CH;break;
		case S_B_H_L: channel = S_B_H_L_CH;break;
		case S_B_H_R: channel = S_B_H_R_CH;break;
		case S_M: channel = S_M_CH;break;
		case S_V_L: channel = S_V_L_CH;break;
		case S_V_R: channel = S_V_R_CH;break;
	}
	
	return channel;
}

/*
*********************************************************************************************************
*                                bsp_sensor_DataNormalized          
*
* Description: ���������ݹ�һ������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ�˽�к���
*********************************************************************************************************
*/
void bsp_sensor_DataNormalized(void)
{
	uint8_t cnt = 0;
	
	/*  ѭ����һ��ÿһ���������Ĳ���  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																						 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
}


/*
*********************************************************************************************************
*                            bsp_sensor_DataCopy              
*
* Description: ���ݿ�������
*             
* Arguments  : 1> dst: Ŀ�껺����
*              2> src: ����Դ��ַ
*              3> length: Ҫ���������ݳ���
*
* Reutrn     : None.
*
* Note(s)    : Ŀ���ַ�Ĵ�С������С��Ҫ���������ݳ���
*********************************************************************************************************
*/
void bsp_sensor_DataCopy(uint16_t *dst, uint16_t *src, uint16_t length)
{
	while(length--) *dst++ = *src++;
}


/*
*********************************************************************************************************
*                           bsp_sensor_DataProcess               
*
* Description: ���������ݴ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ӧ�������Ե���
*********************************************************************************************************
*/
void bsp_sensor_DataProcess(void)
{	
	uint8_t cnt = 0;
	int16_t MValue = 0;
	
	/*  ѭ������ÿһ����������ֵ  */
	for(cnt = 0; cnt < SENSOR_COUNT; cnt ++)
	{
		Car.Sensor[cnt].FIFO[Car.Sensor[cnt].Write++] = drv_adc_ConvOnce(bsp_sensor_GetSensorChannel(cnt), ADC_Resolution_8b);
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  ���ζ���  */
		
		/*  ����ƽ���˲���  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);
		
		/*  �޷������������ֵ����  */
    if(Car.Sensor[cnt].Average < 5) 		Car.Sensor[cnt].Average = 5;
		
		/*  ��һ������  */
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																				 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
	
			/*  ����ˮƽ��Ⱥ�,����100��  */
	Car.FHAE = 100 * ((Car.Sensor[S_F_H_R].NormalizedValue - Car.Sensor[S_F_H_L].NormalizedValue) / 
														(Car.Sensor[S_F_H_R].NormalizedValue + Car.Sensor[S_F_H_L].NormalizedValue)) + 0;
	
	Car.BHAE = 100 * ((Car.Sensor[S_B_H_R].NormalizedValue - Car.Sensor[S_B_H_L].NormalizedValue) / 
														(Car.Sensor[S_B_H_R].NormalizedValue + Car.Sensor[S_B_H_L].NormalizedValue)) + 0;
	/*  ���㴹ֱ�ͱȲ�,����100��  */
	Car.VAE = 100 * ((Car.Sensor[S_V_R].NormalizedValue - Car.Sensor[S_V_L].NormalizedValue) / 
														(Car.Sensor[S_V_R].NormalizedValue + Car.Sensor[S_V_L].NormalizedValue)) + 0;	
	
//	Car.AE = (Car.Sensor[S_F_H_L].Average - Car.Sensor[S_F_H_R].Average) - (Car.Sensor[S_B_H_L].Average - Car.Sensor[S_B_H_R].Average);
	
	Car.Voltage = (float)(((drv_adc_ConvOnce(ADC_Channel_A1, ADC_Resolution_8b) * 5.0) / 255) / 0.425f);
	
//	Car.AE = 100 * ((Car.Sensor[S_B_H_R].NormalizedValue - Car.Sensor[S_B_H_L].NormalizedValue) /
//									(Car.Sensor[S_B_H_R].NormalizedValue + Car.Sensor[S_B_H_L].NormalizedValue));
//	Car.AE = Car.FHAE - Car.BHAE;
//	
//	
//	
	/*  ȡ���м���ֵ  */
	MValue = Car.Sensor[S_M].Average;
	if(MValue < 80) 	/*  ��ͨ�˲�,ֻ����Բ�����м��е�ֵ����Ч  */
	{
		Car.VAE = 0;		/*  ��ֱ��Ⱥ�ҲΪ0  */
		MValue = 0;
	}
		
	Car.VAE = (float)(Car.VAE * MValue / 100.0);		/*  �ϳ�������,�����ж�Բ��  */
	
	if(Car.VAE > -20 && Car.VAE < 20) Car.VAE = 0;		/*  ��ͨ�˲�  */
}

/*
*********************************************************************************************************
*                       bsp_sensor_Calibration                   
*
* Description: ����������У׼����,���ڱ�־�ܵ��ϸ��������������Сֵ
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_sensor_Calibration(void)
{
	uint16_t i = 0, j, cnt = 0;
	uint8_t adc = 0;
	uint32_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
		
	oled_showString(0, 0, "Calibration...", 6, 12);
	oled_showChar(100, 0, '%', 6, 12, 1);
	oled_refreshGram();
	
	for(j = 0; j < SENSOR_COUNT * 2; j++)
	{
		CalibrationValueTemp[j] = 0;
	}
	/*  �ȼ��������Сֵ��Ϊ0  */
	for(j = 0; j < SENSOR_COUNT; j++)
	{
		Car.Sensor[j].CalibrationMax = 0;
		Car.Sensor[j].CalibrationMin = 0;
	}
	
	/*  ƽ���ƶ�����,�ҳ��������������Сֵ  */
	for(i = 0; i < 3000; i ++)
	{
		
		/*  ѭ������ÿһ��������  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
			
			Car.Sensor[j].FIFO[Car.Sensor[j].Write++] = drv_adc_ConvOnce(bsp_sensor_GetSensorChannel(j), ADC_Resolution_8b);
			
			
			if(Car.Sensor[j].Write >= SENSOR_FIFO_SIZE) Car.Sensor[j].Write = 0;	/*  ���ζ���  */
			
			sort_QuickSort(Car.Sensor[j].FIFO, 0, SENSOR_FIFO_SIZE-1);	/*  �����ݽ�������,��С����  */
			
			/*  �������ֵ  */
			if(Car.Sensor[j].CalibrationMax < Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1])		
				Car.Sensor[j].CalibrationMax = Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1];
			
			/*  �ҵ���Сֵ  */
			if(Car.Sensor[j].CalibrationMin > Car.Sensor[j].FIFO[0])
				Car.Sensor[j].CalibrationMin = Car.Sensor[j].FIFO[0];
		}
		
		cnt ++;
		if(cnt % 10 == 0) 
		{
			bsp_led_Toggle(0);	
			oled_refreshGram();
		}
		oled_showNum(84, 0, (i+1)/30, 3, 6, 12);
		bsp_tim_DelayMs(5);
	} /*  end of for  */
	
	/*  ����������ֱ���,�����ֶ��궨  */
	Car.Sensor[S_V_L].CalibrationMax = 150;
	Car.Sensor[S_V_L].CalibrationMin = 0;
	
	Car.Sensor[S_V_R].CalibrationMax = 150;
	Car.Sensor[S_V_R].CalibrationMin = 0;
	/*  �Ƚ����궨ֵ�ݴ浽������,����д��Flash  */
	for(j = 0; j < SENSOR_COUNT; j ++)
	{
		CalibrationValueTemp[j * 2] = Car.Sensor[j].CalibrationMax;
		CalibrationValueTemp[j * 2 + 1] = Car.Sensor[j].CalibrationMin;
	}
	
	oled_showString(0, 12, "Calibration OK!", 6 ,12);
	oled_showString(0, 24, "Saving...", 6, 12);
	oled_refreshGram();
	bsp_tim_DelayMs(1000);
	
	/*  ����궨���ֵ��FLASH  */
	drv_flash_EraseSector(SENSOR_PARA_FLASH_ADDR);
	drv_flash_WriteSector(SENSOR_PARA_FLASH_ADDR, (const uint8_t *)CalibrationValueTemp, SENSOR_COUNT * 8, 0);
	oled_showString(0,36, "Saved!!", 6, 12);
	oled_refreshGram();
	bsp_tim_DelayMs(1000);
}

/********************************************  END OF FILE  *******************************************/



