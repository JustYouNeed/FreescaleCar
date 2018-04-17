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
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app_sort.h"
# include "app_filter.h"
# include "FreescaleCar.h"
# include "app_debug.h"


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
	
	ADC_InitStruct.ADC_Channel = ADC_Channel_C2 | ADC_Channel_C3 | ADC_Channel_F6 | ADC_Channel_F7 | ADC_Channel_C0 | ADC_Channel_A1;
//	ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
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


uint16_t adc_once(void)
{
        //��Ƶ������ADC��CLKΪ����ʱ��/2
        ADC->SC3 = (0
                    | ADC_SC3_ADIV(1)           //��Ƶϵ��
                    | ADC_SC3_MODE(0x0)         //�ֱ���
                    | ADC_SC3_ADICLK(0)         //ʹ������ʱ����ΪADC��ʱ��Դ
                    //| ADC_SC3_ADLSMP_MASK       //1��������ʱ��  0���̲���ʱ��   ע��Ϊ0 ������ʱ��ɼ����ȶ�
                    );
    

    ADC->SC1 = ADC_SC1_ADCH(8);       //����ת��
    
    while(!(ADC->SC1 & ADC_SC1_COCO_MASK)); //�ȴ�ת�����
    return (ADC->R & ADC_R_ADR_MASK);       //���ؽ��
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
	
	/*  ѭ������ÿһ����������ֵ  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
		switch(cnt)
		{
			case SENSOR_H_L: Car.Sensor[SENSOR_H_L].FIFO[Car.Sensor[SENSOR_H_L].Write++] = drv_adc_ConvOnce(ADC_Channel_F6, ADC_Resolution_8b);break;
			case SENSOR_H_R: Car.Sensor[SENSOR_H_R].FIFO[Car.Sensor[SENSOR_H_R].Write++] = drv_adc_ConvOnce(ADC_Channel_C3, ADC_Resolution_8b);break;
			case SENSOR_V_L: Car.Sensor[SENSOR_V_L].FIFO[Car.Sensor[SENSOR_V_L].Write++] = drv_adc_ConvOnce(ADC_Channel_F7, ADC_Resolution_8b);break;
			case SENSOR_V_R: Car.Sensor[SENSOR_V_R].FIFO[Car.Sensor[SENSOR_V_R].Write++] = drv_adc_ConvOnce(ADC_Channel_C2, ADC_Resolution_8b);break;
			case SENSOR_M: Car.Sensor[SENSOR_M].FIFO[Car.Sensor[SENSOR_M].Write++] = adc_once();break;//drv_adc_ConvOnce(ADC_Channel_C0, ADC_Resolution_8b);break;
		}
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  ���ζ���  */
		
		/*  ����ƽ���˲���  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);	
		
		/*  ��һ������  */
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																				 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
	
	/*  ����ˮƽ��Ⱥ�,����100��  */
	Car.HorizontalAE = 100 * ((Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue) / 
														(Car.Sensor[SENSOR_H_R].NormalizedValue + Car.Sensor[SENSOR_H_L].NormalizedValue));
	
	/*  ���㴹ֱ�ͱȲ�,����100��  */
	Car.VecticalAE = 100 * ((Car.Sensor[SENSOR_V_R].NormalizedValue - Car.Sensor[SENSOR_V_L].NormalizedValue) / 
														(Car.Sensor[SENSOR_V_R].NormalizedValue + Car.Sensor[SENSOR_V_L].NormalizedValue));
	
	/*  ����Ͳ��  */
	Car.AE = Car.HorizontalAE - Car.VecticalAE;
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
	
	uint32_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
		
	oled_showString(0, 0, "Calibration...", 6, 12);
	oled_showChar(100, 0, '%', 6, 12, 1);
	oled_refreshGram();
	
	for(j = 0; i < SENSOR_COUNT * 2; i++)
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
	for(; i < 3000; i ++)
	{
		
		/*  ѭ������ÿһ��������  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
			switch(j)
			{
				case SENSOR_H_L: Car.Sensor[SENSOR_H_L].FIFO[Car.Sensor[SENSOR_H_L].Write++] = drv_adc_ConvOnce(ADC_Channel_F6, ADC_Resolution_8b);break;
				case SENSOR_H_R: Car.Sensor[SENSOR_H_R].FIFO[Car.Sensor[SENSOR_H_R].Write++] = drv_adc_ConvOnce(ADC_Channel_C3, ADC_Resolution_8b);break;
				case SENSOR_V_L: Car.Sensor[SENSOR_V_L].FIFO[Car.Sensor[SENSOR_V_L].Write++] = drv_adc_ConvOnce(ADC_Channel_F7, ADC_Resolution_8b);break;
				case SENSOR_V_R: Car.Sensor[SENSOR_V_R].FIFO[Car.Sensor[SENSOR_V_R].Write++] = drv_adc_ConvOnce(ADC_Channel_C2, ADC_Resolution_8b);break;
				case SENSOR_M: Car.Sensor[SENSOR_M].FIFO[Car.Sensor[SENSOR_M].Write++] = drv_adc_ConvOnce(ADC_Channel_C0, ADC_Resolution_8b);break;
			}
			
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



