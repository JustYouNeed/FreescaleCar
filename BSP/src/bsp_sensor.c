/**
  *******************************************************************************************************
  * File Name: bsp_sensor.c
  * Author: Vector
  * Version: V1.3.0
  * Date: 2018-3-1
  * Brief: 本文件为电磁传感器提供了基本的操作函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 建立文件
	*
	*		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.添加新函数bsp_sensor_DataNormalized,bsp_sensor_DataCopy,bsp_sensor_Calibration
	*					 2.将独立的传感器数据整合到由Car结构体中,便于管理
	*
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 传感器数据采集由中断改为查询方式
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_sensor.h"
# include "app_sort.h"
# include "app_filter.h"
# include "FreescaleCar.h"

extern uint16_t ADC_Value[SENSOR_COUNT];


/*
*********************************************************************************************************
*                         bsp_sensor_Config                 
*
* Description: 初始化电磁传感器
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
	
	ADC_InitStruct.ADC_Channel = SENSOR_1 | SENSOR_2 | SENSOR_3 | SENSOR_4;
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
* Description: 传感器数据归一化处理
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数为本文件私有函数
*********************************************************************************************************
*/
void bsp_sensor_DataNormalized(void)
{
	uint8_t cnt = 0;
	
	/*  循环归一化每一个传感器的参数  */
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
* Description: 数据拷贝函数
*             
* Arguments  : 1> dst: 目标缓存区
*              2> src: 数据源地址
*              3> length: 要拷贝的数据长度
*
* Reutrn     : None.
*
* Note(s)    : 目标地址的大小不可以小于要拷贝的数据长度
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
* Description: 传感器数据处理函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数应该周期性调用
*********************************************************************************************************
*/
void bsp_sensor_DataProcess(void)
{	
	uint8_t cnt = 0;
	
//	/*  用于暂存各个传感器的数据FIFO,并用作排序,因为FIFO由于要用作滑动平均滤波,不能排序  */
//	uint16_t ADC_ValueTemp[SENSOR_FIFO_SIZE] = {0};	
	
	/*  用于从ADC寄存器中读取出转换结果  */
//	uint16_t ADC_Value[SENSOR_COUNT];
	
	/*  获取各个传感器的电压值  */
//	drv_adc_GetMultiADCResult(ADC_Value);
	
	/*  循环处理每一个传感器的值  */
	for(; cnt < SENSOR_COUNT; cnt ++)
	{
//		Car.Sensor[cnt].FIFO[Car.Sensor[cnt].Write++] = ADC_Value[cnt];		/*  写入FIFO  */
		switch(cnt)
		{
			case SENSOR_H_L: Car.Sensor[SENSOR_H_L].FIFO[Car.Sensor[SENSOR_H_L].Write++] = drv_adc_ConvOnce(SENSOR_1, ADC_Resolution_8b);break;
			case SENSOR_V_L: Car.Sensor[SENSOR_V_L].FIFO[Car.Sensor[SENSOR_V_L].Write++] = drv_adc_ConvOnce(SENSOR_2, ADC_Resolution_8b);break;
			case SENSOR_H_R: Car.Sensor[SENSOR_H_R].FIFO[Car.Sensor[SENSOR_H_R].Write++] = drv_adc_ConvOnce(SENSOR_3, ADC_Resolution_8b);break;
			case SENSOR_V_R: Car.Sensor[SENSOR_V_R].FIFO[Car.Sensor[SENSOR_V_R].Write++] = drv_adc_ConvOnce(SENSOR_4, ADC_Resolution_8b);break;
		}
		
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  环形队列  */
		
//		/*  暂存数据  */
//		bsp_sensor_DataCopy(ADC_ValueTemp, Car.Sensor[cnt].FIFO, SENSOR_FIFO_SIZE);
//		
//		sort_QuickSort(ADC_ValueTemp, 0, SENSOR_COUNT-1);		/*  快速排序,由小到大  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);	/*  滑动平均滤波器  */
		bsp_sensor_DataNormalized();		/*  归一化  */
	}
	
	/*  计算水平差比和  */
	Car.HorizontalAE = ((Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue) / 
														(Car.Sensor[SENSOR_H_R].NormalizedValue + Car.Sensor[SENSOR_H_L].NormalizedValue));
	/*  垂直差比和  */
	Car.VecticalAE = ((Car.Sensor[SENSOR_V_L].NormalizedValue - Car.Sensor[SENSOR_V_R].NormalizedValue) / 
													(Car.Sensor[SENSOR_V_L].NormalizedValue + Car.Sensor[SENSOR_V_R].NormalizedValue));
}

/*
*********************************************************************************************************
*                       bsp_sensor_Calibration                   
*
* Description: 传感器数据校准函数,用于标志跑道上各传感器的最大最小值
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
	uint16_t i = 0, j;
	
//	uint16_t ADC_ValueTemp[SENSOR_COUNT];
	uint16_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
	
	oled_showString(0, 0, (uint8_t *)"Calibration...", 8, 16);
	oled_refreshGram();
	
	/*  先假设最大最小值都为0  */
	for(j = 0; j < SENSOR_COUNT; j++)
	{
		Car.Sensor[j].CalibrationMax = 0;
		Car.Sensor[j].CalibrationMin = 0;
	}
	
	/*  平行移动车子,找出传感器的最大最小值  */
	for(; i < 3000; i ++)
	{
		/*  获取传感器电压值  */
//		drv_adc_GetMultiADCResult(ADC_ValueTemp);
		
		/*  循环处理每一个传感器  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
//			Car.Sensor[j].FIFO[Car.Sensor[j].Write++] = ADC_ValueTemp[j];					/*  保存电压值  */
			switch(j)
			{
				case SENSOR_H_L: Car.Sensor[SENSOR_H_L].FIFO[Car.Sensor[SENSOR_H_L].Write++] = drv_adc_ConvOnce(SENSOR_1, ADC_Resolution_8b);break;
				case SENSOR_V_L: Car.Sensor[SENSOR_V_L].FIFO[Car.Sensor[SENSOR_V_L].Write++] = drv_adc_ConvOnce(SENSOR_2, ADC_Resolution_8b);break;
				case SENSOR_H_R: Car.Sensor[SENSOR_H_R].FIFO[Car.Sensor[SENSOR_H_R].Write++] = drv_adc_ConvOnce(SENSOR_3, ADC_Resolution_8b);break;
				case SENSOR_V_R: Car.Sensor[SENSOR_V_R].FIFO[Car.Sensor[SENSOR_V_R].Write++] = drv_adc_ConvOnce(SENSOR_4, ADC_Resolution_8b);break;
			}
			if(Car.Sensor[j].Write >= SENSOR_FIFO_SIZE) Car.Sensor[j].Write = 0;	/*  环形FIFO  */
			
			sort_QuickSort(Car.Sensor[j].FIFO, 0, SENSOR_FIFO_SIZE-1);	/*  对数据进行排序,由小到大  */
			
			/*  保存最大值  */
			if(Car.Sensor[j].CalibrationMax < Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1])		
				Car.Sensor[j].CalibrationMax = Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1];
			
			/*  找到最小值  */
			if(Car.Sensor[j].CalibrationMin > Car.Sensor[j].FIFO[0])
				Car.Sensor[j].CalibrationMin = Car.Sensor[j].FIFO[0];
		}
		bsp_tim_DelayMs(5);
	}
	
	/*  先将各标定值暂存到缓存区,便于写入Flash  */
	for(j = 0; j < SENSOR_COUNT; j ++)
	{
		CalibrationValueTemp[j] = Car.Sensor[j].CalibrationMax;
		CalibrationValueTemp[j + SENSOR_COUNT] = Car.Sensor[j + SENSOR_COUNT].CalibrationMin;
	}
	
	oled_showString(0, 2, (uint8_t *)"Calibration OK!", 8 ,16);
	oled_refreshGram();
//	bsp_tim_DelayMs(500);
	/*  保存标定最大值到FLASH  */
	drv_flash_EraseSector(SENSOR_PARA_FLASH_ADDR);
	drv_flash_WriteSector(SENSOR_PARA_FLASH_ADDR, (const uint8_t *)CalibrationValueTemp, SENSOR_COUNT * 4, 0);
}

/********************************************  END OF FILE  *******************************************/



