/**
  *******************************************************************************************************
  * File Name: bsp_sensor.c
  * Author: Vector
  * Version: V1.4.0
  * Date: 2018-3-1
  * Brief: 本文件为电磁传感器提供了基本的操作函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: 建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.添加新函数bsp_sensor_DataNormalized,bsp_sensor_DataCopy,bsp_sensor_Calibration
	*					 2.将独立的传感器数据整合到由Car结构体中,便于管理
	*
	*		3.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 传感器数据采集由中断改为查询方式
	*
	*		4.Author: Vector
	*			Date: 2018-3-27
	*			Mod: 将水平和比差与垂直和比差计算结果放大100倍,便于后续使用
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


uint16_t adc_once(void)
{
        //超频后，设置ADC的CLK为总线时钟/2
        ADC->SC3 = (0
                    | ADC_SC3_ADIV(1)           //分频系数
                    | ADC_SC3_MODE(0x0)         //分辨率
                    | ADC_SC3_ADICLK(0)         //使用总线时钟做为ADC得时钟源
                    //| ADC_SC3_ADLSMP_MASK       //1：长采样时间  0：短采样时间   注释为0 长采样时间采集更稳定
                    );
    

    ADC->SC1 = ADC_SC1_ADCH(8);       //启动转换
    
    while(!(ADC->SC1 & ADC_SC1_COCO_MASK)); //等待转换完成
    return (ADC->R & ADC_R_ADR_MASK);       //返回结果
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
	
	/*  循环处理每一个传感器的值  */
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
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  环形队列  */
		
		/*  滑动平均滤波器  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);	
		
		/*  归一化处理  */
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																				 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
	
	/*  计算水平差比和,扩大100倍  */
	Car.HorizontalAE = 100 * ((Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue) / 
														(Car.Sensor[SENSOR_H_R].NormalizedValue + Car.Sensor[SENSOR_H_L].NormalizedValue));
	
	/*  计算垂直和比差,扩大100倍  */
	Car.VecticalAE = 100 * ((Car.Sensor[SENSOR_V_R].NormalizedValue - Car.Sensor[SENSOR_V_L].NormalizedValue) / 
														(Car.Sensor[SENSOR_V_R].NormalizedValue + Car.Sensor[SENSOR_V_L].NormalizedValue));
	
	/*  计算和差比  */
	Car.AE = Car.HorizontalAE - Car.VecticalAE;
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
	uint16_t i = 0, j, cnt = 0;
	
	uint32_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
		
	oled_showString(0, 0, "Calibration...", 6, 12);
	oled_showChar(100, 0, '%', 6, 12, 1);
	oled_refreshGram();
	
	for(j = 0; i < SENSOR_COUNT * 2; i++)
	{
		CalibrationValueTemp[j] = 0;
	}
	/*  先假设最大最小值都为0  */
	for(j = 0; j < SENSOR_COUNT; j++)
	{
		Car.Sensor[j].CalibrationMax = 0;
		Car.Sensor[j].CalibrationMin = 0;
	}
	
	/*  平行移动车子,找出传感器的最大最小值  */
	for(; i < 3000; i ++)
	{
		
		/*  循环处理每一个传感器  */
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
			
			if(Car.Sensor[j].Write >= SENSOR_FIFO_SIZE) Car.Sensor[j].Write = 0;	/*  环形队列  */
			
			sort_QuickSort(Car.Sensor[j].FIFO, 0, SENSOR_FIFO_SIZE-1);	/*  对数据进行排序,由小到大  */
			
			/*  保存最大值  */
			if(Car.Sensor[j].CalibrationMax < Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1])		
				Car.Sensor[j].CalibrationMax = Car.Sensor[j].FIFO[SENSOR_FIFO_SIZE - 1];
			
			/*  找到最小值  */
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
	
	/*  先将各标定值暂存到缓存区,便于写入Flash  */
	for(j = 0; j < SENSOR_COUNT; j ++)
	{
		CalibrationValueTemp[j * 2] = Car.Sensor[j].CalibrationMax;
		CalibrationValueTemp[j * 2 + 1] = Car.Sensor[j].CalibrationMin;
	}
	
	oled_showString(0, 12, "Calibration OK!", 6 ,12);
	oled_showString(0, 24, "Saving...", 6, 12);
	oled_refreshGram();
	bsp_tim_DelayMs(1000);
	
	/*  保存标定最大值到FLASH  */
	drv_flash_EraseSector(SENSOR_PARA_FLASH_ADDR);
	drv_flash_WriteSector(SENSOR_PARA_FLASH_ADDR, (const uint8_t *)CalibrationValueTemp, SENSOR_COUNT * 8, 0);
	oled_showString(0,36, "Saved!!", 6, 12);
	oled_refreshGram();
	bsp_tim_DelayMs(1000);
}

/********************************************  END OF FILE  *******************************************/



