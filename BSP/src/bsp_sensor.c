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
	*		5.Author: Vector
	*			Date: 2018-6-12
	*			Mod: 增加后面两个水平电感,同时更改传感器数据采集逻辑
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
* Description: 获取传感器的ADC通道
*             
* Arguments  : 1>SensorID: 传感器编号
*
* Reutrn     : 传感器ADC通道
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
	int16_t MValue = 0;
	
	/*  循环处理每一个传感器的值  */
	for(cnt = 0; cnt < SENSOR_COUNT; cnt ++)
	{
		Car.Sensor[cnt].FIFO[Car.Sensor[cnt].Write++] = drv_adc_ConvOnce(bsp_sensor_GetSensorChannel(cnt), ADC_Resolution_8b);
		if(Car.Sensor[cnt].Write >= SENSOR_FIFO_SIZE) Car.Sensor[cnt].Write = 0;	/*  环形队列  */
		
		/*  滑动平均滤波器  */
		filter_SildingAverage(Car.Sensor[cnt].FIFO, &Car.Sensor[cnt].Average, SENSOR_FIFO_SIZE);
		
		/*  限幅，避免出现最值跳变  */
    if(Car.Sensor[cnt].Average < 5) 		Car.Sensor[cnt].Average = 5;
		
		/*  归一化处理  */
		Car.Sensor[cnt].NormalizedValue = (float)(Car.Sensor[cnt].Average - Car.Sensor[cnt].CalibrationMin) / 
																				 (Car.Sensor[cnt].CalibrationMax - Car.Sensor[cnt].CalibrationMin);
	}
	
			/*  计算水平差比和,扩大100倍  */
	Car.FHAE = 100 * ((Car.Sensor[S_F_H_R].NormalizedValue - Car.Sensor[S_F_H_L].NormalizedValue) / 
														(Car.Sensor[S_F_H_R].NormalizedValue + Car.Sensor[S_F_H_L].NormalizedValue)) + 0;
	
	Car.BHAE = 100 * ((Car.Sensor[S_B_H_R].NormalizedValue - Car.Sensor[S_B_H_L].NormalizedValue) / 
														(Car.Sensor[S_B_H_R].NormalizedValue + Car.Sensor[S_B_H_L].NormalizedValue)) + 0;
	/*  计算垂直和比差,扩大100倍  */
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
	/*  取得中间电感值  */
	MValue = Car.Sensor[S_M].Average;
	if(MValue < 80) 	/*  高通滤波,只有在圆环处中间电感的值才有效  */
	{
		Car.VAE = 0;		/*  垂直差比和也为0  */
		MValue = 0;
	}
		
	Car.VAE = (float)(Car.VAE * MValue / 100.0);		/*  合成新数据,用于判断圆环  */
	
	if(Car.VAE > -20 && Car.VAE < 20) Car.VAE = 0;		/*  高通滤波  */
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
	uint8_t adc = 0;
	uint32_t CalibrationValueTemp[SENSOR_COUNT * 2] = {0};
		
	oled_showString(0, 0, "Calibration...", 6, 12);
	oled_showChar(100, 0, '%', 6, 12, 1);
	oled_refreshGram();
	
	for(j = 0; j < SENSOR_COUNT * 2; j++)
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
	for(i = 0; i < 3000; i ++)
	{
		
		/*  循环处理每一个传感器  */
		for(j = 0; j < SENSOR_COUNT; j ++)
		{
			
			Car.Sensor[j].FIFO[Car.Sensor[j].Write++] = drv_adc_ConvOnce(bsp_sensor_GetSensorChannel(j), ADC_Resolution_8b);
			
			
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
	
	/*  对于两个垂直电感,进行手动标定  */
	Car.Sensor[S_V_L].CalibrationMax = 150;
	Car.Sensor[S_V_L].CalibrationMin = 0;
	
	Car.Sensor[S_V_R].CalibrationMax = 150;
	Car.Sensor[S_V_R].CalibrationMin = 0;
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



