/**
  *******************************************************************************************************
  * File Name: drv_adc.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: KEA128芯片ADC底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Data: 2018-3-1
	*     Author: Vector
	*     Mod: 建立文件,添加基本函数
  *
  *******************************************************************************************************
  */
	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_adc.h"
# include "drv_rcc.h"


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/

static uint16_t 	ADC_Result[24] = {0};		/*  暂存ADC转换结果  */
static uint8_t  	ADC_Count = 0;					/*  使用的ADC通道数  */
static uint32_t   ADC_Order[24] = {0};		/*  ADC通道顺序,用来获取单个ADC转换结果  */


/*
*********************************************************************************************************
*                              drv_adc_Init            
*
* Description: 初始化ADC外设
*             
* Arguments  : 1> ADC_Initstruct: ADC初始化结构体指针
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_adc_Init(ADC_InitTypeDef *ADC_InitStruct)
{
	uint32_t posbit = 0, channel = 0;
	uint8_t reg = 0;
	uint8_t i = 0, cnt = 0;
	
	drv_rcc_ClockCmd(RCC_PeriphClock_ADC, ENABLE);  /*  开启ADC时钟  */
	
	ADC->APCTL1 |= ADC_InitStruct->ADC_Channel;     /*  设置ADC采样通道  */
	ADC->SC3 |= ADC_InitStruct->ADC_Resolution;			/*  设置ADC采样位数  */
	ADC->SC3 |= ADC_InitStruct->ADC_Prescaler;			/*  时钟分频  */
	ADC->SC2 |= ADC_InitStruct->ADC_ClockSource;		/*  时钟源  */
	ADC->SC2 |= ADC_InitStruct->ADC_RefSource;			/*  参考电压源  */
	
	if(ADC_InitStruct->ADC_ContinuousConvMode == ENABLE)	/*  如果开启了连续转换模式  */
		reg |= ADC_SC1_ADCO_MASK;
	
	if(ADC_InitStruct->ADC_IRQCmd == ENABLE)		/*  如果使能了中断  */
	{
		reg |= ADC_SC1_AIEN_MASK;
		NVIC_EnableIRQ(ADC_IRQn);
	}
	
	if(ADC_InitStruct->ADC_ScanConvMode != DISABLE)		/*  如果开启了扫描模式  */
	{
		ADC->SC4 |= ADC_SC4_ASCANE_MASK;
	}
	
	/*  如果设置了FIFO采样深度,则需要将需要采样的ADC通道连续写入FIFO,直到写满,系统才会启动采样  */
	channel = ADC_InitStruct->ADC_Channel;
	ADC->SC4 |= (uint8_t)(ADC_InitStruct->ADC_ChannelCount - 1); /*  设置FIFO深度  */
	
	for(i = 0; i < 16; i++)	/*  一位一位检查  */
	{
		posbit = (uint32_t)((channel >> i) & 0x01);
		if(posbit)		/*  如果开启了该通道  */
		{
			ADC->SC1 = (ADC_SC1_ADCH(i) | reg);
			
			ADC_Order[ADC_Count++] = (uint32_t)(1 << i);
			cnt++;
		}
		
		/*  已经设置完成了所有通道  */
		if(cnt >= ADC_InitStruct->ADC_ChannelCount) break;	
	}
	
	/*  如果开启了测量VSS  */
	if(channel & ADC_Channel_VSS)	
	{
		ADC->SC1 = (ADC_SC1_ADCH(16) | reg);
		ADC_Order[ADC_Count++] = ADC_Channel_VSS;
	}
	
	/*  开启了内部温度测量  */
	if(channel & ADC_Channel_Temperature)
	{
		ADC->SC1 = (ADC_SC1_ADCH(22) | reg);
		ADC_Order[ADC_Count++] = ADC_Channel_Temperature;
	}
}



/*
*********************************************************************************************************
*                                ADC0_IRQHandler          
*
* Description: ADC中断函数,保存转换结果
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void ADC0_IRQHandler(void)
{
	uint8_t i = 0;
	
	for(; i < ADC_Count; i++)
	{
		ADC_Result[i] = ADC->R;
	}
}


/*
*********************************************************************************************************
*                         drv_adc_GetMultiADCResult                 
*
* Description: 获取多个ADC转换结果
*             
* Arguments  : 1> ADCBuff: ADC结果缓存区指针
*
* Reutrn     : None.
*
* Note(s)    : 注意,该缓存区的大小不可小于所使用的数量
*********************************************************************************************************
*/
void drv_adc_GetMultiADCResult(uint16_t *ADCBuff)
{
	uint8_t i = 0;
	
	for(; i < ADC_Count; i++)		
	{
		ADCBuff[i] = ADC_Result[i];
	}
}


/*
*********************************************************************************************************
*                              drv_adc_GetSingleADCResult            
*
* Description: 多通道ADC采样时获取单个通道的采样结果
*             
* Arguments  : 1> ADC_Channel: ADC通道
*
* Reutrn     : 1> 0xffff:通道错误
*              2> 对应通道的转换结果
*
* Note(s)    : 该函数只能在ADC开启了多通道采样时才能用
*********************************************************************************************************
*/
uint16_t drv_adc_GetSingleADCResult(uint32_t ADC_Channel)
{
	uint16_t cnt = 0;
	
	for(; cnt < ADC_Count; cnt++)		/*  得到通道  */
	{
		if(ADC_Order[cnt] == ADC_Channel) break;
	}
	if(cnt >= ADC_Count) return 0xffff;	/*  未找到通道  */
	
	return ADC_Result[cnt];
}

/*
*********************************************************************************************************
*                             drv_adc_ConvOnce             
*
* Description: 转换一个通道的ADC一次
*             
* Arguments  : 1> ADC_Channel: 要转换的ADC通道
*              2> ADC_Resolution: ADC转换位数
*
* Reutrn     : 1> ADC转换结果
*
* Note(s)    : 该函数只能转换一个通道的结果,如果输入通道有多个,则转换高位通道
*********************************************************************************************************
*/
uint16_t drv_adc_ConvOnce(uint32_t ADC_Channel, uint8_t ADC_Resolution)
{
	uint16_t channel = 0;
	uint16_t result = 0;
	
	for(; channel < 16; channel++)		/*  得到通道  */
	{
		if((ADC_Channel >> channel) & 0x01) break;
	}
	
	if(ADC_Channel & ADC_Channel_VSS) channel = 16;
	if(ADC_Channel & ADC_Channel_Temperature) channel = 22;
	
	ADC->SC3 |= ADC_Resolution;			/*  设置转换位数  */
	ADC->SC3 |= ADC_SC3_ADIV(0);		/*  不分频  */
	ADC->SC3 |= ADC_SC3_ADICLK(0);	/*  使用总线时钟  */

	ADC->SC2 |= ADC_SC2_REFSEL(0);	/*  外部参考电压  */
	
	ADC->SC1 = ADC_SC1_ADCH(channel);				/*  启动转换  */
	while(!(ADC->SC1 & ADC_SC1_COCO_MASK));	/*  等待转换完成  */
	result = ADC->R;
	ADC_SC1 &= ~ADC_SC1_COCO_MASK;	/*  清除转换完成标志  */
	return (result);       /*  返回转换结果  */
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
void drv_adc_StartConv(void)
{
//	uint8_t i = 0;
//	
//	ADC->SC1 = ADC->SC1;
//	while(!(ADC->SC1 & ADC_SC1_COCO_MASK));
//	
//	for(; i < ADC_Count; i++)
//	{
//		ADC_Result[i] = ADC->R;
//	}
	uint8_t reg = ADC->SC1;
	
	reg |= 1 << 6;
	
	ADC->SC1 = reg;
}

/********************************************  END OF FILE  *******************************************/
