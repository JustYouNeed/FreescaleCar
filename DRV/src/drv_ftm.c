/**
  *******************************************************************************************************
  * File Name: drv_ftm.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: KEA128芯片FTM底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Date: 2018-3-1
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
# include "drv.h"


static FTM_Type * const FTMX[] = FTM_BASES;
uint16_t period[3];

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
void drv_ftm_PWMInit(PWM_InitTypeDef *PWM_InitStruct)
{
	uint8_t FTMx = 0;
	uint8_t reg = 0;
	uint8_t offset = 0;
	uint8_t clrbit = 0;
	uint8_t channel = 0;
	uint8_t channelbit = 0;
	uint32_t pwmclk = 0;
	uint16_t mod = 0;
	uint8_t ps = 0;
	uint16_t cv = 0;
	uint16_t temp = 0;

	temp = PWM_InitStruct->PWM_Channel;
	offset = (uint8_t)((temp & 0x000f) >> 0);		/*  得到标志位偏移量  */
	clrbit = (uint8_t)((temp & 0x0030) >> 4);		/*  得到标志位清零量  */
	channel = (uint8_t)((temp & 0x03c0) >> 6);				/*  得到通道号  */
	channelbit = (uint8_t)((temp & 0x0c00) >> 10);
	FTMx = (uint8_t)((temp & 0x3000) >> 12);			/*  得到定时器编号  */
	reg = (uint8_t)((temp & 0x8000) >> 15);			/*  得到寄存器编号  */
	
	SIM->SCGC |= (1 << (FTMx + 5));		/*  开启相应时钟  */
	
	/*  先清除相应标志位,然后再设置  */
	if(reg == 0) 
	{
		SIM->PINSEL &= ~(clrbit << offset);
		SIM->PINSEL |= channelbit << offset;
	}
	else if(reg == 1)
	{
		SIM->PINSEL1 &= ~(clrbit << offset);
		SIM->PINSEL1 |= channelbit << offset;
	}
	
	pwmclk = SystemBusClock;
	mod = (pwmclk >> 16) / (PWM_InitStruct->PWM_Frequency * 1000);
	while((mod >> ps) >= 1) ps++;
	if(ps > 0x07) return ;
	mod = (pwmclk >> ps) / (PWM_InitStruct->PWM_Frequency * 1000);
	period[FTMx] = mod;
	
	cv = (PWM_InitStruct->PWM_Pulse * (mod -0 + 1)) / 1000;
	
	FTMX[FTMx]->MOD = mod;
	FTMX[FTMx]->CONTROLS[channel].CnSC &= ~FTM_CnSC_ELSA_MASK;
	FTMX[FTMx]->CONTROLS[channel].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTMX[FTMx]->SC = ( 0
									 | FTM_SC_PS(ps)             //分频因子，分频系数 = 2^PS
									 | FTM_SC_CLKS(1)            //时钟选择， 0：没选择时钟，禁用； 1：bus 时钟； 2：MCGFFCLK； 3：EXTCLK（ 由SIM_SOPT4 选择输入管脚 FTM_CLKINx）
								 );
	
	if(2 == FTMx)FTMX[FTMx]->CNTIN = 0;
	FTMX[FTMx]->CONTROLS[channel].CnV = cv;             //设置占空比
	FTMX[FTMx]->CNT = 0; 
}

/*
*********************************************************************************************************
*                          drv_ftm_PWMSetDuty                
*
* Description: 设置PWM通道占空比
*             
* Arguments  : 1> PWM_Channel: PWM通道
*							 2> duty: 占空比
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_ftm_PWMSetDuty(uint16_t PWM_Channel, uint16_t duty)
{
	uint16_t temp = PWM_Channel;
	uint8_t channel = (uint8_t)((temp & 0x03c0) >> 6);
	uint8_t ftmx = (uint8_t)((temp & 0x3000) >> 12);
	uint16_t pluse = 0;
	
	pluse = (duty *(period[ftmx] - 0 + 1)) / 1000;
	FTMX[ftmx]->CONTROLS[channel].CnV = pluse;
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
void drv_ftm_CounterInit()
{
	
}

/********************************************  END OF FILE  *******************************************/

