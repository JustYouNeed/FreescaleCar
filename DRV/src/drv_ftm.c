/**
  *******************************************************************************************************
  * File Name: drv_ftm.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: KEA128оƬFTM�ײ���������
  *******************************************************************************************************
  * History
  *		1.Date: 2018-3-1
	*     Author: Vector
	*     Mod: �����ļ�,��ӻ�������
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
	offset = (uint8_t)((temp & 0x000f) >> 0);		/*  �õ���־λƫ����  */
	clrbit = (uint8_t)((temp & 0x0030) >> 4);		/*  �õ���־λ������  */
	channel = (uint8_t)((temp & 0x03c0) >> 6);				/*  �õ�ͨ����  */
	channelbit = (uint8_t)((temp & 0x0c00) >> 10);
	FTMx = (uint8_t)((temp & 0x3000) >> 12);			/*  �õ���ʱ�����  */
	reg = (uint8_t)((temp & 0x8000) >> 15);			/*  �õ��Ĵ������  */
	
	SIM->SCGC |= (1 << (FTMx + 5));		/*  ������Ӧʱ��  */
	
	/*  �������Ӧ��־λ,Ȼ��������  */
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
									 | FTM_SC_PS(ps)             //��Ƶ���ӣ���Ƶϵ�� = 2^PS
									 | FTM_SC_CLKS(1)            //ʱ��ѡ�� 0��ûѡ��ʱ�ӣ����ã� 1��bus ʱ�ӣ� 2��MCGFFCLK�� 3��EXTCLK�� ��SIM_SOPT4 ѡ������ܽ� FTM_CLKINx��
								 );
	
	if(2 == FTMx)FTMX[FTMx]->CNTIN = 0;
	FTMX[FTMx]->CONTROLS[channel].CnV = cv;             //����ռ�ձ�
	FTMX[FTMx]->CNT = 0; 
}

/*
*********************************************************************************************************
*                          drv_ftm_PWMSetDuty                
*
* Description: ����PWMͨ��ռ�ձ�
*             
* Arguments  : 1> PWM_Channel: PWMͨ��
*							 2> duty: ռ�ձ�
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

