# include "drv_ftm.h"

static FTM_Type * const FTMX[] = FTM_BASES;
uint16_t period[3];

void drv_ftm_BaseInit(FTM_Type *FTMx, FTM_BaseInitTypeDef *FTM_BaseInitStruct)
{
	uint32_t ftmclk ;
	uint16_t mod;
	uint8_t  ps = 0;
//	uint16_t cv;
//	uint16_t period = 0;
	
	if(FTMx == FTM0) drv_rcc_ClockCmd(RCC_PeriphClock_FTM0, ENABLE);
	else if(FTMx == FTM1) drv_rcc_ClockCmd(RCC_PeriphClock_FTM1, ENABLE);
	else if(FTMx == FTM2) drv_rcc_ClockCmd(RCC_PeriphClock_FTM2, ENABLE);
	
	FTMx->MODE |= 1 << 2;  /*  ȥ��д����  */
	FTMx->SC |= FTM_BaseInitStruct->FTM_ClockSource << 3;  /*  ����ʱ��Դ  */
//	FTMx->SC |= FTM_BaseInitStruct->FTM_Prescaler << 0;		 /*  ʱ��Ԥ��Ƶϵ��  */
	
	ftmclk = SystemTimerClock;
	mod = (ftmclk >> 16) * FTM_BaseInitStruct->FTM_Period;
	while((mod >> ps) >= 1) ps++;
	if(ps > 0x07) return ;
	mod = (ftmclk >> ps) * FTM_BaseInitStruct->FTM_Period;
//	period = mod;
	
	FTMx->SC |= FTM_SC_PS(ps);
	FTMx->MOD = mod;
	FTMx->CNT = 0;
}

void drv_ftm_SetChannel(FTM_Type *FTMx, uint8_t Channel)
{
	
}

void drv_ftm_OCInit(FTM_Type *FTMx, FTM_OCInitTypeDef *FTM_OCInitStruct)
{
	uint16_t mod = 0;
	uint16_t cv = 0;
	
	mod = FTMx->MOD;
	cv = (FTM_OCInitStruct->FTM_Pulse * (mod - 0 + 1)) / 1000;
	
	FTMx->CNTIN = 0;
	FTMx->CONTROLS[FTM_OCInitStruct->FTM_OCChannel].CnV = cv;
	FTMx->CONTROLS[FTM_OCInitStruct->FTM_OCChannel].CnSC &= ~FTM_CnSC_ELSA_MASK;
	FTMx->CONTROLS[FTM_OCInitStruct->FTM_OCChannel].CnSC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	
}

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
void drv_ftm_PWMSetDuty(uint16_t PWM_Channel, uint16_t duty)
{
	uint16_t temp = PWM_Channel;
	uint8_t channel = (uint8_t)((temp & 0x03c0) >> 6);
	uint8_t ftmx = (uint8_t)((temp & 0x3000) >> 12);
	uint16_t pluse = 0;
	
	pluse = (duty *(period[ftmx] - 0 + 1)) / 1000;
	FTMX[ftmx]->CONTROLS[channel].CnV = pluse;
}

