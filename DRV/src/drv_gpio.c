/**
  *******************************************************************************************************
  * File Name: drv_pit.c
  * Author: Vector
  * Version: V2.2.0
  * Date: 2018-2-1
  * Brief: KEA128оƬGPIO�ײ���������
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *     Author: Vector
  *     Mod: �����ļ�,��ӻ�������
	*		
	*		2.Date: 2018-2-9
	*			Author:	Vector
	*			Mod: ����º���drv_gpio_PinAFConfig,�����������ŵĸ��ù���
	*
	*		3.Date:2018-2-28
	*			Author: Vector
	*			Mod: 1.����º���drv_gpio_PullCmd,�����������ŵ���������
	*					 2.�޸����ų�ʼ���߼�,���GPIO�˿�����GPIOX[],
	*	
  *
  *******************************************************************************************************
  */
	
# include "drv_gpio.h"

static GPIO_Type * const GPIOX[] = GPIO_BASES;

/*
*********************************************************************************************************
*                                       drv_gpio_Init   
*
* Description: ��ʼ��һ��GPIO����
*             
* Arguments  : 1> GPIO_InitStruct: GPIO�������ýṹ��ָ��
*
* Reutrn     : None.
*
* Note(s)    : һ��ֻ������һ������
*********************************************************************************************************
*/
void drv_gpio_Init(GPIO_InitTypeDef *GPIO_InitStruct)
{
	uint32_t reg = 0;
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_InitStruct->GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  �����GPIO�˿�  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  �����GPIO����  */
	
	GPIOX[gpiox]->PIDR |= ((uint32_t)1 << pinx);	/*  �Ƚ�ֹ����  */
	GPIOX[gpiox]->PDDR &= ~((uint32_t)1 << pinx);
	
	/*  ���ݳ�ʼ���ṹ���ֵ����Ϊ��Ӧ������/���  */
	if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IN)
	{
		reg = GPIOX[gpiox]->PIDR;
		reg &= ~((uint32_t)1 << pinx);
		GPIOX[gpiox]->PIDR = reg;
	}
	else if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT)
	{
		reg = GPIOX[gpiox]->PIDR;
		reg |= ((uint32_t)1 << pinx);
		GPIOX[gpiox]->PIDR = reg;
		
		reg = GPIOX[gpiox]->PDDR;
		reg |= ((uint32_t)1 << pinx);
		GPIOX[gpiox]->PDDR = reg;
	}
	
	/*  �������������  */
	if(GPIO_InitStruct->GPIO_PuPd == GPIO_PuPd_UP)
	{
		/*  GPIOA,GPIOB,���������õļĴ�����ͬ  */
		if(GPIOA == GPIOX[gpiox]) PORT->PUE0 |= 1 << pinx;
		else if(GPIOB == GPIOX[gpiox]) PORT->PUE1 |= 1 << pinx;
		else	if(GPIOC == GPIOX[gpiox])PORT->PUE2 |= 1 << pinx;
	}
	
	/*  �������������ֻ�й̶��ļ���������  */
	if(GPIO_InitStruct->GPIO_HDrv == ENABLE)
	{
		if(GPIOX[gpiox] == PORTH)
		{
			switch(GPIO_InitStruct->GPIO_Pin)
			{
				case GPIO_Pin_H1:PORT->HDRVE |= 1 << 7;break;
				case GPIO_Pin_H0:PORT->HDRVE |= 1 << 6;break;
				default: break;
			}
		}
		if(GPIOX[gpiox] == PORTE)
		{
			switch(GPIO_InitStruct->GPIO_Pin)
			{
				case GPIO_Pin_E1:PORT->HDRVE |= 1 << 5;break;
				case GPIO_Pin_E0:PORT->HDRVE |= 1 << 4;break;
				default: break;
			}
		}
		if(GPIOX[gpiox] == PORTD)
		{
			switch(GPIO_InitStruct->GPIO_Pin)
			{
				case GPIO_Pin_D1:PORT->HDRVE |= 1 << 3;break;
				case GPIO_Pin_D0:PORT->HDRVE |= 1 << 2;break;
				default: break;
			}
		}
		if(GPIOX[gpiox] == PORTB)
		{
			switch(GPIO_InitStruct->GPIO_Pin)
			{
				case GPIO_Pin_B5:PORT->HDRVE |= 1 << 1;break;
				case GPIO_Pin_B4:PORT->HDRVE |= 1 << 0;break;
				default: break;
			}
		}
	}
}

/*
*********************************************************************************************************
*                                  drv_gpio_ReadPin        
*
* Description: ��ȡһ�����ŵĵ�ƽֵ
*             
* Arguments  : 1> GPIO_Pin: ���ű��
*
* Reutrn     : ���ŵ�ƽ
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_gpio_ReadPin(uint8_t GPIO_Pin)
{
	uint32_t pin;
	
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  �����GPIO�˿�  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  �����GPIO����  */
	
	/*  ��ȡ���ŵ�ƽ  */
	pin = (uint32_t)GPIOX[gpiox]->PDIR;
	
	return ((pin >> pinx) & 0x01);
}

/*
*********************************************************************************************************
*                                drv_gpio_WritePin          
*
* Description: д���ŵ�ƽ
*             
* Arguments  : 1> GPIO_Pin: ���ű��
*              2> NewState: ö�ٱ���,SET ���� RESET
*
* Reutrn     : Nono.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_gpio_WritePin(uint8_t GPIO_Pin, GPIO_PinState PinState)
{
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  �����GPIO�˿�  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  �����GPIO����  */
	
	if(PinState == GPIO_PIN_RESET)  /*  ������õ͵�ƽ  */
	{
		GPIOX[gpiox]->PCOR |= 1 << pinx;
	}
	else
	{
		GPIOX[gpiox]->PSOR |= 1 << pinx;
	}
}

/*
*********************************************************************************************************
*                                  drv_gpio_TogglePin        
*
* Description: ��ת���ŵ�ƽ
*             
* Arguments  : 1> GPIO_Pin: ���ű��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_gpio_TogglePin(uint8_t GPIO_Pin)
{
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  �����GPIO�˿�  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  �����GPIO����  */
	
	GPIOX[gpiox]->PTOR |= 1 << pinx;		/*  GPIOX[gpiox]->PTOR�Ĵ���,����Ĵ���  */
}


/*
*********************************************************************************************************
*                                    drv_gpio_PinAFConfig      
*
* Description: �������ŵĸ��ù���
*             
* Arguments  : 1> GPIO_PinSource: GPIO������Դ,��drv_gpio.h��������������Դ�Ķ���
*              2> GPIO_AF       : Ҫ���õĹ���,��drv_gpio.h�������и��ù��ܵĶ���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_gpio_PinAFConfig(uint8_t GPIO_PinSource, uint16_t GPIO_AF)
{
	uint8_t reg = 0;
	uint8_t bit = 0;
	uint8_t clrbit = 0;
	reg = (uint8_t)(GPIO_AF >> 8);		/*  ��ȡ�Ĵ�����  */
	bit = (uint8_t)(GPIO_AF);					/*  ��ȡ��Ҫ���õļĴ���λ��  */
	
	switch(GPIO_AF)				/*  ��Ϊ��ͬ�ĸ��ù���ռ�õ�λ����һ��  */
	{
		case GPIO_AF_FTM0_CH0:
		case GPIO_AF_FTM0_CH1:
		case GPIO_AF_FTM1_CH0:
		case GPIO_AF_FTM1_CH1:
		case GPIO_AF_PWT_IN0:
		case GPIO_AF_PWT_IN1:
		case GPIO_AF_FTM2_CH4:
		case GPIO_AF_FTM2_CH5:
		case GPIO_AF_RTCO:clrbit = 1;break;		/*  ֻռ��һλ  */
		case GPIO_AF_FTM2_CH0:
		case GPIO_AF_FTM2_CH1:
		case GPIO_AF_FTM2_CH2:
		case GPIO_AF_FTM2_CH3: clrbit = 3;break;	/*  ռ����λ  */
		case GPIO_AF_IRQ: clrbit = 7; break;			/*  ռ����λ  */
	}
	if(reg == 0)			/*  ��PINSEL0�Ĵ�������  */
	{
		SIM->PINSEL &= ~(clrbit << bit);		/*  ���������  */
		SIM->PINSEL |= GPIO_PinSource << bit;
	}
	else							/*  ��PINSEL1�Ĵ�������  */
	{
		SIM->PINSEL1 &= ~(clrbit << bit);		/*  ���������  */
		SIM->PINSEL1 |= GPIO_PinSource << bit;
	}
}


/*
*********************************************************************************************************
*                                    drv_gpio_PullCmd      
*
* Description: ʹ���ڲ���������
*             
* Arguments  : 1> GPIO_Pin: Ҫʹ�ܵ�����
*              2> NewState: ʹ�ܻ��߽���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_gpio_PullCmd(uint8_t GPIO_Pin, FunctionalState NewState)
{
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  �����GPIO�˿�  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  �����GPIO����  */
	
	if(NewState != DISABLE)
	{
		if(GPIOX[gpiox] == GPIOA)
			PORT->PUE0 |= (uint32_t)1<<pinx;
		else if(GPIOX[gpiox] == GPIOB)
			PORT->PUE1 |= (uint32_t)1<<pinx;
		else if(GPIOX[gpiox] == GPIOC)
			PORT->PUE2 |= (uint32_t)1<<pinx;
	}
	else
	{
		if(GPIOX[gpiox] == GPIOA)
			PORT->PUE0 &= ~((uint32_t)1<<GPIO_Pin);
		else if(GPIOX[gpiox] == GPIOB)
			PORT->PUE1 &= ~((uint32_t)1<<GPIO_Pin);
		else if(GPIOX[gpiox] == GPIOC)
			PORT->PUE2 &= ~((uint32_t)1<<GPIO_Pin);
	}
}

/********************************************  END OF FILE  *******************************************/

