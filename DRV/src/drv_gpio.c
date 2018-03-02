/**
  *******************************************************************************************************
  * File Name: drv_pit.c
  * Author: Vector
  * Version: V2.2.0
  * Date: 2018-2-1
  * Brief: KEA128芯片GPIO底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *     Author: Vector
  *     Mod: 建立文件,添加基本函数
	*		
	*		2.Date: 2018-2-9
	*			Author:	Vector
	*			Mod: 添加新函数drv_gpio_PinAFConfig,用于设置引脚的复用功能
	*
	*		3.Date:2018-2-28
	*			Author: Vector
	*			Mod: 1.添加新函数drv_gpio_PullCmd,用于设置引脚的上拉电阻
	*					 2.修改引脚初始化逻辑,添加GPIO端口数组GPIOX[],
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
* Description: 初始化一个GPIO引脚
*             
* Arguments  : 1> GPIO_InitStruct: GPIO引脚配置结构体指针
*
* Reutrn     : None.
*
* Note(s)    : 一次只能配置一个引脚
*********************************************************************************************************
*/
void drv_gpio_Init(GPIO_InitTypeDef *GPIO_InitStruct)
{
	uint32_t reg = 0;
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_InitStruct->GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  计算出GPIO端口  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  计算出GPIO引脚  */
	
	GPIOX[gpiox]->PIDR |= ((uint32_t)1 << pinx);	/*  先禁止输入  */
	GPIOX[gpiox]->PDDR &= ~((uint32_t)1 << pinx);
	
	/*  根据初始化结构体的值设置为相应的输入/输出  */
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
	
	/*  如果开启了上拉  */
	if(GPIO_InitStruct->GPIO_PuPd == GPIO_PuPd_UP)
	{
		/*  GPIOA,GPIOB,及其他配置的寄存器不同  */
		if(GPIOA == GPIOX[gpiox]) PORT->PUE0 |= 1 << pinx;
		else if(GPIOB == GPIOX[gpiox]) PORT->PUE1 |= 1 << pinx;
		else	if(GPIOC == GPIOX[gpiox])PORT->PUE2 |= 1 << pinx;
	}
	
	/*  大电流驱动能力只有固定的几个引脚有  */
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
* Description: 读取一个引脚的电平值
*             
* Arguments  : 1> GPIO_Pin: 引脚编号
*
* Reutrn     : 引脚电平
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_gpio_ReadPin(uint8_t GPIO_Pin)
{
	uint32_t pin;
	
	uint8_t gpiox = 0, pinx = 0;
	uint8_t pintemp = GPIO_Pin;
	
	gpiox = (uint8_t)(pintemp>>5);		/*  计算出GPIO端口  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  计算出GPIO引脚  */
	
	/*  读取引脚电平  */
	pin = (uint32_t)GPIOX[gpiox]->PDIR;
	
	return ((pin >> pinx) & 0x01);
}

/*
*********************************************************************************************************
*                                drv_gpio_WritePin          
*
* Description: 写引脚电平
*             
* Arguments  : 1> GPIO_Pin: 引脚编号
*              2> NewState: 枚举变量,SET 或者 RESET
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
	
	gpiox = (uint8_t)(pintemp>>5);		/*  计算出GPIO端口  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  计算出GPIO引脚  */
	
	if(PinState == GPIO_PIN_RESET)  /*  如果设置低电平  */
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
* Description: 翻转引脚电平
*             
* Arguments  : 1> GPIO_Pin: 引脚编号
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
	
	gpiox = (uint8_t)(pintemp>>5);		/*  计算出GPIO端口  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  计算出GPIO引脚  */
	
	GPIOX[gpiox]->PTOR |= 1 << pinx;		/*  GPIOX[gpiox]->PTOR寄存器,跳变寄存器  */
}


/*
*********************************************************************************************************
*                                    drv_gpio_PinAFConfig      
*
* Description: 设置引脚的复用功能
*             
* Arguments  : 1> GPIO_PinSource: GPIO引脚资源,在drv_gpio.h中有所有引脚资源的定义
*              2> GPIO_AF       : 要复用的功能,在drv_gpio.h中有所有复用功能的定义
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
	reg = (uint8_t)(GPIO_AF >> 8);		/*  获取寄存器号  */
	bit = (uint8_t)(GPIO_AF);					/*  获取需要设置的寄存器位置  */
	
	switch(GPIO_AF)				/*  因为不同的复用功能占用的位数不一样  */
	{
		case GPIO_AF_FTM0_CH0:
		case GPIO_AF_FTM0_CH1:
		case GPIO_AF_FTM1_CH0:
		case GPIO_AF_FTM1_CH1:
		case GPIO_AF_PWT_IN0:
		case GPIO_AF_PWT_IN1:
		case GPIO_AF_FTM2_CH4:
		case GPIO_AF_FTM2_CH5:
		case GPIO_AF_RTCO:clrbit = 1;break;		/*  只占用一位  */
		case GPIO_AF_FTM2_CH0:
		case GPIO_AF_FTM2_CH1:
		case GPIO_AF_FTM2_CH2:
		case GPIO_AF_FTM2_CH3: clrbit = 3;break;	/*  占用两位  */
		case GPIO_AF_IRQ: clrbit = 7; break;			/*  占用三位  */
	}
	if(reg == 0)			/*  在PINSEL0寄存器里面  */
	{
		SIM->PINSEL &= ~(clrbit << bit);		/*  先清除设置  */
		SIM->PINSEL |= GPIO_PinSource << bit;
	}
	else							/*  在PINSEL1寄存器里面  */
	{
		SIM->PINSEL1 &= ~(clrbit << bit);		/*  先清除设置  */
		SIM->PINSEL1 |= GPIO_PinSource << bit;
	}
}


/*
*********************************************************************************************************
*                                    drv_gpio_PullCmd      
*
* Description: 使能内部上拉电阻
*             
* Arguments  : 1> GPIO_Pin: 要使能的引脚
*              2> NewState: 使能或者禁用
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
	
	gpiox = (uint8_t)(pintemp>>5);		/*  计算出GPIO端口  */
	pinx = (uint8_t)(pintemp & 0x1f);	/*  计算出GPIO引脚  */
	
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

