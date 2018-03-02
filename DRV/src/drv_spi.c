/**
  *******************************************************************************************************
  * File Name: drv_spi.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-27
  * Brief: 本文件定义了有关SPI操作的底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-2-27
	*			Mod:	建立本文件,添加基本函数
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_spi.h"


/*
*********************************************************************************************************
*                                 drv_spi_Init         
*
* Description: 根据SPI初始化结构体的值初始化相关寄存器
*             
* Arguments  : 1> SPIx: 要初始化的SPI, 取值范围: SPI0, SPI1
*							 2> SPI_InitStruct: SPI初始化结构体
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_spi_Init(SPI_Type *SPIx, SPI_InitTypeDef *SPI_InitStruct)
{
	drv_spi_Cmd(SPIx, DISABLE);	/*  配置前先关闭SPI  */
	
	if(SPIx == SPI0) drv_rcc_ClockCmd(RCC_PeriphClock_SPI0, ENABLE);
	else if(SPIx == SPI1)drv_rcc_ClockCmd(RCC_PeriphClock_SPI1, ENABLE);
	
	/*  设置SPI通道  */
	drv_spi_SetChannel(SPIx, SPI_InitStruct->SPI_Channel);
	
	/*  设置LSB  */
	SPIx->C1 &= ~(SPI_C1_LSBFE_MASK);
	SPIx->C1 |= SPI_InitStruct->SPI_FirstBit << SPI_C1_LSBFE_SHIFT;
	
	/*  设置SPI运行模式,主机或者从机  */
	SPIx->C1 &= ~(1 << SPI_C1_MSTR_SHIFT);
	SPIx->C1 |= SPI_InitStruct->SPI_Mode << SPI_C1_MSTR_SHIFT;
	
	/*  设置SPI时钟相位  */
	SPIx->C1 &= ~(1 << SPI_C1_CPHA_SHIFT);
	SPIx->C1 |= SPI_InitStruct->SPI_CPHA << SPI_C1_CPHA_SHIFT;
	
	/*  设置SPI时钟极性  */
	SPIx->C1 &= ~(1 << SPI_C1_CPOL_SHIFT);
	SPIx->C1 |= (SPI_InitStruct->SPI_CPOL << SPI_C1_CPOL_SHIFT);
	
	/*  设置时钟分频因子  */
	SPIx->BR &= ~(SPI_BR_SPR_MASK);
	SPIx->BR |= SPI_InitStruct->SPI_BaudRateDivider << SPI_BR_SPR_SHIFT;
	
	/*  设置时钟预分频因子  */
	SPIx->BR &= ~(SPI_BR_SPPR_MASK);
	SPIx->BR |= (SPI_InitStruct->SPI_BaudRatePrescaler << SPI_BR_SPPR_SHIFT);
	
	/*  设置是否使用硬件片选  */
	SPIx->C2 &= ~(SPI_C2_MODFEN_MASK);
	SPIx->C2 |= SPI_InitStruct->SPI_NSS << SPI_C2_MODFEN_SHIFT;
	SPIx->C1 &= ~(SPI_C1_SSOE_MASK);
	SPIx->C1 |= SPI_InitStruct->SPI_NSS << SPI_C1_SSOE_SHIFT;
	
	/*  设置全双工或者单线模式  */
	SPIx->C2 &= ~(SPI_C2_BIDIROE_MASK << SPI_C2_BIDIROE_SHIFT);
	switch(SPI_InitStruct->SPI_Direction)
	{	
		case SPI_Direction_2Lines_FullDuplex:
		{
			SPIx->C2 &= ~(SPI_C2_SPC0_MASK);
		}break;
		
		case SPI_Direction_2Lines_RxOnly:
		case SPI_Direction_1Line_Rx:
		case SPI_Direction_1Line_Tx:
		{
			SPIx->C2 |= SPI_C2_SPC0_MASK;
			SPIx->C2 &= ~(SPI_C2_BIDIROE_MASK);
		}break;
		default: break;
	}
	
	drv_spi_Cmd(SPIx, ENABLE);	/*  配置完毕开启SPI  */
}

/*
*********************************************************************************************************
*                              drv_spi_ReadWriteByte            
*
* Description: 通过SPI读写一个字节数据
*             
* Arguments  : 1> SPIx: 要接收数据的SPI, 取值范围: SPI0, SPI1
*							 2> byte: 要发送的数据
*
* Reutrn     : 1> 0xff: 大多数情况下该值代表函数超时
*							 2> other: 接收到的数据
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_spi_ReadWriteByte(SPI_Type *SPIx, uint8_t byte)
{
	uint8_t recv = 0;
	uint16_t timeout = 0;
	while(!(SPIx->S & SPI_S_SPTEF_MASK))  	/*  等待SPI发送缓存区为空  */
	{
		timeout ++;
		if(timeout > SPI_RECV_TIMEOUT) return 0xff;
	}
	(void)SPIx->S;	/*  读取一次寄存器清除标志  */
	SPIx->D = (uint8_t)byte;
	
	while(!(SPIx->S & SPI_S_SPMF_MASK))  /*  等待接收到数据  */
	{
		timeout ++;
		if(timeout > SPI_RECV_TIMEOUT) return 0xff;
	}
	(void)SPIx->S;  /*  读取一次寄存器清除标志  */
	recv = (uint8_t)SPIx->D;
	
	return recv;	/*  返回接收到的数据  */
}


/*
*********************************************************************************************************
*                           drv_spi_SetChannel               
*
* Description: 设置SPI通道,关于通道引脚定义在drv_spi.h中声明
*             
* Arguments  : 1> SPIx: 要设置通道的SPI
*							 2> SPI_Channel_x: SPI通道
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_spi_SetChannel(SPI_Type *SPIx, uint8_t SPI_Channel_x)
{
	if(SPI_Channel_x > SPI_Channel_1) return;
	
	if(SPIx == SPI0)
	{
		SIM->PINSEL &= ~SIM_PINSEL_SPI0PS_MASK;	/*  先清除该位  */
		SIM->PINSEL |= SPI_Channel_x << SIM_PINSEL_SPI0PS_SHIFT;
	}
	else if(SPIx == SPI1)
	{
		SIM->PINSEL1 &= ~SIM_PINSEL1_SPI1PS_MASK; /*  先清除该位  */
		SIM->PINSEL1 |= SPI_Channel_x << SIM_PINSEL_SPI0PS_SHIFT;
	}
}

/*
*********************************************************************************************************
*                         drv_spi_Cmd                 
*
* Description: SPI使能函数
*             
* Arguments  : 1> SPIx: 要开启或者关闭的SPI,取值范围: SPI0, SPI1
*							 2> NewState: 状态值, ENABLE或者DISABLE
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_spi_Cmd(SPI_Type *SPIx, FunctionalState NewState)
{
	if(NewState != DISABLE)
	{
		SPIx->C1 |= SPI_C1_SPTIE_MASK;
	}
	else
	{
		SPIx->C1 &= ~SPI_C1_SPTIE_MASK;
	}
}


/********************************************  END OF FILE  *******************************************/
