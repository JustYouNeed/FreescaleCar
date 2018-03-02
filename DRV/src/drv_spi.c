/**
  *******************************************************************************************************
  * File Name: drv_spi.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-27
  * Brief: ���ļ��������й�SPI�����ĵײ���������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-2-27
	*			Mod:	�������ļ�,��ӻ�������
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
* Description: ����SPI��ʼ���ṹ���ֵ��ʼ����ؼĴ���
*             
* Arguments  : 1> SPIx: Ҫ��ʼ����SPI, ȡֵ��Χ: SPI0, SPI1
*							 2> SPI_InitStruct: SPI��ʼ���ṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_spi_Init(SPI_Type *SPIx, SPI_InitTypeDef *SPI_InitStruct)
{
	drv_spi_Cmd(SPIx, DISABLE);	/*  ����ǰ�ȹر�SPI  */
	
	if(SPIx == SPI0) drv_rcc_ClockCmd(RCC_PeriphClock_SPI0, ENABLE);
	else if(SPIx == SPI1)drv_rcc_ClockCmd(RCC_PeriphClock_SPI1, ENABLE);
	
	/*  ����SPIͨ��  */
	drv_spi_SetChannel(SPIx, SPI_InitStruct->SPI_Channel);
	
	/*  ����LSB  */
	SPIx->C1 &= ~(SPI_C1_LSBFE_MASK);
	SPIx->C1 |= SPI_InitStruct->SPI_FirstBit << SPI_C1_LSBFE_SHIFT;
	
	/*  ����SPI����ģʽ,�������ߴӻ�  */
	SPIx->C1 &= ~(1 << SPI_C1_MSTR_SHIFT);
	SPIx->C1 |= SPI_InitStruct->SPI_Mode << SPI_C1_MSTR_SHIFT;
	
	/*  ����SPIʱ����λ  */
	SPIx->C1 &= ~(1 << SPI_C1_CPHA_SHIFT);
	SPIx->C1 |= SPI_InitStruct->SPI_CPHA << SPI_C1_CPHA_SHIFT;
	
	/*  ����SPIʱ�Ӽ���  */
	SPIx->C1 &= ~(1 << SPI_C1_CPOL_SHIFT);
	SPIx->C1 |= (SPI_InitStruct->SPI_CPOL << SPI_C1_CPOL_SHIFT);
	
	/*  ����ʱ�ӷ�Ƶ����  */
	SPIx->BR &= ~(SPI_BR_SPR_MASK);
	SPIx->BR |= SPI_InitStruct->SPI_BaudRateDivider << SPI_BR_SPR_SHIFT;
	
	/*  ����ʱ��Ԥ��Ƶ����  */
	SPIx->BR &= ~(SPI_BR_SPPR_MASK);
	SPIx->BR |= (SPI_InitStruct->SPI_BaudRatePrescaler << SPI_BR_SPPR_SHIFT);
	
	/*  �����Ƿ�ʹ��Ӳ��Ƭѡ  */
	SPIx->C2 &= ~(SPI_C2_MODFEN_MASK);
	SPIx->C2 |= SPI_InitStruct->SPI_NSS << SPI_C2_MODFEN_SHIFT;
	SPIx->C1 &= ~(SPI_C1_SSOE_MASK);
	SPIx->C1 |= SPI_InitStruct->SPI_NSS << SPI_C1_SSOE_SHIFT;
	
	/*  ����ȫ˫�����ߵ���ģʽ  */
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
	
	drv_spi_Cmd(SPIx, ENABLE);	/*  ������Ͽ���SPI  */
}

/*
*********************************************************************************************************
*                              drv_spi_ReadWriteByte            
*
* Description: ͨ��SPI��дһ���ֽ�����
*             
* Arguments  : 1> SPIx: Ҫ�������ݵ�SPI, ȡֵ��Χ: SPI0, SPI1
*							 2> byte: Ҫ���͵�����
*
* Reutrn     : 1> 0xff: ���������¸�ֵ��������ʱ
*							 2> other: ���յ�������
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_spi_ReadWriteByte(SPI_Type *SPIx, uint8_t byte)
{
	uint8_t recv = 0;
	uint16_t timeout = 0;
	while(!(SPIx->S & SPI_S_SPTEF_MASK))  	/*  �ȴ�SPI���ͻ�����Ϊ��  */
	{
		timeout ++;
		if(timeout > SPI_RECV_TIMEOUT) return 0xff;
	}
	(void)SPIx->S;	/*  ��ȡһ�μĴ��������־  */
	SPIx->D = (uint8_t)byte;
	
	while(!(SPIx->S & SPI_S_SPMF_MASK))  /*  �ȴ����յ�����  */
	{
		timeout ++;
		if(timeout > SPI_RECV_TIMEOUT) return 0xff;
	}
	(void)SPIx->S;  /*  ��ȡһ�μĴ��������־  */
	recv = (uint8_t)SPIx->D;
	
	return recv;	/*  ���ؽ��յ�������  */
}


/*
*********************************************************************************************************
*                           drv_spi_SetChannel               
*
* Description: ����SPIͨ��,����ͨ�����Ŷ�����drv_spi.h������
*             
* Arguments  : 1> SPIx: Ҫ����ͨ����SPI
*							 2> SPI_Channel_x: SPIͨ��
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
		SIM->PINSEL &= ~SIM_PINSEL_SPI0PS_MASK;	/*  �������λ  */
		SIM->PINSEL |= SPI_Channel_x << SIM_PINSEL_SPI0PS_SHIFT;
	}
	else if(SPIx == SPI1)
	{
		SIM->PINSEL1 &= ~SIM_PINSEL1_SPI1PS_MASK; /*  �������λ  */
		SIM->PINSEL1 |= SPI_Channel_x << SIM_PINSEL_SPI0PS_SHIFT;
	}
}

/*
*********************************************************************************************************
*                         drv_spi_Cmd                 
*
* Description: SPIʹ�ܺ���
*             
* Arguments  : 1> SPIx: Ҫ�������߹رյ�SPI,ȡֵ��Χ: SPI0, SPI1
*							 2> NewState: ״ֵ̬, ENABLE����DISABLE
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
