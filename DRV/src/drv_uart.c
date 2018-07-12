/**
  *******************************************************************************************************
  * File Name: drv_uart.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-1
  * Brief: KEA128оƬ���ڵײ���������
  *******************************************************************************************************
  * History
  *		1.Data: 2018-2-1
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


/*
*********************************************************************************************************
*                             drv_uart_Init             
*
* Description: ��ʼ��UART����
*             
* Arguments  : 1> UARTx: UART����ṹ��ָ��
*              2> UART_InitStruct: UART�����ʼ���ṹ��ָ��
*
* Reutrn     : ʵ�����õĲ�����
*
* Note(s)    : ����KEA128û�в�����΢����,��������õĲ����ʽϸߵ�ʱ��׼ȷ�ʻ��½�,����ʵ�ʲ�������У��
*********************************************************************************************************
*/
uint32_t drv_uart_Init(UART_Type *UARTx, UART_InitTypeDef *UART_InitStruct)
{
	volatile uint32_t uart_clk;
	uint16_t sbr;
	
	/*  ��ʱ��  */
	if(UARTx == UART0) drv_rcc_ClockCmd(RCC_PeriphClock_UART0, ENABLE);
	else if(UARTx == UART1) drv_rcc_ClockCmd(RCC_PeriphClock_UART1, ENABLE);
	else if(UARTx == UART2) drv_rcc_ClockCmd(RCC_PeriphClock_UART2, ENABLE);
	
	drv_uart_SelectChannle(UARTx, UART_InitStruct->UART_Channel);		/*  ѡ��ͨ��  */
	
	UARTx->C2 &= ~(0 | UART_C2_TE_MASK | UART_C2_RE_MASK);	/*  ����ǰ�ȹرշ��ͺͽ���  */
	
	UARTx->C1 |= UART_InitStruct->UART_Parity;			/*  ������żУ��  */
	UARTx->C1 |= UART_InitStruct->UART_WordLength;	/*  �������ݳ���  */
	UARTx->C1 |= UART_InitStruct->UART_StopBits;		/*  ����ֹͣλ  */
	
	//UART������ = UARTģ��ʱ��/(16 X (SBR[12:0]))
	uart_clk = SystemBusClock;
	sbr = (uint16_t)(((uart_clk / 16) + (UART_InitStruct->UART_BaudRate / 2)) / UART_InitStruct->UART_BaudRate);

	if(sbr > 0x1fff) sbr = 0x1fff;
	
	UARTx->BDH &= ~UART_BDH_SBR_MASK;				/*  ��ղ����ʼĴ���  */
	UARTx->BDH |= UART_BDH_SBR(sbr>>8);     /*  ��д��SBR��λ  */
  UARTx->BDL  = UART_BDL_SBR(sbr);  /*  д���λ  */
	
	UARTx->C2 |= UART_InitStruct->UART_Mode;		/*  ���ô���ģʽ  */
	
	return ((uart_clk>>4)/sbr); 				/*  ����ʵ�ʲ�����  */
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
void drv_uart_DeInit(UART_Type *UARTx)
{
	
}

/*
*********************************************************************************************************
*                                  drv_uart_Cmd        
*
* Description: �򿪻�رմ���ʱ��
*             
* Arguments  : 1> UARTx: Ҫ�򿪵Ĵ���,���ڽṹ��ָ��
*              2> NewState: ״̬,ENABLE����DISABLE
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_uart_Cmd(UART_Type *UARTx, FunctionalState NewState)
{
	if(UARTx == UART0) drv_rcc_ClockCmd(RCC_PeriphClock_UART0, NewState);
	else if(UARTx == UART1) drv_rcc_ClockCmd(RCC_PeriphClock_UART1, NewState);
	else if(UARTx == UART2) drv_rcc_ClockCmd(RCC_PeriphClock_UART2, NewState);
}

/*
*********************************************************************************************************
*                                drv_uart_SelectChannle          
*
* Description: ���ô���ͨ��
*             
* Arguments  : 1> UARTx: UARTָ��
*              2> Channel: ͨ��,ֻ����UART_Channel_0����UART_Channel_0
*
* Reutrn     : None.
*
* Note(s)    : Channelֻ��Ϊ0����1
*********************************************************************************************************
*/
void drv_uart_SelectChannle(UART_Type *UARTx, uint8_t Channel)
{
	if(Channel > 1) return;  /*  ֻ������ͨ��  */
	
	if(UART0 == UARTx)
	{
		SIM->PINSEL |= Channel << 7;
		if(Channel == 1) 
		{
			PORT->PUE0 |= 1 << 2 | 1 << 3;
		}
	}
	else if(UART1 == UARTx)
	{
		SIM->PINSEL1 |= Channel << 12;
	}
	else if(UART2 == UARTx)
	{
		SIM->PINSEL1 |= Channel << 13;
	}
}

/*
*********************************************************************************************************
*                                 drv_uart_StructInit         
*
* Description: Ĭ�����ô��ڳ�ʼ���ṹ���ֵ
*             
* Arguments  : 1> UART_InitStruct: ���ڳ�ʼ���ṹ��ָ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_uart_StructInit(UART_InitTypeDef *UART_InitStruct)
{
	UART_InitStruct->UART_BaudRate = 9600;
	UART_InitStruct->UART_Channel = UART_Channel_0;
	UART_InitStruct->UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
	UART_InitStruct->UART_Parity = UART_Parity_No;
	UART_InitStruct->UART_StopBits = UART_StopBits_1;
	UART_InitStruct->UART_WordLength = UART_WordLength_8b;
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
void drv_uart_ITConfig(UART_Type *UARTx, uint16_t UART_IT, FunctionalState NewState)
{
}

/*
*********************************************************************************************************
*                                 drv_uart_GetITStatus         
*
* Description: ��ȡ�Ĵ����ж�״̬��־λ���
*             
* Arguments  : 1> UARTx: ���ڽṹ��ָ��
*              2> UART_IT: Ҫ��ȡ���жϱ�־λ
*
* Reutrn     : 1> SET: ��־λ�Ѿ�����λ
*              2> RESET: ��־λû�б���λ
*
* Note(s)    : None.
*********************************************************************************************************
*/
FlagStatus drv_uart_GetITStatus(UART_Type *UARTx, uint16_t UART_IT)
{
	uint8_t uartreg = 0x00;
	uint8_t bitstatus = 0x00;
	FlagStatus status = RESET;
	
	uartreg = (uint8_t)(UART_IT >> 8);		/*  ��ȡ�Ĵ������  */
	
	if(uartreg == 0x04)  /*  ״̬�Ĵ���SR1  */
	{
		bitstatus = (uint8_t)(UARTx->S1 & (uint8_t)UART_IT);
	}
	else if(uartreg == 0x05)		/*  ״̬�Ĵ���SR2  */
	{
		bitstatus = (uint8_t)(UARTx->S2 & (uint8_t)UART_IT);
	}
	
	if(bitstatus != 0x00) status = SET;  /*  bitstatus��ֵ��Ϊ0�Ļ�˵���ñ�־�Ѿ���λ  */
	else status = RESET;
	return status;
}

/*
*********************************************************************************************************
*                              drv_uart_SendData            
*
* Description: ͨ�����ڷ���һ������
*             
* Arguments  : 1> UARTx: ���ڽṹ��ָ��
*              2> data: Ҫ���͵�����
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_uart_SendData(UART_Type *UARTx, uint8_t data)
{
	UARTx->D = (data & 0xff); /*  д�����ݼĴ���  */
}

/*
*********************************************************************************************************
*                              drv_uart_ReceiveData            
*
* Description: ͨ�����ڽ�������
*             
* Arguments  : 1> UARTx: ���ڽṹ��ָ��
*
* Reutrn     : ���յ�������
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_uart_ReceiveData(UART_Type *UARTx)
{
	return UARTx->D;		/*  д���ݼĴ���  */
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
//void UART0_IRQHandler(void)
//{
//	
//}


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
void UART1_IRQHandler(void)
{
	
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
void UART2_IRQHandler(void)
{
	
}


/********************************************  END OF FILE  *******************************************/
