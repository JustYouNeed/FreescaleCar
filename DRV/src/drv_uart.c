/**
  *******************************************************************************************************
  * File Name: drv_uart.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-1
  * Brief: KEA128芯片串口底层驱动函数
  *******************************************************************************************************
  * History
  *		1.Data: 2018-2-1
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
# include "drv_uart.h"


/*
*********************************************************************************************************
*                             drv_uart_Init             
*
* Description: 初始化UART外设
*             
* Arguments  : 1> UARTx: UART外设结构体指针
*              2> UART_InitStruct: UART外设初始化结构体指针
*
* Reutrn     : 实际设置的波特率
*
* Note(s)    : 由于KEA128没有波特率微调器,因此在设置的波特率较高的时候准确率会下降,返回实际波特率来校验
*********************************************************************************************************
*/
uint32_t drv_uart_Init(UART_Type *UARTx, UART_InitTypeDef *UART_InitStruct)
{
	volatile uint32_t uart_clk;
	uint16_t sbr;
	
	/*  打开时钟  */
	if(UARTx == UART0) drv_rcc_ClockCmd(RCC_PeriphClock_UART0, ENABLE);
	else if(UARTx == UART1) drv_rcc_ClockCmd(RCC_PeriphClock_UART1, ENABLE);
	else if(UARTx == UART2) drv_rcc_ClockCmd(RCC_PeriphClock_UART2, ENABLE);
	
	drv_uart_SelectChannle(UARTx, UART_InitStruct->UART_Channel);		/*  选择通道  */
	
	UARTx->C2 &= ~(0 | UART_C2_TE_MASK | UART_C2_RE_MASK);	/*  配置前先关闭发送和接收  */
	
	UARTx->C1 |= UART_InitStruct->UART_Parity;			/*  设置奇偶校验  */
	UARTx->C1 |= UART_InitStruct->UART_WordLength;	/*  设置数据长度  */
	UARTx->C1 |= UART_InitStruct->UART_StopBits;		/*  设置停止位  */
	
	//UART波特率 = UART模块时钟/(16 X (SBR[12:0]))
	uart_clk = SystemBusClock;
	sbr = (uint16_t)(((uart_clk / 16) + (UART_InitStruct->UART_BaudRate / 2)) / UART_InitStruct->UART_BaudRate);

	if(sbr > 0x1fff) sbr = 0x1fff;
	
	UARTx->BDH &= ~UART_BDH_SBR_MASK;				/*  清空波特率寄存器  */
	UARTx->BDH |= UART_BDH_SBR(sbr>>8);     /*  先写入SBR高位  */
  UARTx->BDL  = UART_BDL_SBR(sbr);  /*  写入低位  */
	
	UARTx->C2 |= UART_InitStruct->UART_Mode;		/*  设置串口模式  */
	
	return ((uart_clk>>4)/sbr); 				/*  返回实际波特率  */
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
* Description: 打开或关闭串口时钟
*             
* Arguments  : 1> UARTx: 要打开的串口,串口结构体指针
*              2> NewState: 状态,ENABLE或者DISABLE
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
* Description: 设置串口通道
*             
* Arguments  : 1> UARTx: UART指针
*              2> Channel: 通道,只能是UART_Channel_0或者UART_Channel_0
*
* Reutrn     : None.
*
* Note(s)    : Channel只能为0或者1
*********************************************************************************************************
*/
void drv_uart_SelectChannle(UART_Type *UARTx, uint8_t Channel)
{
	if(Channel > 1) return;  /*  只有两个通道  */
	
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
* Description: 默认设置串口初始化结构体的值
*             
* Arguments  : 1> UART_InitStruct: 串口初始化结构体指针
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
* Description: 获取寄存器中断状态标志位情况
*             
* Arguments  : 1> UARTx: 串口结构体指针
*              2> UART_IT: 要获取的中断标志位
*
* Reutrn     : 1> SET: 标志位已经被置位
*              2> RESET: 标志位没有被置位
*
* Note(s)    : None.
*********************************************************************************************************
*/
FlagStatus drv_uart_GetITStatus(UART_Type *UARTx, uint16_t UART_IT)
{
	uint8_t uartreg = 0x00;
	uint8_t bitstatus = 0x00;
	FlagStatus status = RESET;
	
	uartreg = (uint8_t)(UART_IT >> 8);		/*  获取寄存器编号  */
	
	if(uartreg == 0x04)  /*  状态寄存器SR1  */
	{
		bitstatus = (uint8_t)(UARTx->S1 & (uint8_t)UART_IT);
	}
	else if(uartreg == 0x05)		/*  状态寄存器SR2  */
	{
		bitstatus = (uint8_t)(UARTx->S2 & (uint8_t)UART_IT);
	}
	
	if(bitstatus != 0x00) status = SET;  /*  bitstatus的值不为0的话说明该标志已经置位  */
	else status = RESET;
	return status;
}

/*
*********************************************************************************************************
*                              drv_uart_SendData            
*
* Description: 通过串口发送一个数据
*             
* Arguments  : 1> UARTx: 串口结构体指针
*              2> data: 要发送的数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_uart_SendData(UART_Type *UARTx, uint8_t data)
{
	UARTx->D = (data & 0xff); /*  写入数据寄存器  */
}

/*
*********************************************************************************************************
*                              drv_uart_ReceiveData            
*
* Description: 通过串口接收数据
*             
* Arguments  : 1> UARTx: 串口结构体指针
*
* Reutrn     : 接收到的数据
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_uart_ReceiveData(UART_Type *UARTx)
{
	return UARTx->D;		/*  写数据寄存器  */
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
