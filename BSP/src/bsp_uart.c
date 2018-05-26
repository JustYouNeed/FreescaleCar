/**
  *******************************************************************************************************
  * File Name: bsp_uart.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-11
  * Brief: 该文件中定义了串口使用函数,同时对串口进行了一定程度的封装,数据和发送与接收都采用中断方式,可以
	*        提高串口的数据发送与接收的及时性
  *******************************************************************************************************
  * History
	*		1.Author: Vector
	*			Data: 2018-2-11
  *			Mod: 建立本文件,发现存在BUG,暂未解决
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
/*  定义串口控制结构体以及接收、发送数据缓存区  */
Uart_Str uart_info;
static uint8_t UartTxBuff[UART_TX_BUFF_SIZE];
static uint8_t UartRxBuff[UART_TX_BUFF_SIZE];


/*
*********************************************************************************************************
*                                          bsp_uart_ParaInit
*
* Description: 初始化串口控制结构体以及接收、发送数据缓存区
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数为本文件私有函数,不得由外部调用
*********************************************************************************************************
*/
void bsp_uart_ParaInit(void)
{
	uart_info.uart = UART0;
	uart_info.pRxBuff = UartRxBuff;
	uart_info.pTxBuff = UartTxBuff;
	uart_info.RxBuffSize = UART_RX_BUFF_SIZE;
	uart_info.TxBuffSize = UART_TX_BUFF_SIZE;
	uart_info.RxCount = 0;
	uart_info.RxRead = 0;
	uart_info.RxWrite = 0;
	uart_info.TxCount = 0;
	uart_info.TxRead = 0;
	uart_info._cbRecvData = 0;
	uart_info._cbSendBefor = 0;
	uart_info._cbSendOver = 0;
}


/*
*********************************************************************************************************
*                                       bsp_uart_Config   
*
* Description: 初始化所有定义使用的串口
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 串口波特率默认使用9600,如果需要更改,请在bsp_uart.h文件中更改宏: UART_BAUD
*********************************************************************************************************
*/
void bsp_uart_Config(void)
{
	UART_InitTypeDef UART_InitStruct;
	
//	drv_gpio_PullCmd(GPIO_Pin_A2, ENABLE);
//	drv_gpio_PullCmd(GPIO_Pin_A3, ENABLE);
	
	bsp_uart_ParaInit();		/*  结构体初始化  */
	
	UART_InitStruct.UART_BaudRate = UART_BAUD;		/*  波特率9600  */
	UART_InitStruct.UART_Channel = UART_Channel_1;	/*  通道1  */
	UART_InitStruct.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	/*  接收与发送,全双工模式  */
	UART_InitStruct.UART_Parity = UART_Parity_No;	/*  无检验  */
	UART_InitStruct.UART_StopBits = UART_StopBits_1;	/*  一位停止位  */
	UART_InitStruct.UART_WordLength = UART_WordLength_8b;	/*  8bit  */
	drv_uart_Init(UART0, &UART_InitStruct);
	
	uart_info.uart->C2 |= 1 << 5;		/*  开启接收中断  */
//	uart_info.uart->C2 |= 1 << 4;
//	uart_info.uart->C2 |= 1 << 6;
//	uart_info.uart->C2 |= 1 << 7;
	
	NVIC_EnableIRQ(UART0_IRQn);
		
	drv_uart_Cmd(UART0, ENABLE);		/*  使能串口  */
}


/*
*********************************************************************************************************
*                                      bsp_uart_GetUartNo    
*
* Description: 由串口编号获取串口控制结构体
*             
* Arguments  : 1> Port: 串口编号, COM0, COM1, COM3
*
* Reutrn     : 1> 0: 串口编号错误
*              2> Uart_Str结构体指针: 成功
*
* Note(s)    : None.
*********************************************************************************************************
*/
Uart_Str *bsp_uart_GetUartNo(COM_PORT_ENUM Port)
{
	switch(Port)
	{
		case COM0:return &uart_info;
		default:return 0;
	}
}

/*
*********************************************************************************************************
*                                     bsp_uart_Put     
*
* Description: 推送一个byte的数据到数据接收缓存区
*             
* Arguments  : 1> pUart: 串口控制结构体
*              2> byte: 要推送的数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_Put(Uart_Str *pUart,uint8_t byte)
{
	pUart->pRxBuff[pUart->RxWrite] = byte;	/*  将数据送入接收缓存区  */
	if(++ pUart->RxWrite >= pUart->RxBuffSize)	/*  环形队列  */
		pUart->RxWrite = 0;
	if(pUart->RxCount < pUart->RxBuffSize)
		pUart->RxCount ++;
}

/*
*********************************************************************************************************
*                                     bsp_uart_ClearTxBuff     
*
* Description: 清空串口发送缓存区
*             
* Arguments  : 1> Port: 串口编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_ClearTxBuff(COM_PORT_ENUM Port)
{
	Uart_Str *pUart = 0;
	pUart = bsp_uart_GetUartNo(Port);		/*  获取串口控制结构体  */
	
	if(pUart == 0) return ;
	pUart->TxCount = 0;
	pUart->TxRead = 0;
	pUart->TxWrite = 0;
}

/*
*********************************************************************************************************
*                                     bsp_uart_ClearRxBuff     
*
* Description: 清空口接收缓存区
*             
* Arguments  : 1> Port: 串口编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_ClearRxBuff(COM_PORT_ENUM Port)
{
	Uart_Str *pUart = 0;
	pUart = bsp_uart_GetUartNo(Port);		/*  获取串口控制结构体  */
	
	if(pUart == 0) return ;
	pUart->RxCount = 0;
	pUart->RxRead = 0;
	pUart->RxWrite = 0;
}

/*
*********************************************************************************************************
*                                          bsp_uart_SendDataToBuff
*
* Description: 推送数据到发送缓存区,等待发送
*             
* Arguments  : 1> Port: 串口编号
*              2> buff: 要发送的数据地址
*              3> length: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_SendDataToBuff(COM_PORT_ENUM Port, uint8_t *buff, uint16_t length)
{
	Uart_Str *pUart = 0;
	uint16_t i = 0;
	pUart = bsp_uart_GetUartNo(Port);		/*  获取串口控制结构体  */
	if(pUart == 0) return ;	/*  获取失败  */
	
	for (i = 0; i < length; i++)
	{
		/* 如果发送缓存区不为空的话，等待发送缓存区空闲 */
		while(1)
		{
			__IO uint16_t Count;
			
//			DISABLE_INT();
			Count = pUart->TxCount;
//			ENABLE_INT();
			
			if(Count < pUart->TxBuffSize) break;
		}
		
		pUart->pTxBuff[pUart->TxWrite] = buff[i];		/*  拷贝数据  */
		
	//	DISABLE_INT();
		if(++ pUart->TxWrite >= pUart->TxBuffSize)	/*  环形队列  */
		{
			pUart->TxWrite = 0;
		}
		pUart->TxCount ++;		/*  需要发送的数据计数器  */
//		ENABLE_INT();
	}
	pUart->uart->C2 |= 1 << 7; /*  打开TXE中断  */
}


/*
*********************************************************************************************************
*                                         bsp_uart_SendDataToBuff 
*
* Description: 从接收缓存区里读取一个字节的数据
*             
* Arguments  : 1> Port: 串口编号
*              2> byte: 数据指针,存储接收到的数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_uart_GetChar(COM_PORT_ENUM Port, uint8_t *byte)
{
	uint16_t Count;
	Uart_Str *pUart = 0;
	
	pUart = bsp_uart_GetUartNo(Port);  /*  获取串口控制结构体  */
	if(pUart == 0) return 1;	/*  获取失败  */
	Count = pUart->RxCount;

	if(Count == 0)	return 1;  /*  没有数据需要读取  */
	else
	{
		*byte = pUart->pRxBuff[pUart->RxRead];	/*  读取数据  */

		if(++ pUart->RxRead >= pUart->RxBuffSize)	/*  环形队列  */
			pUart->RxRead = 0;
		
		pUart->RxCount -- ;		/*  数据已经被读取走了  */
	}
	return 0;
}

/*
*********************************************************************************************************
*                                     bsp_uart_IRQHandler     
*
* Description: 串口中断执行函数,所有的串口中断发生时都调用此函数
*             
* Arguments  : 1> pUart: 串口控制结构体指针
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_IRQHandler(Uart_Str *pUart)
{
	uint8_t RecvData = 0;
	uint8_t reg = 0x0;
	
	
//	if(pUart->uart->S1 & UART_S1_RDRF_MASK)  /*  接收数据寄存器满  */
//	{
////		reg = (uint8_t)pUart->uart->S1;
////		pUart->uart->S2 |= 1 << 6;
//		RecvData = pUart->uart->D;		/*  读取数据并送入接收缓存区  */
//		bsp_uart_Put(pUart, RecvData);
//	}
	
	if(pUart->uart->S1 & UART_S1_OR_MASK)
	{
		reg = (uint8_t)pUart->uart->S1;
		RecvData = pUart->uart->D;
	}
	reg = (uint8_t)pUart->uart->S1;
	if(pUart->uart->S1 & UART_S1_TDRE_MASK)	/*  发送缓存区为空,说明可以发送数据了  */
	{
		
		if(pUart->TxCount == 0)  /*  没有数据需要发送  */
		{
			pUart->uart->C2 &= ~(1 << 7); /*  关闭TXE中断  */
//			pUart->uart->C2 |= 1 << 6;    /*  打开发送完成中断  */
			
		}
		else
		{
			pUart->uart->D = (uint8_t)pUart->pTxBuff[pUart->TxRead]; /*  将数据写入发送缓存区  */ 
			if(++ pUart->TxRead >= pUart->TxBuffSize)  /*  环形队列  */
			{
				pUart->TxRead = 0;
			}
			pUart->TxCount --;		/*  需要发送的数据递减  */
		}
	}
//	else if((pUart->uart->S1 & 0x40) != RESET)  /*  发送完成中断  */
//	{
//		
//		if(pUart->TxCount == 0)  /*  没有数据需要发送  */
//		{
//			pUart->uart->C2 &= ~(1 << 6);  /*  关闭发送完成中断  */
//			pUart->uart->C2 &= ~(1 << 3);
//			pUart->uart->C2 |= (1 << 3);
//			if(pUart->_cbSendOver)		/*  如果设置了发送完成回调函数  */
//				pUart->_cbSendOver(0);
//		}
//		else			/*  还有数据需要发送  */
//		{
//			pUart->uart->D = pUart->pTxBuff[pUart->TxRead]; /*  将数据写入发送缓存区  */ 
//			if(++ pUart->TxRead >= pUart->TxBuffSize)  /*  环形队列  */
//			{
//				pUart->TxRead = 0;
//			}
//			pUart->TxCount --;		/*  需要发送的数据递减  */
//		}
//	}
}


/*
*********************************************************************************************************
*                                   bsp_uart_Printf       
*
* Description: 自定义串口打印函数
*             
* Arguments  : 1> format: 输出格式控制字符串
*              2> ...: 长度不定形参,需要打印的数据
*
* Reutrn     : None.
*
* Note(s)    : 默认调用串口0,需要根据情况更改
*********************************************************************************************************
*/
uint8_t bsp_uart_Printf(const char *format, ...)
{
	uint16_t i = 0;
	char buf[256];
	
	va_list arg;
	va_start(arg ,format);
	
	i = vsprintf(buf, format, arg);
	
	bsp_uart_SendDataToBuff(COM0, (uint8_t *)buf, i);
	
	va_end(arg);
	return i;
}


/*
*********************************************************************************************************
*                                     UART0_IRQHandler     
*
* Description: 串口0中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
//void UART0_IRQHandler(void)
//{
//	(void)UART0_S1;
//	bsp_uart_IRQHandler(&uart_info);
//}


/*  加入以下代码,支持printf, 而不需要选择use MicroLIB  */
#if 1
#pragma import(__use_no_semihosting)             
 
struct __FILE /*  标准库需要的支持函数  */
{ 
	int handle; 
}; 

FILE __stdout;       
   
/*  定义_sys_exit() 避免使用半主机模式  */
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 

/*  重定义fputc函数,支持printf  */
int fputc(int ch, FILE *f)
{ 	
	while((UART0->S1&0X80)==0);//循环发送,直到发送完毕   
	UART0->D = (uint8_t) ch;      
	return ch;
}
#endif


/********************************************  END OF FILE  *******************************************/

