/**
  *******************************************************************************************************
  * File Name: bsp_uart.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-11
  * Brief: ���ļ��ж����˴���ʹ�ú���,ͬʱ�Դ��ڽ�����һ���̶ȵķ�װ,���ݺͷ�������ն������жϷ�ʽ,����
	*        ��ߴ��ڵ����ݷ�������յļ�ʱ��
  *******************************************************************************************************
  * History
	*		1.Author: Vector
	*			Data: 2018-2-11
  *			Mod: �������ļ�,���ִ���BUG,��δ���
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
/*  ���崮�ڿ��ƽṹ���Լ����ա��������ݻ�����  */
Uart_Str uart_info;
static uint8_t UartTxBuff[UART_TX_BUFF_SIZE];
static uint8_t UartRxBuff[UART_TX_BUFF_SIZE];


/*
*********************************************************************************************************
*                                          bsp_uart_ParaInit
*
* Description: ��ʼ�����ڿ��ƽṹ���Լ����ա��������ݻ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ�˽�к���,�������ⲿ����
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
* Description: ��ʼ�����ж���ʹ�õĴ���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ���ڲ�����Ĭ��ʹ��9600,�����Ҫ����,����bsp_uart.h�ļ��и��ĺ�: UART_BAUD
*********************************************************************************************************
*/
void bsp_uart_Config(void)
{
	UART_InitTypeDef UART_InitStruct;
	
//	drv_gpio_PullCmd(GPIO_Pin_A2, ENABLE);
//	drv_gpio_PullCmd(GPIO_Pin_A3, ENABLE);
	
	bsp_uart_ParaInit();		/*  �ṹ���ʼ��  */
	
	UART_InitStruct.UART_BaudRate = UART_BAUD;		/*  ������9600  */
	UART_InitStruct.UART_Channel = UART_Channel_1;	/*  ͨ��1  */
	UART_InitStruct.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	/*  �����뷢��,ȫ˫��ģʽ  */
	UART_InitStruct.UART_Parity = UART_Parity_No;	/*  �޼���  */
	UART_InitStruct.UART_StopBits = UART_StopBits_1;	/*  һλֹͣλ  */
	UART_InitStruct.UART_WordLength = UART_WordLength_8b;	/*  8bit  */
	drv_uart_Init(UART0, &UART_InitStruct);
	
	uart_info.uart->C2 |= 1 << 5;		/*  ���������ж�  */
//	uart_info.uart->C2 |= 1 << 4;
//	uart_info.uart->C2 |= 1 << 6;
//	uart_info.uart->C2 |= 1 << 7;
	
	NVIC_EnableIRQ(UART0_IRQn);
		
	drv_uart_Cmd(UART0, ENABLE);		/*  ʹ�ܴ���  */
}


/*
*********************************************************************************************************
*                                      bsp_uart_GetUartNo    
*
* Description: �ɴ��ڱ�Ż�ȡ���ڿ��ƽṹ��
*             
* Arguments  : 1> Port: ���ڱ��, COM0, COM1, COM3
*
* Reutrn     : 1> 0: ���ڱ�Ŵ���
*              2> Uart_Str�ṹ��ָ��: �ɹ�
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
* Description: ����һ��byte�����ݵ����ݽ��ջ�����
*             
* Arguments  : 1> pUart: ���ڿ��ƽṹ��
*              2> byte: Ҫ���͵�����
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_Put(Uart_Str *pUart,uint8_t byte)
{
	pUart->pRxBuff[pUart->RxWrite] = byte;	/*  ������������ջ�����  */
	if(++ pUart->RxWrite >= pUart->RxBuffSize)	/*  ���ζ���  */
		pUart->RxWrite = 0;
	if(pUart->RxCount < pUart->RxBuffSize)
		pUart->RxCount ++;
}

/*
*********************************************************************************************************
*                                     bsp_uart_ClearTxBuff     
*
* Description: ��մ��ڷ��ͻ�����
*             
* Arguments  : 1> Port: ���ڱ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_ClearTxBuff(COM_PORT_ENUM Port)
{
	Uart_Str *pUart = 0;
	pUart = bsp_uart_GetUartNo(Port);		/*  ��ȡ���ڿ��ƽṹ��  */
	
	if(pUart == 0) return ;
	pUart->TxCount = 0;
	pUart->TxRead = 0;
	pUart->TxWrite = 0;
}

/*
*********************************************************************************************************
*                                     bsp_uart_ClearRxBuff     
*
* Description: ��տڽ��ջ�����
*             
* Arguments  : 1> Port: ���ڱ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_ClearRxBuff(COM_PORT_ENUM Port)
{
	Uart_Str *pUart = 0;
	pUart = bsp_uart_GetUartNo(Port);		/*  ��ȡ���ڿ��ƽṹ��  */
	
	if(pUart == 0) return ;
	pUart->RxCount = 0;
	pUart->RxRead = 0;
	pUart->RxWrite = 0;
}

/*
*********************************************************************************************************
*                                          bsp_uart_SendDataToBuff
*
* Description: �������ݵ����ͻ�����,�ȴ�����
*             
* Arguments  : 1> Port: ���ڱ��
*              2> buff: Ҫ���͵����ݵ�ַ
*              3> length: Ҫ���͵����ݳ���
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
	pUart = bsp_uart_GetUartNo(Port);		/*  ��ȡ���ڿ��ƽṹ��  */
	if(pUart == 0) return ;	/*  ��ȡʧ��  */
	
	for (i = 0; i < length; i++)
	{
		/* ������ͻ�������Ϊ�յĻ����ȴ����ͻ��������� */
		while(1)
		{
			__IO uint16_t Count;
			
//			DISABLE_INT();
			Count = pUart->TxCount;
//			ENABLE_INT();
			
			if(Count < pUart->TxBuffSize) break;
		}
		
		pUart->pTxBuff[pUart->TxWrite] = buff[i];		/*  ��������  */
		
	//	DISABLE_INT();
		if(++ pUart->TxWrite >= pUart->TxBuffSize)	/*  ���ζ���  */
		{
			pUart->TxWrite = 0;
		}
		pUart->TxCount ++;		/*  ��Ҫ���͵����ݼ�����  */
//		ENABLE_INT();
	}
	pUart->uart->C2 |= 1 << 7; /*  ��TXE�ж�  */
}


/*
*********************************************************************************************************
*                                         bsp_uart_SendDataToBuff 
*
* Description: �ӽ��ջ��������ȡһ���ֽڵ�����
*             
* Arguments  : 1> Port: ���ڱ��
*              2> byte: ����ָ��,�洢���յ�������
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
	
	pUart = bsp_uart_GetUartNo(Port);  /*  ��ȡ���ڿ��ƽṹ��  */
	if(pUart == 0) return 1;	/*  ��ȡʧ��  */
	Count = pUart->RxCount;

	if(Count == 0)	return 1;  /*  û��������Ҫ��ȡ  */
	else
	{
		*byte = pUart->pRxBuff[pUart->RxRead];	/*  ��ȡ����  */

		if(++ pUart->RxRead >= pUart->RxBuffSize)	/*  ���ζ���  */
			pUart->RxRead = 0;
		
		pUart->RxCount -- ;		/*  �����Ѿ�����ȡ����  */
	}
	return 0;
}

/*
*********************************************************************************************************
*                                     bsp_uart_IRQHandler     
*
* Description: �����ж�ִ�к���,���еĴ����жϷ���ʱ�����ô˺���
*             
* Arguments  : 1> pUart: ���ڿ��ƽṹ��ָ��
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
	
	
//	if(pUart->uart->S1 & UART_S1_RDRF_MASK)  /*  �������ݼĴ�����  */
//	{
////		reg = (uint8_t)pUart->uart->S1;
////		pUart->uart->S2 |= 1 << 6;
//		RecvData = pUart->uart->D;		/*  ��ȡ���ݲ�������ջ�����  */
//		bsp_uart_Put(pUart, RecvData);
//	}
	
	if(pUart->uart->S1 & UART_S1_OR_MASK)
	{
		reg = (uint8_t)pUart->uart->S1;
		RecvData = pUart->uart->D;
	}
	reg = (uint8_t)pUart->uart->S1;
	if(pUart->uart->S1 & UART_S1_TDRE_MASK)	/*  ���ͻ�����Ϊ��,˵�����Է���������  */
	{
		
		if(pUart->TxCount == 0)  /*  û��������Ҫ����  */
		{
			pUart->uart->C2 &= ~(1 << 7); /*  �ر�TXE�ж�  */
//			pUart->uart->C2 |= 1 << 6;    /*  �򿪷�������ж�  */
			
		}
		else
		{
			pUart->uart->D = (uint8_t)pUart->pTxBuff[pUart->TxRead]; /*  ������д�뷢�ͻ�����  */ 
			if(++ pUart->TxRead >= pUart->TxBuffSize)  /*  ���ζ���  */
			{
				pUart->TxRead = 0;
			}
			pUart->TxCount --;		/*  ��Ҫ���͵����ݵݼ�  */
		}
	}
//	else if((pUart->uart->S1 & 0x40) != RESET)  /*  ��������ж�  */
//	{
//		
//		if(pUart->TxCount == 0)  /*  û��������Ҫ����  */
//		{
//			pUart->uart->C2 &= ~(1 << 6);  /*  �رշ�������ж�  */
//			pUart->uart->C2 &= ~(1 << 3);
//			pUart->uart->C2 |= (1 << 3);
//			if(pUart->_cbSendOver)		/*  ��������˷�����ɻص�����  */
//				pUart->_cbSendOver(0);
//		}
//		else			/*  ����������Ҫ����  */
//		{
//			pUart->uart->D = pUart->pTxBuff[pUart->TxRead]; /*  ������д�뷢�ͻ�����  */ 
//			if(++ pUart->TxRead >= pUart->TxBuffSize)  /*  ���ζ���  */
//			{
//				pUart->TxRead = 0;
//			}
//			pUart->TxCount --;		/*  ��Ҫ���͵����ݵݼ�  */
//		}
//	}
}


/*
*********************************************************************************************************
*                                   bsp_uart_Printf       
*
* Description: �Զ��崮�ڴ�ӡ����
*             
* Arguments  : 1> format: �����ʽ�����ַ���
*              2> ...: ���Ȳ����β�,��Ҫ��ӡ������
*
* Reutrn     : None.
*
* Note(s)    : Ĭ�ϵ��ô���0,��Ҫ�����������
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
* Description: ����0�жϺ���
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


/*  �������´���,֧��printf, ������Ҫѡ��use MicroLIB  */
#if 1
#pragma import(__use_no_semihosting)             
 
struct __FILE /*  ��׼����Ҫ��֧�ֺ���  */
{ 
	int handle; 
}; 

FILE __stdout;       
   
/*  ����_sys_exit() ����ʹ�ð�����ģʽ  */
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 

/*  �ض���fputc����,֧��printf  */
int fputc(int ch, FILE *f)
{ 	
	while((UART0->S1&0X80)==0);//ѭ������,ֱ���������   
	UART0->D = (uint8_t) ch;      
	return ch;
}
#endif


/********************************************  END OF FILE  *******************************************/

