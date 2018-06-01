/**
  *******************************************************************************************************
  * File Name: bsp_uart.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-11
  * Brief: 该文件声明了有关串口操作的变量与函数
  *******************************************************************************************************
  * History
	*		1.Author: Vector
	*			Date:	2018-2-11
  *			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_UART_H
# define __BSP_UART_H


# define UART_BAUD		115200
# define UART_TX_BUFF_SIZE	256
# define UART_RX_BUFF_SIZE	256


/*  串口编号枚举变量  */
typedef enum
{
	COM0 = 0x01,
	COM1 = 0x02,
	COM2 = 0x03,
}COM_PORT_ENUM;


/*  串口控制结构体定义  */
typedef struct
{
	UART_Type *uart;	/*  串口指针  */
	uint8_t *pTxBuff;	/*  发送数据缓存区  */
	uint8_t *pRxBuff;	/*  接收数据缓存区  */
	uint16_t TxBuffSize;	/*  发送缓存区大小  */
	uint16_t RxBuffSize;	/*  接收缓存区大小  */
	
	__IO uint16_t TxWrite;	/*  用于发送FIFO的写变量  */
	__IO uint16_t TxRead;		/*  用于发送FIFO的读变量  */
	__IO uint16_t TxCount;	/*  统计还需要发送的数据  */
	
	__IO uint16_t RxWrite;	/*  用于接收FIFO的写变量  */
	__IO uint16_t RxRead;		/*  用于接收数据的读变量  */
	__IO uint16_t RxCount;	/*  统计还需要被读取的数据量  */
	
	void (*_cbSendBefor)(void *p_arg);	/*  发送数据前的回调指针  */
	void (*_cbSendOver)(void *p_arg);		/*  发送后的回调指针  */
	void (*_cbRecvData)(uint8_t byte);	/*  接收到数据时的回调指针  */
}Uart_Str;

void bsp_uart_Config(void);
void bsp_uart_Put(Uart_Str *pUart,uint8_t byte);
void bsp_uart_ClearTxBuff(COM_PORT_ENUM Port);
void bsp_uart_ClearRxBuff(COM_PORT_ENUM Port);
void bsp_uart_SendDataToBuff(COM_PORT_ENUM Port, uint8_t *buff, uint16_t length);
uint8_t bsp_uart_GetChar(COM_PORT_ENUM Port, uint8_t *byte);
uint8_t bsp_uart_Printf(const char *format, ...);

void bsp_uart_IRQHandler(Uart_Str *pUart);

# endif


/********************************************  END OF FILE  *******************************************/
