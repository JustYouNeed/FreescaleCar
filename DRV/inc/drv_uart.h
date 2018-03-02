# ifndef __DRV_UART_H
# define __DRV_UART_H

# include "derivative.h"

# define  UART_SR_PF                         ((uint16_t)0x0401)            /*  奇偶检验错误标志  */
# define  UART_SR_FE                         ((uint16_t)0x0402)            /*  帧传输错误标志  */
# define  UART_SR_NF                         ((uint16_t)0x0404)            /*  噪声标志  */
# define  UART_SR_ORE                        ((uint16_t)0x0408)            /*  接收器溢出标志  */
# define  UART_SR_IDLE                       ((uint16_t)0x0410)            /*  线路空闲标志  */
# define  UART_SR_RXNE                       ((uint16_t)0x0420)            /*  接收数据寄存器满标志  */
# define  UART_SR_TC                         ((uint16_t)0x0440)            /*  发送完成标志  */
# define  UART_SR_TXE                        ((uint16_t)0x0480)            /*  发送数据寄存器空标志  */

# define  UART_SR_RAF												 ((uint16_t)0x0501)						 /*  接收器有效标志  */
# define  UART_SR_LBDE											 ((uint16_t)0x0502)						 /*  LIN断点检测使能  */
# define  UART_SR_BRK												 ((uint16_t)0x0504)						 /*  断点字符生成长度  */		 
# define  UART_SR_RWUID											 ((uint16_t)0x0508)						 /*  接收唤醒空闲检测  */
# define  UART_SR_RXINV											 ((uint16_t)0x0510)						 /*  接收数据反转  */
# define  UART_SR_RESV											 ((uint16_t)0x0520)						 /*  保留  */
# define  UART_SR_RXED											 ((uint16_t)0x0540)						 /*  RXD引脚有效边沿中断标志  */
# define  UART_SR_LBKDIF										 ((uint16_t)0x0580)						 /*  LIN断点检测中断标志  */

# define UART_Mode_Rx                        ((uint8_t)0x04)
# define UART_Mode_Tx                        ((uint8_t)0x08)


#define UART_Parity_No                      ((uint8_t)0x00)
#define UART_Parity_Even                    ((uint8_t)0x02)
#define UART_Parity_Odd                     ((uint8_t)0x03) 

#define UART_StopBits_1                     ((uint8_t)0x00)
#define UART_StopBits_2											((uint8_t)0x01)

#define UART_WordLength_8b                  ((uint8_t)0x00)
#define UART_WordLength_9b                  ((uint8_t)0x10)

typedef enum
{
	UART_Channel_0 = 0x0,
	UART_Channel_1,
}UARTChanel_TypeDef;

typedef struct
{
	uint32_t UART_BaudRate;
	uint16_t UART_WordLength;
	uint16_t UART_StopBits;
	uint16_t UART_Parity;
	uint16_t UART_Mode;
	uint32_t UART_Channel;
}UART_InitTypeDef;


uint32_t drv_uart_Init(UART_Type *UARTx, UART_InitTypeDef *UART_InitStruct);
void drv_uart_DeInit(UART_Type *UARTx);
void drv_uart_Cmd(UART_Type *UARTx, FunctionalState NewState);
void drv_uart_SelectChannle(UART_Type *UARTx, uint8_t Channel);
void drv_uart_StructInit(UART_InitTypeDef *UART_InitStruct);
void drv_uart_ITConfig(UART_Type *UARTx, uint16_t UART_IT, FunctionalState NewState);
FlagStatus drv_uart_GetITStatus(UART_Type *UARTx, uint16_t UART_IT);
void drv_uart_SendData(UART_Type *UARTx, uint8_t data);
uint8_t drv_uart_ReceiveData(UART_Type *UARTx);



# endif
