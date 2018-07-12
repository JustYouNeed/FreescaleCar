/**
  *******************************************************************************************************
  * File Name: drv_uart.h
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

# ifndef __DRV_UART_H
# define __DRV_UART_H


# define  UART_SR_PF                         ((uint16_t)0x0401)            /*  ��ż��������־  */
# define  UART_SR_FE                         ((uint16_t)0x0402)            /*  ֡��������־  */
# define  UART_SR_NF                         ((uint16_t)0x0404)            /*  ������־  */
# define  UART_SR_ORE                        ((uint16_t)0x0408)            /*  �����������־  */
# define  UART_SR_IDLE                       ((uint16_t)0x0410)            /*  ��·���б�־  */
# define  UART_SR_RXNE                       ((uint16_t)0x0420)            /*  �������ݼĴ�������־  */
# define  UART_SR_TC                         ((uint16_t)0x0440)            /*  ������ɱ�־  */
# define  UART_SR_TXE                        ((uint16_t)0x0480)            /*  �������ݼĴ����ձ�־  */

# define  UART_SR_RAF												 ((uint16_t)0x0501)						 /*  ��������Ч��־  */
# define  UART_SR_LBDE											 ((uint16_t)0x0502)						 /*  LIN�ϵ���ʹ��  */
# define  UART_SR_BRK												 ((uint16_t)0x0504)						 /*  �ϵ��ַ����ɳ���  */		 
# define  UART_SR_RWUID											 ((uint16_t)0x0508)						 /*  ���ջ��ѿ��м��  */
# define  UART_SR_RXINV											 ((uint16_t)0x0510)						 /*  �������ݷ�ת  */
# define  UART_SR_RESV											 ((uint16_t)0x0520)						 /*  ����  */
# define  UART_SR_RXED											 ((uint16_t)0x0540)						 /*  RXD������Ч�����жϱ�־  */
# define  UART_SR_LBKDIF										 ((uint16_t)0x0580)						 /*  LIN�ϵ����жϱ�־  */

# define UART_Mode_Rx                        ((uint8_t)0x04)
# define UART_Mode_Tx                        ((uint8_t)0x08)


#define UART_Parity_No                      ((uint8_t)0x00)
#define UART_Parity_Even                    ((uint8_t)0x02)
#define UART_Parity_Odd                     ((uint8_t)0x03) 

#define UART_StopBits_1                     ((uint8_t)0x00)
#define UART_StopBits_2											((uint8_t)0x01)

#define UART_WordLength_8b                  ((uint8_t)0x00)
#define UART_WordLength_9b                  ((uint8_t)0x10)

/*  ����ͨ��  */
typedef enum
{
	UART_Channel_0 = 0x0,
	UART_Channel_1,
}UARTChanel_TypeDef;

/*  ���ڳ�ʼ���ṹ��  */
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

/********************************************  END OF FILE  *******************************************/

