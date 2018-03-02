/**
  *******************************************************************************************************
  * File Name: drv_gpio.h
  * Author: Vector
  * Version: V2.3.1
  * Date: 2018-2-1
  * Brief: ���ļ�����GPIO��ص��������������,ͬʱ�����˲���GPIO�ĺ������ṹ�塢ö�ٱ�����
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *			Author: Vector
  *			Mod: �����ļ�
	*
	*		2.Data: 2018-2-9
	*     Author: Vector
	*     Mod: ���������Դ�Ķ���,�Լ����Ÿ��ù��ܵĶ���
	*
	*		3.Data: 2018-2-17
	*			Author:	Vector
	*			Mod:	��������״̬ö�ٱ���,�޸���������״̬��������
	*
	*		4.Date:2018-2-28
	*			Author: Vector
	*			Mod: 1.����º���drv_gpio_PullCmd������,�����������ŵ���������
	*					 2.ɾ��GPIOBPin_TypeDef.GPIOCPin_TypeDefö�ٱ���,�������ű�ʾ��ʽ,������ö�ٱ���GPIOPin_TypeDef��
	*          3.���ĺ��������߼�,����Ҫ�ֶ�����˿ں�,���ɺ������
  *
  *******************************************************************************************************
  */
# ifndef __DRV_GPIO_H
# define __DRV_GPIO_H

# include "derivative.h"

/*  �������PORT,GPIOA�Ĵ�����PORTA-PORTD,32��IO
		GPIOB��PORTE-PORTH,32��IO��
		GPIOC��PORTI,8��IO
*/
#define PORTA  GPIOA
#define PORTB  GPIOA
#define PORTC  GPIOA
#define PORTD  GPIOA

#define PORTE	 GPIOB
#define PORTF	 GPIOB
#define PORTG  GPIOB
#define PORTH	 GPIOB
 
#define PORTI	 GPIOC

/*  ������Դ�Ķ���  */
# define GPIO_PinSource_F0	(uint8_t)0x02
# define GPIO_PinSource_H0	(uint8_t)0x01
# define GPIO_PinSource_C0	(uint8_t)0x00

# define GPIO_PinSource_C1	(uint8_t)0x00
# define GPIO_PinSource_H1	(uint8_t)0x01
# define GPIO_PinSource_F1	(uint8_t)0x02

# define GPIO_PinSource_C2	(uint8_t)0x00
# define GPIO_PinSource_D0	(uint8_t)0x01
# define GPIO_PinSource_G4	(uint8_t)0x02

# define GPIO_PinSource_C3	(uint8_t)0x00
# define GPIO_PinSource_D1	(uint8_t)0x01
# define GPIO_PinSource_G5	(uint8_t)0x02

# define GPIO_PinSource_B4	(uint8_t)0x00
# define GPIO_PinSource_G6	(uint8_t)0x01

# define GPIO_PinSource_B5	(uint8_t)0x00
# define GPIO_PinSource_G7	(uint8_t)0x01

# define GPIO_PinSource_D5	(uint8_t)0x00
# define GPIO_PinSource_E2	(uint8_t)0x01

# define GPIO_PinSource_B0	(uint8_t)0x00
# define GPIO_PinSource_H7	(uint8_t)0x00

# define GPIO_PinSource_A5	(uint8_t)0x00
# define GPIO_PinSource_I0	(uint8_t)0x01
# define GPIO_PinSource_I1	(uint8_t)0x02
# define GPIO_PinSource_I2	(uint8_t)0x03
# define GPIO_PinSource_I3	(uint8_t)0x04
# define GPIO_PinSource_I4	(uint8_t)0x05
# define GPIO_PinSource_I5	(uint8_t)0x06
# define GPIO_PinSource_I6	(uint8_t)0x07

# define GPIO_PinSource_C4	(uint8_t)0x00
# define GPIO_PinSource_C5	(uint8_t)0x01

# define GPIO_PinSource_A0	(uint8_t)0x00
# define GPIO_PinSource_B2	(uint8_t)0x01

# define GPIO_PinSource_A1	(uint8_t)0x00
# define GPIO_PinSource_B3	(uint8_t)0x01

# define GPIO_PinSource_C4	(uint8_t)0x00
# define GPIO_PinSource_H2	(uint8_t)0x01

# define GPIO_PinSource_E7	(uint8_t)0x01


/*  ���Ÿ��ù��ܶ���  */
# define GPIO_AF_FTM0_CH0		(uint16_t)0x0008
# define GPIO_AF_FTM0_CH1		(uint16_t)0x0009

# define GPIO_AF_FTM1_CH0		(uint16_t)0x000a
# define GPIO_AF_FTM1_CH1		(uint16_t)0x000b

# define GPIO_AF_FTM2_CH0		(uint16_t)0x0100
# define GPIO_AF_FTM2_CH1		(uint16_t)0x0102
# define GPIO_AF_FTM2_CH2		(uint16_t)0x0104
# define GPIO_AF_FTM2_CH3		(uint16_t)0x0106
# define GPIO_AF_FTM2_CH4		(uint16_t)0x0108
# define GPIO_AF_FTM2_CH5		(uint16_t)0x0109

# define GPIO_AF_PWT_IN0		(uint16_t)0x010e
# define GPIO_AF_PWT_IN1		(uint16_t)0x010f

# define GPIO_AF_IRQ				(uint16_t)0x0000
# define GPIO_AF_RTCO				(uint16_t)0x0004


/*  ���ų�ʼ���ṹ��  */
typedef struct 
{
	uint8_t GPIO_Pin;		/*  ����  */
	uint8_t GPIO_Mode;	/*  ģʽ  */
	uint8_t GPIO_PuPd;	/*  �������  */
	uint8_t GPIO_HDrv;	/*  �������������ѡ��,��ѡ��ֻ��ĳ������������  */
}GPIO_InitTypeDef;


/*  ����ģʽö�ٱ���  */
typedef enum
{
	GPIO_Mode_IN = 0x00,
	GPIO_Mode_OUT = 0x01,
}GPIOMode_TypeDef;


/*  ������/����ģʽö�ٱ���  */
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,		/*  ����/����  */
  GPIO_PuPd_UP     = 0x01,		/*  �ڲ�����  */
  GPIO_PuPd_DOWN   = 0x02,		/*  �ڲ�����,KEA��֧��  */
}GPIOPuPd_TypeDef;


/*  GPIOA�Ĵ����ܵ�����ö�ٱ���  */
typedef enum
{
	GPIO_Pin_A0 = 0x0,
	GPIO_Pin_A1,
	GPIO_Pin_A2,
	GPIO_Pin_A3,
	GPIO_Pin_A4,
	GPIO_Pin_A5,
	GPIO_Pin_A6,
	GPIO_Pin_A7,
	
	GPIO_Pin_B0,
	GPIO_Pin_B1,
	GPIO_Pin_B2,
	GPIO_Pin_B3,
	GPIO_Pin_B4,
	GPIO_Pin_B5,
	GPIO_Pin_B6,
	GPIO_Pin_B7,
	
	GPIO_Pin_C0,
	GPIO_Pin_C1,
	GPIO_Pin_C2,
	GPIO_Pin_C3,
	GPIO_Pin_C4,
	GPIO_Pin_C5,
	GPIO_Pin_C6,
	GPIO_Pin_C7,
	
	GPIO_Pin_D0,
	GPIO_Pin_D1,
	GPIO_Pin_D2,
	GPIO_Pin_D3,
	GPIO_Pin_D4,
	GPIO_Pin_D5,
	GPIO_Pin_D6,
	GPIO_Pin_D7,
	
	
	GPIO_Pin_E0,
	GPIO_Pin_E1,
	GPIO_Pin_E2,
	GPIO_Pin_E3,
	GPIO_Pin_E4,
	GPIO_Pin_E5,
	GPIO_Pin_E6,
	GPIO_Pin_E7,
	
	GPIO_Pin_F0,
	GPIO_Pin_F1,
	GPIO_Pin_F2,
	GPIO_Pin_F3,
	GPIO_Pin_F4,
	GPIO_Pin_F5,
	GPIO_Pin_F6,
	GPIO_Pin_F7,
	
	GPIO_Pin_G0,
	GPIO_Pin_G1,
	GPIO_Pin_G2,
	GPIO_Pin_G3,
	GPIO_Pin_G4,
	GPIO_Pin_G5,
	GPIO_Pin_G6,
	GPIO_Pin_G7,
	
	GPIO_Pin_H0,
	GPIO_Pin_H1,
	GPIO_Pin_H2,
	GPIO_Pin_H3,
	GPIO_Pin_H4,
	GPIO_Pin_H5,
	GPIO_Pin_H6,
	GPIO_Pin_H7,
	
	
	GPIO_Pin_I0,
	GPIO_Pin_I1,
	GPIO_Pin_I2,
	GPIO_Pin_I3,
	GPIO_Pin_I4,
	GPIO_Pin_I5,
	GPIO_Pin_I6,
	GPIO_Pin_I7,
}GPIOPin_TypeDef;


/*  GPIO����״̬ö�ٱ���  */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET = 1,
}GPIO_PinState;

/*  ���ⲿʹ�õ�GPIO���Ų�������  */
void drv_gpio_Init(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t drv_gpio_ReadPin(uint8_t GPIO_Pin);
void drv_gpio_WritePin(uint8_t GPIO_Pin, GPIO_PinState PinState);
void drv_gpio_TogglePin(uint8_t GPIO_Pin);
void drv_gpio_PinAFConfig(uint8_t GPIO_PinSource, uint16_t GPIO_AF);
void drv_gpio_PullCmd(uint8_t GPIO_Pin, FunctionalState NewState);
# endif

/********************************************  END OF FILE  *******************************************/
