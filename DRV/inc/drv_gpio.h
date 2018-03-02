/**
  *******************************************************************************************************
  * File Name: drv_gpio.h
  * Author: Vector
  * Version: V2.3.1
  * Date: 2018-2-1
  * Brief: 该文件对与GPIO相关的外设进行了声明,同时声明了操作GPIO的函数、结构体、枚举变量等
  *******************************************************************************************************
  * History
  *		1.Date: 2018-2-1
  *			Author: Vector
  *			Mod: 建立文件
	*
	*		2.Data: 2018-2-9
	*     Author: Vector
	*     Mod: 添加引脚资源的定义,以及引脚复用功能的定义
	*
	*		3.Data: 2018-2-17
	*			Author:	Vector
	*			Mod:	新增引脚状态枚举变量,修复设置引脚状态函数错误
	*
	*		4.Date:2018-2-28
	*			Author: Vector
	*			Mod: 1.添加新函数drv_gpio_PullCmd的声明,用于设置引脚的上拉电阻
	*					 2.删除GPIOBPin_TypeDef.GPIOCPin_TypeDef枚举变量,更改引脚表示方式,整个到枚举变量GPIOPin_TypeDef中
	*          3.更改函数调用逻辑,不需要手动输入端口号,改由函数求出
  *
  *******************************************************************************************************
  */
# ifndef __DRV_GPIO_H
# define __DRV_GPIO_H

# include "derivative.h"

/*  定义相关PORT,GPIOA寄存器管PORTA-PORTD,32个IO
		GPIOB管PORTE-PORTH,32个IO，
		GPIOC管PORTI,8个IO
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

/*  引脚资源的定义  */
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


/*  引脚复用功能定义  */
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


/*  引脚初始化结构体  */
typedef struct 
{
	uint8_t GPIO_Pin;		/*  引脚  */
	uint8_t GPIO_Mode;	/*  模式  */
	uint8_t GPIO_PuPd;	/*  输出类型  */
	uint8_t GPIO_HDrv;	/*  大电流驱动能力选项,该选项只对某几个引脚有用  */
}GPIO_InitTypeDef;


/*  引脚模式枚举变量  */
typedef enum
{
	GPIO_Mode_IN = 0x00,
	GPIO_Mode_OUT = 0x01,
}GPIOMode_TypeDef;


/*  引脚上/下拉模式枚举变量  */
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,		/*  无上/下拉  */
  GPIO_PuPd_UP     = 0x01,		/*  内部上拉  */
  GPIO_PuPd_DOWN   = 0x02,		/*  内部下拉,KEA不支持  */
}GPIOPuPd_TypeDef;


/*  GPIOA寄存器管的引脚枚举变量  */
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


/*  GPIO引脚状态枚举变量  */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET = 1,
}GPIO_PinState;

/*  供外部使用的GPIO引脚操作函数  */
void drv_gpio_Init(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t drv_gpio_ReadPin(uint8_t GPIO_Pin);
void drv_gpio_WritePin(uint8_t GPIO_Pin, GPIO_PinState PinState);
void drv_gpio_TogglePin(uint8_t GPIO_Pin);
void drv_gpio_PinAFConfig(uint8_t GPIO_PinSource, uint16_t GPIO_AF);
void drv_gpio_PullCmd(uint8_t GPIO_Pin, FunctionalState NewState);
# endif

/********************************************  END OF FILE  *******************************************/
