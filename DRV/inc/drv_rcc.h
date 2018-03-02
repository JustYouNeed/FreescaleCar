/**
  *******************************************************************************************************
  * File Name: drv_rcc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-1
  * Brief: 该文件声明了有关RCC的变量与操作函数
  *******************************************************************************************************
  * History
	*		1.Author: Vector
	*			Date:	2018-2-1
  *			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __DRV_RCC_H
# define __DRV_RCC_H

# include "derivative.h"

# define RCC_PeriphClock_RTC			((uint32_t)0x00000001)
# define RCC_PeriphClock_PIT			((uint32_t)0x00000002)
# define RCC_PeriphClock_PWT			((uint32_t)0x00000010)
# define RCC_PeriphClock_FTM0			((uint32_t)0x00000020)
# define RCC_PeriphClock_FTM1			((uint32_t)0x00000040)
# define RCC_PeriphClock_FTM2			((uint32_t)0x00000080)
# define RCC_PeriphClock_CRC			((uint32_t)0x00000400)
# define RCC_PeriphClock_FLASH		((uint32_t)0x00001000)
# define RCC_PeriphClock_SWD			((uint32_t)0x00002000)
# define RCC_PeriphClock_MSCAN		((uint32_t)0x00008000)
# define RCC_PeriphClock_I2C0			((uint32_t)0x00010000)
# define RCC_PeriphClock_I2C1			((uint32_t)0x00020000)
# define RCC_PeriphClock_SPI0			((uint32_t)0x00040000)
# define RCC_PeriphClock_SPI1			((uint32_t)0x00080000)
# define RCC_PeriphClock_UART0		((uint32_t)0x00100000)
# define RCC_PeriphClock_UART1		((uint32_t)0x00200000)
# define RCC_PeriphClock_UART2		((uint32_t)0x00400000)
# define RCC_PeriphClock_KBI0			((uint32_t)0x01000000)
# define RCC_PeriphClock_KBI1			((uint32_t)0x02000000)
# define RCC_PeriphClock_IRQ			((uint32_t)0x08000000)
# define RCC_PeriphClock_ADC			((uint32_t)0x20000000)
# define RCC_PeriphClock_ACMP0		((uint32_t)0x40000000)
# define RCC_PeriphClock_ACMP1		((uint32_t)0x80000000)

void drv_rcc_ClockCmd(uint32_t PeriphColck, FunctionalState NewState);

# endif

