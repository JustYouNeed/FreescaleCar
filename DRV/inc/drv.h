/**
  *******************************************************************************************************
  * File Name: drv.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-24
  * Brief: ���ļ�ΪKEA128������ļ���ͷ�ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-24
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
# ifndef __DRV_H
# define __DRV_H

/*  ����/�ر����ж�  */
# define ENABLE_INT()		__enable_irq()
# define DISABLE_INT()			__disable_irq()


typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;



/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "drv_gpio.h"
# include "drv_pit.h"
# include "drv_uart.h"
# include "drv_rcc.h"
# include "drv_adc.h"
# include "drv_acmp.h"
# include "drv_flash.h"
# include "drv_irq.h"
# include "drv_power.h"
# include "drv_pwt.h"
# include "drv_rtc.h"
# include "drv_spi.h"
# include "drv_wdog.h"
# include "drv_ftm.h"
# include "drv_kbi.h"

# endif
/********************************************  END OF FILE  *******************************************/

