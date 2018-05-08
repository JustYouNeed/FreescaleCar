/**
  *******************************************************************************************************
  * File Name: bsp_enconder.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-28
  * Brief: 本文件提供了车子编码器的驱动函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-18
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_ENCODER_H
# define __BSP_ENCODER_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

typedef enum
{
    ftm0,
    ftm1,
    ftm2,
} FTMn;

# define LEFTENCONDER_DIR_PIN		GPIO_Pin_B2

# define RIGHTENCONDER_DIR_PIN		GPIO_Pin_E6

# define READ_DIR(pin)		drv_gpio_ReadPin(pin)

void bsp_encoder_Config(void);
uint16_t ftm_count_get(FTMn ftmn);
void ftm_count_clean(FTMn ftmn);
void bsp_encoder_ReadCounter(void);

# endif

/********************************************  END OF FILE  *******************************************/

