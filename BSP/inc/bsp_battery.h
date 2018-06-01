/**
  *******************************************************************************************************
  * File Name: bsp_battery.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-17
  * Brief: 本文件提供了测量车子电池电量的功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-17
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_BATTERY_H
# define __BSP_BATTERY_H


# define BAT_CHANNEL		ADC_Channel_A1
# define BAT_FULL_VOL		8.0f
# define BAT_LOW_VOL		6.8f

# define R1 2.2f
# define R2 1.92f

void bsp_bat_Config(void);
float bsp_bat_GetVol(void);

# endif


/********************************************  END OF FILE  *******************************************/

