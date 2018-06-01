/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
	
# ifndef __BSP_SWITCH_H
# define __BSP_SWITCH_H


# define BIT1_PIN			GPIO_Pin_F0
# define BIT2_PIN			GPIO_Pin_F1
# define BIT3_PIN			GPIO_Pin_G4
# define BIT4_PIN			GPIO_Pin_G7



void bsp_switch_Config(void);

uint8_t bsp_switch_GetValue(void);
# endif

/********************************************  END OF FILE  *******************************************/

