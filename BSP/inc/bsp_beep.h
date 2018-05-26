/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 2018-4-18
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_BEEP_H
# define __BSP_BEEP_H
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/



typedef struct 
{
	uint8_t ucEnable;
	uint8_t ucState;
	uint16_t usBeepTime;
	uint16_t usStopTime;
	uint16_t usCycle;
	uint16_t usCount;
	uint16_t usCycleCount;
	uint8_t ucMute;
}BEEP_TypeDef;

void bsp_beep_Config(void);
void bsp_beep_ON(uint16_t BeepTime, uint16_t StopTime, uint16_t Cycle);
void bsp_beep_OFF(void);
void bsp_beep_KeyTone(void);
void bsp_beep_Pause(void);
void bsp_beep_Resume(void);
void bsp_beep_Thread(void);
# endif


/********************************************  END OF FILE  *******************************************/
