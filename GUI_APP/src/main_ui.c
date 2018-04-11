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

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "main_ui.h"
# include "bsp_oled.h"
# include "font.h"
# include "bsp_led.h"

uint8_t isMainUIChange = true;
void main_ui(void)
{
	if(isMainUIChange == false) return;
	
	oled_showPicture(108, 4, bmp_battery[12], 10, 16);
	oled_showPicture(0,0,bmp_rssi[5],24,22);
	oled_showString(26,30,"Vector", 12, 24);
	
	isMainUIChange = false;
}




/********************************************  END OF FILE  *******************************************/
