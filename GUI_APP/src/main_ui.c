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
# include "bsp.h"
# include "main_ui.h"
# include "font.h"


uint8_t isMainUIChange = true;
void main_ui(void)
{
	float vol = 0.0f;
	static uint8_t batPercent = 0;
	
//	if(isMainUIChange == false) return;
	
	oled_showPicture(0,0,bmp_rssi[5],24,22);
	oled_showString(26,30,"Vector", 12, 24);
	
//	vol = bsp_bat_GetVol();
//	batPercent = (vol - BAT_LOW_VOL) / (BAT_FULL_VOL - BAT_LOW_VOL) * 100;
//	if(batPercent >= 99) batPercent = 99;
//	if(batPercent <= 0) batPercent = 0;
//	oled_showPicture(100, 4, bmp_battery[batPercent*12/99], 10, 16);
//	oled_showNum(110, 7, batPercent, 3, 6, 12);
	
	isMainUIChange = false;
}




/********************************************  END OF FILE  *******************************************/
