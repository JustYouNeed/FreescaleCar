# include "para_show.h"
# include "gui_menu.h"
# include "lcmdrv.h"
# include "windows.h"
# include "FreescaleCar.h"
# include "text.h"
# include "button.h"
#include "scrollbar.h"
# include "display.h"

extern uint8_t key;
static uint8_t showFlag = 0;
static bool isWindowChange = true;
WINDOWS DebugWindow={
.x = 0,
.y = 0,	
.width = 128,
.height = 64,
.itemsperpage = 3,
.topitem = 0,
.title = "Sensor Parameters",
};

Scrollbar_Typedef DebugScrollbar={
.x = 118,
.y = 14,
.width = 10,
.height = 50,
.itemsperpage = 3,
.topitem = 0,
.scbbarlen = 0,
};

void Para_ShowRefresh(void)
{
	if(isWindowChange == false) return;
	GUI_WindowsDraw(&DebugWindow);
	GUI_ScrollbarDraw(&DebugScrollbar);
	isWindowChange = false;
}

void Para_Show_UI(void)
{
	Para_ShowRefresh();
	if(showFlag == 0)
	{
		/*  显示电感值  */
		oled_showString(3,15,"H_L:", 6, 12);
		oled_showNum(27, 15, Car.Sensor[SENSOR_H_L].Average, 3, 6, 12);
		
		oled_showString(65,15,"H_R:", 6, 12);
		oled_showNum(95, 15, Car.Sensor[SENSOR_H_R].Average, 3, 6, 12);
		
		oled_showString(3,33,"V_L:", 6, 12);
		oled_showNum(27, 33, Car.Sensor[SENSOR_V_L].Average, 3, 6, 12);
		
		oled_showString(65,33,"V_R:", 6, 12);
		oled_showNum(95, 33, Car.Sensor[SENSOR_V_R].Average, 3, 6, 12);
		
		oled_showString(3,50,"HAE:", 6, 12);
		oled_showNum(27, 50, (Car.HorizontalAE*100), 3, 6, 12);
		
		oled_showString(65,50,"VAE:", 6, 12);
		oled_showNum(95, 50, (Car.VecticalAE*100), 3, 6, 12);
	}
	else if(showFlag == 1)
	{
		/*  电机编码器  */
		oled_showString(55, 15, "L_E:", 6, 12);
		oled_showNum(85, 15, Car.Motor.LeftEncoder, 4, 6, 12);
		
		oled_showString(55, 27, "R_E:", 6, 12);
		oled_showNum(85, 27, Car.Motor.RightEncoder, 4, 6, 12);
		
		oled_showString(55, 39, "L_P:", 6, 12);
		oled_showNum(85, 39, Car.Motor.LeftPwm, 4, 6, 12);
		
		oled_showString(55, 51, "R_P:", 6, 12);
		oled_showNum(85, 51, Car.Motor.RightPwm, 4, 6, 12);
	}
	
	if(key == KEY_DOWN_PRESS && showFlag == 0)
	{
		showFlag++;
		isWindowChange = true;
		DebugWindow.title = "Motor Parameters";
	}
	else if(key == KEY_UP_PRESS && showFlag == 1)
	{
		showFlag--;
		isWindowChange = true;
		DebugWindow.title = "Sensor Parameters";
	}else if(key == KEY_OK_PRESS)
	{
		isWindowChange = true;
		setShow_ui(MAIN_UI);
	}

}


