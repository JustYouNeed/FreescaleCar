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
static int8_t showFlag = 0;
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
		oled_showNum(27, 50, (int16_t)(Car.HorizontalAE*100), 3, 6, 12);
		
		oled_showString(65,50,"VAE:", 6, 12);
		oled_showNum(95, 50, (int16_t)(Car.VecticalAE*100), 3, 6, 12);
	}
	else if(showFlag == 1)
	{
		/*  电机编码器  */
		oled_showString(3, 15, "L_E:", 6, 12);
		oled_showNum(27, 15, Car.Motor.LeftEncoder, 4, 6, 12);
		
		oled_showString(55, 15, "R_E:", 6, 12);
		oled_showNum(85, 15, Car.Motor.RightEncoder, 4, 6, 12);
		
		oled_showString(3, 33, "L_P:", 6, 12);
		oled_showNum(27, 33, Car.Motor.LeftPwm, 4, 6, 12);
		
		oled_showString(55, 33, "R_P:", 6, 12);
		oled_showNum(85, 33, Car.Motor.RightPwm, 4, 6, 12);
		
		oled_showString(3,50,"Nor:", 6, 12);
		oled_showNum(27, 50, (Car.Sensor[0].CalibrationMax), 3, 6, 12);
		
		oled_showString(55,50,"Nor:", 6, 12);
		oled_showNum(85, 50, (Car.Sensor[0].CalibrationMax), 3, 6, 12);
	}else if(showFlag == 2)
	{
		oled_showString(3, 15, "SKP:", 6, 12);
		oled_showNum(27, 15, Car.PID.SpeedKp*10, 4, 6, 12);
		
		oled_showString(55, 15, "SKI:", 6, 12);
		oled_showNum(85, 15, Car.PID.SpeedKi*10, 4, 6, 12);
		
		oled_showString(3, 33, "DKP:", 6, 12);
		oled_showNum(27, 33, Car.PID.DirctionKp, 4, 6, 12);
		
		oled_showString(55, 33, "DKD:", 6, 12);
		oled_showNum(85, 33, Car.PID.DirctionKd, 4, 6, 12);
	}
	
	if(key == KEY_DOWN_PRESS)
	{
		showFlag--;
		isWindowChange = true;
		if(showFlag < 0)
			showFlag = 2;
		switch(showFlag)
		{
			case 0: DebugWindow.title = "Motor Parameters";break;
			case 1: DebugWindow.title = "Sensor Parameters";break;
			case 2: DebugWindow.title = "PID Parameters";
			default: break;
		}
	}else if(key == KEY_UP_PRESS)
	{
		showFlag++;
		isWindowChange = true;
		if(showFlag>2)
			showFlag = 0;
		
		switch(showFlag)
		{
			case 0: DebugWindow.title = "Motor Parameters";break;
			case 1: DebugWindow.title = "Sensor Parameters";break;
			case 2: DebugWindow.title = "PID Parameters";
			default: break;
		}
	}
	else	if(key == KEY_OK_PRESS)
	{
		isWindowChange = true;
		setShow_ui(MAIN_UI);
	}

}


