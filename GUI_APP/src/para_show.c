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
static int8_t showPage = 0;
static bool isWindowChange = true;

static int16_t HAEFifo[128];
static uint8_t fifocnt = 0;
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
	
	/*  第一页显示电感参数  */
	if(showPage == 0)
	{
		/*  前排左边水平电感  */
		oled_showString(3,15,"HFL:", 6, 12);
		oled_showNum(27, 15, Car.Sensor[SENSOR_H_L].Average, 3, 6, 12);
		
		/*  前排右边水平电感  */
		oled_showString(65,15,"HFR:", 6, 12);
		oled_showNum(95, 15, Car.Sensor[SENSOR_H_R].Average, 3, 6, 12);
		
		oled_showString(3,33,"HBL:", 6, 12);
		oled_showNum(27, 33, Car.Sensor[SENSOR_V_L].Average, 3, 6, 12);
		
		oled_showString(65,33,"HBR:", 6, 12);
		oled_showNum(95, 33, Car.Sensor[SENSOR_V_R].Average, 3, 6, 12);
		
		oled_showString(3,50,"HAE:", 6, 12);
		if(Car.HorizontalAE < 0)
		{
			oled_showChar(27, 50, '-', 6, 12, 1);
			oled_showNum(33, 50, (int16_t)(-Car.HorizontalAE), 3, 6, 12);
		}else
			oled_showNum(27, 50, (int16_t)(Car.HorizontalAE), 3, 6, 12);
		
		oled_showString(65,50,"VAE:", 6, 12);
		if(Car.VecticalAE < 0)
		{
			oled_showChar(90, 50, '-', 6, 12, 1);
			oled_showNum(90+6, 50, (int16_t)(-Car.VecticalAE), 3, 6, 12);
		}else
		oled_showNum(90, 50, (int16_t)(Car.VecticalAE), 3, 6, 12);
	}
	/*  第二页显示电机参数  */
	else if(showPage == 1)
	{
		/*  电机编码器  */
		oled_showString(3, 15, "L_E:", 6, 12);
		oled_showNum(27, 15, Car.Motor.LeftEncoder, 4, 6, 12);
		
		oled_showString(55, 15, "R_E:", 6, 12);
		oled_showNum(85, 15, Car.Motor.RightEncoder, 4, 6, 12);
		
		oled_showString(3, 33, "L_P:", 6, 12);
		if(Car.Motor.LeftPwm<0)
		{
			oled_showChar(27, 50, '-', 6, 12, 1);
			oled_showNum(33, 50, -Car.Motor.LeftPwm, 3, 6, 12);
		}else
			oled_showNum(27, 33, Car.Motor.LeftPwm, 4, 6, 12);
		
		oled_showString(55, 33, "R_P:", 6, 12);
		if(Car.Motor.RightPwm<0)
		{
			oled_showChar(85, 50, '-', 6, 12, 1);
			oled_showNum(91, 50, -Car.Motor.RightPwm, 3, 6, 12);
		}else
			oled_showNum(85, 33, Car.Motor.RightPwm, 4, 6, 12);
		
		oled_showString(3,50,"Nor:", 6, 12);
		oled_showNum(27, 50, (Car.Sensor[0].CalibrationMax), 3, 6, 12);
		
		oled_showString(55,50,"Nor:", 6, 12);
		oled_showNum(85, 50, (Car.Sensor[0].CalibrationMax), 3, 6, 12);
	}
	/*  第三页显示PID参数  */
	else if(showPage == 2)
	{
		oled_showString(3, 15, "SKP:", 6, 12);
		oled_showNum(27, 15, Car.PID.SpeedKp*10, 4, 6, 12);
		
		oled_showString(55, 15, "SKI:", 6, 12);
		oled_showNum(85, 15, Car.PID.SpeedKi*10, 4, 6, 12);
		
		oled_showString(3, 33, "DKP:", 6, 12);
		oled_showNum(27, 33, Car.PID.DirectionKp, 4, 6, 12);
		
		oled_showString(55, 33, "DKD:", 6, 12);
		oled_showNum(85, 33, Car.PID.DirectionKd, 4, 6, 12);
		
		oled_showString(3,50,"LT:", 6, 12);
		oled_showNum(27, 50, (Car.LeftTargetSpeed), 3, 6, 12);
		
		oled_showString(55,50,"RT:", 6, 12);
		oled_showNum(85, 50, (Car.RightTargetSpeed), 3, 6, 12);
	}
	
	if(key == KEY_DOWN_PRESS)
	{
		showPage--;
		isWindowChange = true;
		if(showPage < 0)
			showPage = 2;
		switch(showPage)
		{
			case 0: DebugWindow.title = "Motor Parameters";break;
			case 1: DebugWindow.title = "Sensor Parameters";break;
			case 2: DebugWindow.title = "PID Parameters";
			default: break;
		}
	}else if(key == KEY_UP_PRESS)
	{
		showPage++;
		isWindowChange = true;
		if(showPage>2)
			showPage = 0;
		
		switch(showPage)
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


