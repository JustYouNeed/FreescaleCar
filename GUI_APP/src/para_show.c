/**
  *******************************************************************************************************
  * File Name: para_show.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-24
  * Brief: 用于显示小车的各种参数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-24
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
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

/*  定义一个窗口  */
WINDOWS DebugWindow={
.x = 0,
.y = 0,	
.width = 128,
.height = 64,
.itemsperpage = 3,
.topitem = 0,
.title = "Sensor Parameters",
};

/*  定义一个滚动条  */
Scrollbar_Typedef DebugScrollbar={
.x = 118,
.y = 14,
.width = 10,
.height = 50,
.itemsperpage = 3,
.topitem = 0,
.scbbarlen = 0,
};

/*
*********************************************************************************************************
*                             Para_ShowRefresh             
*
* Description: 用于改变参数显示后刷新窗口标题
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Para_ShowRefresh(void)
{
	if(isWindowChange == false) return;		/*  只有在窗口被改变的情况下才需要刷新  */
	GUI_WindowsDraw(&DebugWindow);
	GUI_ScrollbarDraw(&DebugScrollbar);
	isWindowChange = false;
}

/*
*********************************************************************************************************
*                             Para_Show_UI             
*
* Description: 用于显示调试期间的各种参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Para_Show_UI(void)
{
	int32_t ten = 0, dot = 0;
	
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
		
		oled_showString(3,27,"HBL:", 6, 12);
		oled_showNum(27, 27, Car.Sensor[SENSOR_V_L].Average, 3, 6, 12);
		
		oled_showString(65,27,"HBR:", 6, 12);
		oled_showNum(95, 27, Car.Sensor[SENSOR_V_R].Average, 3, 6, 12);
		
		oled_showString(3,39,"HAE:", 6, 12);
		if(Car.HorizontalAE < 0)
		{
			oled_showChar(27, 39, '-', 6, 12, 1);
			oled_showNum(33, 39, (int16_t)(-Car.HorizontalAE), 3, 6, 12);
		}else
			oled_showNum(27, 39, (int16_t)(Car.HorizontalAE), 4, 6, 12);
		
		oled_showString(65,39,"VAE:", 6, 12);
		if(Car.VecticalAE < 0)
		{
			oled_showChar(90, 39, '-', 6, 12, 1);
			oled_showNum(90+6, 39, (int16_t)(-Car.VecticalAE), 3, 6, 12);
		}else
		oled_showNum(90, 39, (int16_t)(Car.VecticalAE), 4, 6, 12);
		
		oled_showString(3,51,"HM:", 6, 12);
		oled_showNum(27, 51, (int16_t)(Car.Sensor[SENSOR_M].Average), 3, 6, 12);
		
		oled_showString(65,51,"AE:", 6, 12);
		if(Car.AE < 0)
		{
			oled_showChar(90, 51, '-', 6, 12, 1);
			oled_showNum(90+6, 51, (int16_t)(-Car.AE), 3, 6, 12);
		}else
		{
			oled_showChar(90, 51, ' ', 6, 12, 1);
			oled_showNum(90+6, 51, (int16_t)(Car.AE), 3, 6, 12);
		}
	}
	/*  第二页显示电机参数  */
	else if(showPage == 1)
	{
		/*  电机编码器  */
		oled_showString(3, 15, "LE:", 6, 12);
		if(Car.Motor.LeftEncoder < 0)
		{
			oled_showChar(21, 15, '-', 6, 12, 1);
			oled_showNum(27, 15, -Car.Motor.LeftEncoder, 4, 6, 12);
		}else
		{
			oled_showChar(21, 15, ' ', 6, 12, 1);
			oled_showNum(27, 15, Car.Motor.LeftEncoder, 4, 6, 12);
		}
		
		oled_showString(61, 15, "RE:", 6, 12);
		if(Car.Motor.RightEncoder < 0)
		{
			oled_showChar(79, 15, '-', 6, 12, 1);
			oled_showNum(85, 15, -Car.Motor.RightEncoder, 4, 6, 12);
		}
		else
		{
			oled_showChar(79, 15, ' ', 6, 12, 1);
			oled_showNum(85, 15, Car.Motor.RightEncoder, 4, 6, 12);
		}
		
		oled_showString(3, 33, "LP:", 6, 12);
		if(Car.Motor.LeftPwm<0)
		{
			oled_showChar(21, 50, '-', 6, 12, 1);
			oled_showNum(37, 50, -Car.Motor.LeftPwm, 4, 6, 12);
		}else
		{
			oled_showChar(21, 50, ' ', 6, 12, 1);
			oled_showNum(27, 33, Car.Motor.LeftPwm, 4, 6, 12);
		}
		
		oled_showString(61, 33, "RP:", 6, 12);
		if(Car.Motor.RightPwm<0)
		{
			oled_showChar(79, 50, '-', 6, 12, 1);
			oled_showNum(85, 50, -Car.Motor.RightPwm, 4, 6, 12);
		}else
		{
			oled_showChar(79, 33, ' ', 6, 12, 1);
			oled_showNum(85, 33, Car.Motor.RightPwm, 4, 6, 12);
		}
		
		ten = (int32_t)Car.CarSpeed;
		dot = ((float)(Car.CarSpeed - ten * 10)) * 100;
		
		oled_showString(3, 51, "Speed:", 6, 12);
		if(Car.CarSpeed < 0)
		{
			ten = -ten;
			dot = - dot;
			oled_showChar(39, 51, '-', 6, 12, 1);
			oled_showNum(45, 51, ten, 2, 6, 12);
			oled_showChar(57, 51, '.', 6, 12, 1);
			oled_showNum(63, 51, dot, 2, 6, 12);
		}
		else
		{
			oled_showChar(39, 51, ' ', 6, 12, 1);
			oled_showNum(45, 51, ten, 2, 6, 12);
			oled_showChar(57, 51, '.', 6, 12, 1);
			oled_showNum(63, 51, dot, 2, 6, 12);
		}
		oled_showString(78, 51, "R/S", 6, 12);
	}
	/*  第三页显示PID参数  */
	else if(showPage == 2)
	{
		oled_showString(3, 15, "SKP:", 6, 12);
		oled_showNum(27, 15, Car.VelPID.Kp*10, 4, 6, 12);
		
		oled_showString(55, 15, "SKI:", 6, 12);
		oled_showNum(85, 15, Car.VelPID.Ki*10, 4, 6, 12);
		
		oled_showString(3, 33, "DKP:", 6, 12);
		oled_showNum(27, 33, Car.DirFuzzy.KPMax, 4, 6, 12);
		
		oled_showString(55, 33, "DKD:", 6, 12);
		oled_showNum(85, 33, Car.DirFuzzy.KDMax, 4, 6, 12);
		
				oled_showString(3,50,"LT:", 6, 12);
		oled_showNum(27, 50, (Car.LeftTargetSpeed), 3, 6, 12);
		
		oled_showString(55,50,"RT:", 6, 12);
		oled_showNum(85, 50, (Car.RightTargetSpeed), 3, 6, 12);
		
	} 
	else if(showPage == 3)
	{
		oled_showString(20,20,"AE:",6, 12);
		if(Car.AE < 0)
		{
			oled_showChar(38, 20, '-', 6, 12, 1);
			oled_showNum(44, 20, -Car.AE, 2, 6, 12);
		}
		else
		{
			oled_showChar(38, 20, ' ', 6, 12, 1);
			oled_showNum(44, 20, Car.AE, 2, 6, 12);
		}
		oled_showNum(30,30, Car.NowRoad, 1, 12, 24);
	}
	
	/*  进行翻页  */
	if(key == KEY_DOWN_PRESS)
	{
		showPage--;
		isWindowChange = true;
		if(showPage < 0)
			showPage = 3;
		switch(showPage)
		{
			case 1: DebugWindow.title = "Motor Parameters";break;
			case 0: DebugWindow.title = "Sensor Parameters";break;
			case 2: DebugWindow.title = "PID Parameters";break;
			case 3: DebugWindow.title = "Road Detect";break;
			default: break;
		}
	}else if(key == KEY_UP_PRESS)
	{
		showPage++;
		isWindowChange = true;
		if(showPage>3)
			showPage = 0;
		
		switch(showPage)
		{
			case 1: DebugWindow.title = "Motor Parameters";break;
			case 0: DebugWindow.title = "Sensor Parameters";break;
			case 2: DebugWindow.title = "PID Parameters"; break;
			case 3: DebugWindow.title = "Road Detect";break;
			default: break;
		}
	}
	else	if(key == KEY_OK_PRESS)
	{
		isWindowChange = true;
		setShow_ui(MAIN_UI);
	}

}

/********************************************  END OF FILE  *******************************************/


