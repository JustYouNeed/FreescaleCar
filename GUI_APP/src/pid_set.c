# include "pid_set.h"
#include "windows.h"
# include "button.h"
# include "bsp_oled.h"
# include "bsp_timer.h"
# include "bsp_led.h"
# include "bsp_key.h"
# include "stdio.h"
# include "FreescaleCar.h"
# include "text.h"
# include "menu.h"
# include "display.h"
#include "scrollbar.h"

extern int cur_sequence;
extern WINDOWS MenuWindow;
extern Scrollbar_Typedef MenuScrollbar;
extern MenuItem_Typedef*  CurMenu;
extern MenuItem_Typedef MainMenu[5];
extern MenuItem_Typedef PIDAdjMenu[9];
extern uint8_t key;
extern bool isChangeMenu;
extern int selected;
extern MenuItem_Typedef*  CurItem;
extern uint8_t getMenuSelectitem(MenuItem_Typedef menu[]);


//uint8_t PID_SetSelected = 0;

Button_Typedef bt_add={
4,
46,
36,
14,
"+",
1
};

Button_Typedef bt_mines={
87,
46,
36,
14,
"-",
1
};

Button_Typedef bt_ok={
46,
46,
36,
14,
"OK",
1
};



/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void SpeedKp_Set(void)
{
	static uint16_t cnt = 0;
	static float kp_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Speed Kp";
		kp_temp = (Car.VelPID.Kp);
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		kp_temp += 0.5;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		kp_temp -= 0.5;
	}else if(key == KEY_OK_PRESS)
	{
		Car.VelPID.Kp = kp_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Kp:",12,24);
	oled_showNum(58,16,(uint32_t)(kp_temp*10), 4,12,24);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void SpeedKi_Set(void)
{
	static uint16_t cnt = 0;
	static float ki_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Speed Ki";
		ki_temp = (Car.VelPID.Ki);
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		ki_temp += 0.5;
	}	else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		ki_temp -= 0.5;
	}	else if(key == KEY_OK_PRESS)
	{
		Car.VelPID.Ki = ki_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Ki:",12,24);
	oled_showNum(58,16,(uint32_t)(ki_temp*10), 4,12,24);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void SpeedKd_Set(void)
{
	static uint16_t cnt = 0;
	static float kd_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Speed Kd";
		kd_temp = (Car.VelPID.Kd);
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		kd_temp += 0.5;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		kd_temp -= 0.5;
	}else if(key == KEY_OK_PRESS)
	{
		Car.VelPID.Kd = kd_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		Car_ControlStart();
		setShow_ui(MENU_UI);
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Kd:",12,24);
	oled_showNum(58,16,(uint32_t)(kd_temp*10), 4, 12, 24);
}
/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void DirectionKp_Set(void)
{
	static uint16_t cnt = 0;
	static float kp_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Direction Kp";
		kp_temp = (Car.DirFuzzy.KPMax);
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		kp_temp += 0.5;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		kp_temp -= 0.5;
	}else if(key == KEY_OK_PRESS)
	{
		Car.DirFuzzy.KPMax = kp_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Kp:",12,24);
	oled_showNum(58,16,(uint32_t)(kp_temp*10), 4,12,24);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void DirectionKi_Set(void)
{
	static uint16_t cnt = 0;
	static float ki_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Direction Ki";
		ki_temp = Car.DirFuzzy.KIMax;
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		ki_temp += 0.5;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		ki_temp -= 0.5;
	}else if(key == KEY_OK_PRESS)
	{
		Car.DirFuzzy.KIMax = ki_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Ki:",12,24);
	oled_showNum(58,16,(uint32_t)(ki_temp*10), 4,12,24);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void DirectionKd_Set(void)
{
	static uint16_t cnt = 0;
	static float kd_temp;
	
	if(cnt == 0)
	{
		MenuWindow.title = "Direction Kd";
		kd_temp = Car.DirFuzzy.KDMax;
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		kd_temp += 5;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		kd_temp -= 5;
	}else if(key == KEY_OK_PRESS)
	{
		Car.DirFuzzy.KDMax = kd_temp;
		pid_StorePara();	/*  将PID参数保存到Flash中  */
		cnt = 0;
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(20,16,"Kd:",12,24);
	oled_showNum(58,16,(uint32_t)(kd_temp), 4,12,24);
}


void PID_ParaSet(void)
{
	
}


void Car_SetTarSpeed(void)
{
	static uint16_t cnt = 0;
	static float speed = 0;;

	if(cnt == 0)
	{
		MenuWindow.title = "Set TargetSpped";
		speed = Car.TargetSpeed;
		GUI_WindowsDraw(&MenuWindow);
		GUI_DrawButton(&bt_add);
		GUI_DrawButton(&bt_mines);
		GUI_DrawButton(&bt_ok);
		cnt = 1;
	}
		
	if(key == KEY_UP_PRESS)
	{
		GUI_Button_Selected(&bt_add, 0);
		speed += 1;
	}else if(KEY_DOWN_PRESS == key)
	{
		GUI_Button_Selected(&bt_mines, 0);
		speed -= 1;
	}else if(key == KEY_OK_PRESS)
	{
		Car.TargetSpeed = speed;

		cnt = 0;
		Car_ParaStore();
		displayInit();
		setShow_ui(MENU_UI);
		Car_ControlStart();
	}else
	{
		GUI_Button_Selected(&bt_add, 1);
		GUI_Button_Selected(&bt_mines, 1);
	}	
	oled_showString(10,16,"Speed:",12,24);
	oled_showNum(88,16,(speed), 2,12,24);
}



