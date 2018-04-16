#include "display.h"
# include "main_ui.h"
#include "menu.h"
# include "pid_set.h"
# include "para_show.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly_Remotor
 * 界面显示代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static enum ui_e show_ui = MAIN_UI;
extern uint8_t isMainUIChange;
uint8_t key = KEY_NONE;
/*设置显示界面*/
void setShow_ui(enum ui_e ui)
{
	show_ui = ui;
	if(ui == MAIN_UI) isMainUIChange = true;
	GUI_ClearSCR();
}

/*显示任务*/
void displayTask(void)
{	
	key = bsp_key_GetKey();
	if(key !=KEY_NONE  && show_ui == MAIN_UI) 
	{
		setShow_ui(MENU_UI);
		key = KEY_NONE;
	}
	switch(show_ui)
	{
		case DEBUG_UI: Para_Show_UI(); break;
		case MAIN_UI:	main_ui(); break;
		case MENU_UI:	Menu_Run();	break;
		case SET_S_KP_UI:	SpeedKp_Set(); break;
		case SET_S_KI_UI:	SpeedKi_Set(); break;
		case SET_D_KP_UI: DirectionKp_Set(); break;
		case SET_D_KD_UI: DirectionKd_Set(); break;
		case SET_TAR_SPEED_UI: Car_SetTarSpeed();break;
		case RESET_UI:SystemReset();break;
		default:break;
	}
	
	GUI_Refresh();
}

/*界面显示初始化*/
void displayInit(void)
{
	//菜单初始化
	MainMenu_Init();
	ParaSetMenu_Init();
	PIDAdjMenu_Init();
}



