#include "display.h"
#include "menu.h"

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

/*设置显示界面*/
void setShow_ui(enum ui_e ui)
{
	show_ui = ui;
	GUI_ClearSCR();
}

/*显示任务*/
void displayTask(void)
{
	while(1)
	{
		switch(show_ui)
		{
			case MAIN_UI:
				break;
			case TRIM_UI:
				break;
			case MENU_UI:
				break;
			case DEBUG_UI:
				break;
			case JOYSTICK_CALIB_UI:
				break;
			case MATCH_UI:
				break;
			case RESET_UI:
				break;
			default:break;
		}
		GUI_Refresh();
	}
}

/*界面显示初始化*/
void displayInit(void)
{
	//菜单初始化
	MainMenu_Init();
	ParaSetMenu_Init();
	PIDAdjMenu_Init();
}



