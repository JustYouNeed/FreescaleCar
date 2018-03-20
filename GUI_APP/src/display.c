#include "display.h"
#include "menu.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ������ʾ����	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static enum ui_e show_ui = MAIN_UI;

/*������ʾ����*/
void setShow_ui(enum ui_e ui)
{
	show_ui = ui;
	GUI_ClearSCR();
}

/*��ʾ����*/
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

/*������ʾ��ʼ��*/
void displayInit(void)
{
	//�˵���ʼ��
	MainMenu_Init();
	ParaSetMenu_Init();
	PIDAdjMenu_Init();
}



