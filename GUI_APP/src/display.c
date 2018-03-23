#include "display.h"
# include "main_ui.h"
#include "menu.h"
# include "pid_set.h"
# include "para_show.h"
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
extern uint8_t isMainUIChange;
uint8_t key = KEY_NONE;
/*������ʾ����*/
void setShow_ui(enum ui_e ui)
{
	show_ui = ui;
	if(ui == MAIN_UI) isMainUIChange = true;
	GUI_ClearSCR();
}

/*��ʾ����*/
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
		case SET_S_KD_UI:	SpeedKd_Set(); break;
		case SET_D_KP_UI: DirctionKp_Set(); break;
		case SET_D_KI_UI: DirctionKi_Set(); break;
		case SET_D_KD_UI: DirctionKd_Set(); break;
		default:break;
	}
	
	GUI_Refresh();
}

/*������ʾ��ʼ��*/
void displayInit(void)
{
	//�˵���ʼ��
	MainMenu_Init();
	ParaSetMenu_Init();
	PIDAdjMenu_Init();
}



