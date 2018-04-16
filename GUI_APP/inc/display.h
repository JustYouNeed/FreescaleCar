#ifndef __DISPLAY_H
#define __DISPLAY_H
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

//��������
enum ui_e
{
	MAIN_UI,
	TRIM_UI,
	MENU_UI,
	DEBUG_UI,
	SET_S_KP_UI,
	SET_S_KI_UI,
	SET_D_KP_UI,
	SET_D_KD_UI,
	SET_TAR_SPEED_UI,
	RESET_UI,
};

void setShow_ui(enum ui_e ui);
void displayTask(void);
void displayInit(void);


#endif /*__DISPLAY_H*/
