#ifndef __DISPLAY_H
#define __DISPLAY_H
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

//界面类型
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
