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

uint8_t setkp = 0;
void Kp()
{
	setkp = 1;
}

void exitPIDSet(void)
{
//	setShow_ui(MAIN_UI);
	GUI_ClearSCR();
	displayInit();
	MenuWindow.topitem = 0;
	GUI_WindowsDraw(&MenuWindow);
	MenuScrollbar.topitem = 1;
	MenuScrollbar.totalitems = PIDAdjMenu->menuItemCount;
	GUI_ScrollbarDraw(&MenuScrollbar);
	
//	PIDAdjMenu[0].isSelect = true;
//	for(int i = 0; i< PIDAdjMenu[0].menuItemCount; i++)
//	{
////		if(i!=0)PIDAdjMenu[i].isSelect = false;
//		GUI_MenuItemDraw(30,19,&PIDAdjMenu[i]);
//	}
	bsp_key_ClearFIFO();
	GUI_Refresh();
}

void SpeedKp_Set(void)
{
	uint16_t cnt = 0;
	uint16_t keytime = 0;
	uint8_t Kp[128];
	float kp_temp = (Car.PID.Kp_Straight);
	MenuWindow.title = "Speed Kp";

	GUI_WindowsDraw(&MenuWindow);
	GUI_DrawButton(&bt_add);
	GUI_DrawButton(&bt_mines);
	GUI_DrawButton(&bt_ok);
	
	GUI_Refresh();
//	DISABLE_INT();
	
	setkp = 0;
	drv_gpio_WritePin(KEY_OK_PIN, 1);
	while(drv_gpio_ReadPin(KEY_OK_PIN) == 1)
	{

		if(drv_gpio_ReadPin(KEY_UP_PIN) == 0)
		{
			GUI_Button_Selected(&bt_add, 0);
			bsp_led_Toggle(2);
			GUI_Refresh();
			while(drv_gpio_ReadPin(KEY_UP_PIN) == 0);
			kp_temp += 0.5;
		}
		else
		{
			GUI_Button_Selected(&bt_add, 1);
		}
		
		if(drv_gpio_ReadPin(KEY_DOWN_PIN) == 0)
		{
			GUI_Button_Selected(&bt_mines, 0);
			bsp_led_Toggle(2);
			GUI_Refresh();
			while(drv_gpio_ReadPin(KEY_DOWN_PIN) == 0);
			kp_temp -= 0.5;
//			sprintf(Kp, "Kp:%lf",kp_temp);
		}
		else
		{
			GUI_Button_Selected(&bt_mines, 1);
		}
		
//		if(drv_gpio_ReadPin(KEY_OK_PIN) == 0)
//		{
//			keytime++;
//			GUI_Button_Selected(&bt_ok, 0);
//			GUI_Refresh();
//			while(drv_gpio_ReadPin(KEY_OK_PIN) == 0);
////			PIDAdjMenu[0].isSelect = false;
////			cur_sequence = 0;
////			bsp_key_ClearFIFO();
//			
////		
//			
////			drv_gpio_WritePin(KEY_OK_PIN, 1);
////			
//			return;
//		}
//		else
//		{
//			keytime = 0;
//			GUI_Button_Selected(&bt_ok, 1);
//		}

		cnt++;
		if(cnt % 20 == 0)
		{
			bsp_led_Toggle(1);
			oled_showString(20,16,"Kp:",12,24);
			oled_showNum(58,16,(uint32_t)(kp_temp*100), 4,12,24);
			GUI_Refresh();
		}
	}
	Car.PID.Kp_Straight = kp_temp;
	exitPIDSet();
}

void SpeedKi_Set(void);
void SpeedKd_Set(void);
void DirctionKp_Set(void);
void DirctionKi_Set(void);
void DirctionKd_Set(void);
