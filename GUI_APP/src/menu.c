# include "menu.h"
#include "windows.h"
#include "scrollbar.h"
# include "display.h"
#include "gui_basic.h"
# include "pid_set.h"
# include "stdio.h"
/****** 手指按钮图标 阴码 逐列式 顺向(高位在前) *****/
const uint8_t finger_img[32]={
0x07,0xF0,0x04,0x10,0x04,0x10,0x08,0x08,0x08,0x08,0x08,0x08,0x10,0x04,0x20,0x04,
0x41,0x54,0x4D,0x5C,0x75,0x70,0x05,0xC0,0x05,0x00,0x05,0x00,0x05,0x00,0x02,0x00,
};


MenuItem_Typedef MainMenu[5];
MenuItem_Typedef ParaAdjMenu[5];
MenuItem_Typedef ParaViewMenu[5];
MenuItem_Typedef PIDAdjMenu[9];
static MenuItem_Typedef DirctionPIDAdjMenu[4];

MenuItem_Typedef*  CurMenu = MainMenu;
static MenuItem_Typedef*  CurItem;

const uint8_t *PIDParaAdjMenuTitle[7] = 
{
	"1.SpeedKp",
	"2.SpeedKi",
	"3.SpeedKd",
	"4.DirctionKp",
	"5.DirctionKi",
	"6.DirctionKd",
	"7.Exit",
};
const uint8_t* MainMenuTitle[5]=
{
	"1.Para View",
	"2.Para Adjust",
	"3.Calibration",
	"4.Reset",
	"5.Exit",
};

const uint8_t *ParaAdjMenuTitle[3] = 
{
	"1.PID",
	"2.TargetSpeed",
	"3.Exit",
};



//菜单的位置
#define   MEMU_POSX_1    30
#define   MEMU_POSY_1    19



static int selected;
int cur_sequence; //某一级菜单选中项目的序号
static bool isChangeMenu = true;
const uint8_t* defaultTitle = "Vector Car";

//窗体
WINDOWS MenuWindow={
.x = 0,
.y = 0,	
.width = 128,
.height = 64,
.itemsperpage = 3,
.topitem = 0,
.title = "Vector",
};

//主窗体滚动条
Scrollbar_Typedef MenuScrollbar={
.x = 118,
.y = 14,
.width = 10,
.height = 50,
.itemsperpage = 3,
.topitem = 0,
.scbbarlen = 0,
};

uint8_t getMenuSelectitem(MenuItem_Typedef menu[])
{
	int i;
	for(i=0; i< menu->menuItemCount; i++)
	{
		if(menu[i].isSelect == true) 
			break;
	}
	return i;
}

void exitMenu(void)
{
	CurMenu = MainMenu;
	MenuWindow.title = defaultTitle;
	setShow_ui(MAIN_UI);
	isChangeMenu = true;
}

void gotoNextMenu(void)	
{
	if(CurItem->childrenMenu != NULL)
	{
		CurMenu = CurItem->childrenMenu;
		MenuWindow.title = CurItem->title +2;
		isChangeMenu = true;
	}
	else if(CurItem->Function!=NULL && CurItem->parentMenu==NULL)
	{
		CurItem->Function();
	}
}

void gotoLastMenu(void)
{
	if(CurItem->parentMenu != NULL)
	{
		//恢复默认选中
		CurItem->isSelect = false;
		CurItem = CurMenu + selected;
		CurItem->isSelect = true;
		
		CurMenu = CurItem->parentMenu;
		if(CurMenu == MainMenu)
			MenuWindow.title = defaultTitle;
		else
			MenuWindow.title = CurItem->title +2;
		isChangeMenu = true;
	}
	else
	{
		exitMenu();
	}
}

/*显示菜单*/
void DisplayMenuInit(MenuItem_Typedef* menu)
{
	uint8_t topitem;
	uint8_t showItems;
	
	if(isChangeMenu == false) return;
	
	selected = getMenuSelectitem(menu);
	cur_sequence = selected;
	if(selected < menu->cursorPosition)
		menu->cursorPosition = 0;
	topitem = selected - menu->cursorPosition;
	if(menu->menuItemCount <= MenuWindow.itemsperpage)
		topitem = 0;

	MenuWindow.topitem = topitem;
	GUI_WindowsDraw(&MenuWindow);
	
	MenuScrollbar.topitem = topitem;
	MenuScrollbar.totalitems = menu->menuItemCount;
	GUI_ScrollbarDraw(&MenuScrollbar);
	
	showItems = MenuWindow.itemsperpage;
	if(menu->menuItemCount < MenuWindow.itemsperpage)
		showItems = menu->menuItemCount;
	for(int i=0; i<showItems; i++)
	{
		MenuItem_Typedef* Item = &menu[topitem+i];
		GUI_MenuItemDraw(MEMU_POSX_1, MEMU_POSY_1+i*15, Item);
	}
	CurItem = menu + cur_sequence;
	CurItem->isSelect = true;
	
	GUI_Refresh();//刷新屏幕
	
	isChangeMenu = false;
}

/*菜单运行*/
void Menu_Run(void)
{
	uint8_t key=0;
	static uint8_t once = 0;
	static int timeout;
	volatile uint8_t showItems = 0;
	
	DisplayMenuInit(CurMenu);
	key = bsp_key_GetKey();
	
	if(key != KEY_NONE) timeout = 0;
	switch(key)
	{
			case KEY_DOWN_PRESS:	//PITCH向后
				//清除窗口内容
				GUI_RectangleFill(1, 18, 117, 62, 0);
				CurItem = CurMenu + cur_sequence;
				CurItem->isSelect = false;
				bsp_led_Toggle(2);
				//菜单项目序号++
				cur_sequence++;
				if(cur_sequence >= CurMenu->menuItemCount-1)
					cur_sequence = CurMenu->menuItemCount-1;
				CurItem = CurMenu + cur_sequence;
				CurItem->isSelect = true;
				
				//光标位置++
				CurMenu->cursorPosition++;
				if(CurMenu->menuItemCount <= MenuWindow.itemsperpage)
				{
					showItems = CurMenu->menuItemCount;
					if(CurMenu->cursorPosition >= CurMenu->menuItemCount)
						CurMenu->cursorPosition = CurMenu->menuItemCount-1;
				}
				else
				{
					showItems = MenuWindow.itemsperpage;
					if(CurMenu->cursorPosition > MenuWindow.itemsperpage-1)
					{		
						CurMenu->cursorPosition= MenuWindow.itemsperpage-1;
						if(MenuWindow.topitem < CurMenu->menuItemCount-MenuWindow.itemsperpage)
							MenuWindow.topitem++;
					}
				}
				for(int i=0; i<showItems; i++)
				{
					MenuItem_Typedef* Item = CurMenu+MenuWindow.topitem+i;
					GUI_MenuItemDraw(MEMU_POSX_1, MEMU_POSY_1+i*15, Item);
				}     	
			MenuScrollbar.topitem = cur_sequence;
			GUI_Scrollbar_SetPos(&MenuScrollbar);	 				
				break;
					
		case KEY_UP_PRESS:	//PITCH向前
			//清除窗口内容
			GUI_RectangleFill(1, 18, 117, 62, 0);
			CurItem = CurMenu + cur_sequence;
			CurItem->isSelect = false;
		
			//菜单项目序号--
			cur_sequence--;
			if(cur_sequence <= 0)
				cur_sequence = 0;
			CurItem = CurMenu + cur_sequence;
			CurItem->isSelect = true;
			
			//光标位置--
			CurMenu->cursorPosition--;
			if(CurMenu->menuItemCount <= MenuWindow.itemsperpage)
			{	
				showItems = CurMenu->menuItemCount;
				if(CurMenu->cursorPosition < 0)
					CurMenu->cursorPosition = 0;
			}
			else
			{	
				showItems = MenuWindow.itemsperpage;				
				if(CurMenu->cursorPosition < 0)
				{		
					CurMenu->cursorPosition = 0;
					if(MenuWindow.topitem > 0)
						MenuWindow.topitem--;
				}
			}
			for(int i=0; i<showItems; i++)
			{
				MenuItem_Typedef* Item = CurMenu+MenuWindow.topitem+i;
				GUI_MenuItemDraw(MEMU_POSX_1, MEMU_POSY_1+i*15, Item);
			}  					
			MenuScrollbar.topitem = cur_sequence;
			GUI_Scrollbar_SetPos(&MenuScrollbar);   
			break;	
		default :break;
	}
	
	
	if(key == KEY_OK_PRESS)
	{
		once ++;
		bsp_led_Toggle(1);
		if(CurItem->Function != NULL && once > 1)
		{
			key = KEY_NONE;
			CurItem->Function();
		}
	}
	
	if(timeout ++ > 80)
	{
		timeout = 0;
		if(CurItem->parentMenu != NULL)
		{
			//恢复默认选中
			CurItem->isSelect = false;
			CurItem = CurMenu + selected;
			CurItem->isSelect = true;
		}
		if(CurMenu != MainMenu)
			exitMenu();
	}
	
	GUI_Refresh();
}


void MainMenu_Init(void)
{
	uint8_t i = 0;
	/*  主菜单初始化  */
	for(i=0; i<5; i++)
	{
		MainMenu[i].menuItemCount = 5;
		MainMenu[i].isSelect = false;
		MainMenu[i].icoSelected = finger_img;
		MainMenu[i].icoUnselected = NULL;
		MainMenu[i].title = MainMenuTitle[i];
		MainMenu[i].Function = NULL;
		MainMenu[i].parentMenu = NULL;
		MainMenu[i].childrenMenu = NULL;
	}
	MainMenu[0].isSelect = true;
	
	MainMenu[1].Function = gotoNextMenu;
	MainMenu[1].childrenMenu = ParaAdjMenu;
}

void PIDAdjMenu_Init(void)
{
	uint8_t i = 0;
	
	for(i = 0; i < 7; i++)
	{
		PIDAdjMenu[i].childrenMenu = NULL;
		PIDAdjMenu[i].Function = NULL;
		PIDAdjMenu[i].icoSelected = finger_img;
		PIDAdjMenu[i].icoUnselected = NULL;
		PIDAdjMenu[i].isSelect = false;
		PIDAdjMenu[i].menuItemCount = 7;
		PIDAdjMenu[i].parentMenu = NULL;
		PIDAdjMenu[i].title = PIDParaAdjMenuTitle[i];
	}

	PIDAdjMenu[0].isSelect = true;
	PIDAdjMenu[0].Function = Kp;
	
	PIDAdjMenu[6].childrenMenu = ParaAdjMenu;
	PIDAdjMenu[6].Function = gotoNextMenu;
}

void ParaSetMenu_Init(void)
{
	uint8_t i = 0;
	/*  参数调节菜单初始化  */
	for(i = 0; i< 3; i++)
	{
		ParaAdjMenu[i].menuItemCount = 3;
		ParaAdjMenu[i].childrenMenu = NULL;
		ParaAdjMenu[i].Function = NULL;
		ParaAdjMenu[i].icoSelected = finger_img;
		ParaAdjMenu[i].icoUnselected = NULL;
		ParaAdjMenu[i].isSelect = false;
		ParaAdjMenu[i].parentMenu = NULL;
		ParaAdjMenu[i].title = ParaAdjMenuTitle[i];
	}
	
	ParaAdjMenu[0].isSelect = true;
	ParaAdjMenu[0].Function = gotoNextMenu;
	ParaAdjMenu[0].childrenMenu = PIDAdjMenu;
	
	ParaAdjMenu[2].childrenMenu = MainMenu;
	ParaAdjMenu[2].Function = gotoNextMenu;
}