# ifndef __MENU_H
# define __MENU_H

# include "gui_menu.h"
# include "lcmdrv.h"
# include "FreescaleCar.h"

void MainMenu_Init(void);
void ParaSetMenu_Init(void);
void PIDAdjMenu_Init(void);
void DisplayMenuInit(MenuItem_Typedef* menu);
void menu_Init(void);
void MenuFresh(void);
void Menu_Run(void);//运行菜单
void MenuHookFun(uint8_t key);//钩子函数
void exitMenu(void);
void gotoNextMenu(void);
void gotoLastMenu(void);
void SystemReset(void);
# endif

