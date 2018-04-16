/**
  *******************************************************************************************************
  * File Name: app_pid.h
  * Author: Vector
  * Version: V1.2.0
  * Date: 2018-2-1
  * Brief: 本文件提供了关于PID函数的声明,以及相关变量的定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-1
	*			Mod: 建立本文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 修改PID结体变量名字,删除不必要的变量,Error, Sum等
  *
  *******************************************************************************************************
  */	
	
# ifndef __APP_PID_H
# define __APP_PID_H

/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "bsp.h"
	
typedef struct
{
	float SpeedKp, SpeedKi, SpeedKd;
	float DirectionKp, DirectionKi, DirectionKd;
}PID_TypeDef;


void pid_ParaInit(void);
void pid_ReadPara(void);
void pid_StorePara(void);

# endif
	
	
/********************************************  END OF FILE  *******************************************/
	

