/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
	
# ifndef __SYSTEM_H
# define __SYSTEM_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "app_pid.h"


typedef enum
{
	TurnLeft = 0x0,
	TurnRight,
}Turn_TypeDef;

/*  道路情况枚举变量  */
typedef enum
{
	Road_Straight = 0x0,		/*  直道  */
	Road_SmallCurve,				/*  小弯  */
	Road_BigCurve,					/*  大弯  */
	Road_Island,						/*  环岛  */
	Road_Circle,						/*  圆环  */
	Road_Ramp,							/*  坡道  */
}Road_TypeDef;

/*  车子丢线情况枚举变量  */
typedef enum
{
	LostLine_None = 0x0,		/*  未丢线  */
	LostLine_Left,					/*  左丢线  */
	LostLine_Right,					/*  右丢线  */
}LoseLine_TypeDef;


/*  车子整体参数结构体定义  */
typedef struct
{
	PID_TypeDef PID;											/*  PID参数  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  传感器  */
	Motor_TypeDef Motor;									/*  电机  */
	
	float HorizontalAE, VecticalAE;				/*  传感器水平、垂直和差比  */
	int16_t BaseSpeed;										/*  车子基本速度  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  出线阈值  */
	Road_TypeDef NowRoad;
	LoseLine_TypeDef LossLine;
	int16_t MaxPWM;
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);
void Car_Running(void);
# endif

/********************************************  END OF FILE  *******************************************/


