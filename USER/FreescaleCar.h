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

/*  ��·���ö�ٱ���  */
typedef enum
{
	Road_Straight = 0x0,		/*  ֱ��  */
	Road_SmallCurve,				/*  С��  */
	Road_BigCurve,					/*  ����  */
	Road_Island,						/*  ����  */
	Road_Circle,						/*  Բ��  */
	Road_Ramp,							/*  �µ�  */
}Road_TypeDef;

/*  ���Ӷ������ö�ٱ���  */
typedef enum
{
	LostLine_None = 0x0,		/*  δ����  */
	LostLine_Left,					/*  ����  */
	LostLine_Right,					/*  �Ҷ���  */
}LoseLine_TypeDef;


/*  ������������ṹ�嶨��  */
typedef struct
{
	PID_TypeDef PID;											/*  PID����  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  ������  */
	Motor_TypeDef Motor;									/*  ���  */
	
	float HorizontalAE, VecticalAE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	int16_t BaseSpeed;										/*  ���ӻ����ٶ�  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  ������ֵ  */
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


