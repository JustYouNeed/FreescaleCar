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


/*  车子整体参数结构体定义  */
typedef struct
{
	PID_TypeDef PID;											/*  PID参数  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  传感器  */
	Motor_TypeDef Motor;									/*  电机  */
	
	float HorizontalAE, VecticalAE;				/*  传感器水平、垂直和差比  */
	int16_t BaseSpeed;										/*  车子基本速度  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  出线阈值  */
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);

# endif

/********************************************  END OF FILE  *******************************************/


