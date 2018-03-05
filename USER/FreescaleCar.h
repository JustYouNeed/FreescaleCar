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


/*  ������������ṹ�嶨��  */
typedef struct
{
	PID_TypeDef PID;											/*  PID����  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  ������  */
	Motor_TypeDef Motor;									/*  ���  */
	
	float HorizontalAE, VecticalAE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	int16_t BaseSpeed;										/*  ���ӻ����ٶ�  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  ������ֵ  */
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);

# endif

/********************************************  END OF FILE  *******************************************/


