/**
  *******************************************************************************************************
  * File Name: FreescaleCar.h
  * Author: Vector
  * Version: 1.2.0
  * Date: 2018-3-18
  * Brief: 本文件提供了有关车子控制的函数,变量等
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-18
	*			Mod: 建立文件
  *
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 删除小车结构体中阈值变量
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
# include "app.h"

/*  编码器每圈输出脉冲数  */
# define ENCONDER_LINES			512



/*  车轮周长,单位 米  */
# define WHEEL_GIRTH				0.2
# define WHEEL_LEN					0.16
# define WHEEL_D						0.06

/*  小车走走道时的目标速度  */
# define STRAIGHT_SPEED		15
# define CURVE_SPEED_LOW	20

# define DEFAULT_SPEED_KP	245
# define DEFAULT_SPEED_KD	1

# define DEFAULT_DIRECTION_KP	30
# define DEFAULT_DIRECTION_KD	300

# define DEFAULT_SPEED  20


/*  速度转换比例因子,计算完成后速度单位为 转速  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)


/*  车子整体参数结构体定义  */
typedef struct
{
	PID_TypeDef PID;											/*  PID参数  */
	FuzzyPID_TypeDef DirFuzzy;						/*  转向控制模糊PID  */
	FuzzyPID_TypeDef LVFuzzy;   				  /*  左边速度控制模糊PID  */
	FuzzyPID_TypeDef RVFuzzy;							/*  右边速度控制模糊PID  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  传感器  */
	Motor_TypeDef Motor;									/*  电机  */
	MPU_TypeDef MPU;											/*  MPU参数  */
	
	float HorizontalAE, VecticalAE, AE;				/*  传感器水平、垂直和差比  */
	float CarSpeed, TargetSpeed, LeftTargetSpeed, RightTargetSpeed;	/*  当前车速,整体目标速度,左右轮目标速度  */
	int16_t MaxPWM;												/*  最大PWM  */
}Car_TypeDef;

extern Car_TypeDef Car;
extern Kalman1Dim_TypeDef Kalman_Gryoz;


void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStore(void);
void Car_Reset(void);
void Car_Running(void);
void Car_ControlStop(void);
void Car_ControlStart(void);
# endif

/********************************************  END OF FILE  *******************************************/


