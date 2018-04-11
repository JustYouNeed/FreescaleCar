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
# include "app_pid.h"
# include "bsp_mpu.h"

/*  编码器每圈输出脉冲数  */
# define ENCONDER_LINES			512

/*  速度控制周期,50ms  */
# define SPEED_CONTROL_PERIOD	10	

# define DIRCTION_CONTROL_PERIOD	5

/*  车轮周长,单位 米  */
# define WHEEL_GIRTH				0.2
# define WHEEL_LEN					0.16
# define WHEEL_D						0.06


# define BIG_CURVE_R			0.5
# define MID_CURVE_R			0,3
# define SMALL_CURVE_R		0.2
# define STRAIGHT					0


/*  速度转换比例因子,计算完成后速度单位为 转速  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)



/*  转向情况  */
typedef enum
{
	TurnLeft = 0x0,
	TurnRight,
	Straight,
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
	LoseLine_TypeDef LossLine;						/*  丢线  */
	Road_TypeDef NowRoad;									/*  当前路况  */
	MPU_TypeDef MPU;											/*  MPU参数  */
	
	uint8_t Running;
	float HorizontalAE, VecticalAE;				/*  传感器水平、垂直和差比  */
	float CarSpeed, TargetSpeed, LeftTargetSpeed, RightTargetSpeed;	/*  当前车速,整体目标速度,左右轮目标速度  */
	int16_t MaxPWM;												/*  最大PWM  */
}Car_TypeDef;

extern Car_TypeDef Car;

void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStroe(void);
void Car_Running(void);
void Car_ControlStop(void);
void Car_ControlStart(void);
# endif

/********************************************  END OF FILE  *******************************************/


