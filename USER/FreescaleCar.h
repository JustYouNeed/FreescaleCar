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
# include "bsp_mpu.h"

/*  ������ÿȦ���������  */
# define ENCONDER_LINES			512

/*  �ٶȿ�������,50ms  */
# define SPEED_CONTROL_PERIOD	10	

# define DIRCTION_CONTROL_PERIOD	5

/*  �����ܳ�,��λ ��  */
# define WHEEL_GIRTH				0.2
# define WHEEL_LEN					0.16
# define WHEEL_D						0.06


# define BIG_CURVE_R			0.5
# define MID_CURVE_R			0,3
# define SMALL_CURVE_R		0.2
# define STRAIGHT					0


/*  �ٶ�ת����������,������ɺ��ٶȵ�λΪ m/s  */
# define CAR_SPED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)

/*  С�����ߵ�ʱ��Ŀ���ٶ�  */
# define STRAIGHT_SPEED		25
# define CURVE_SPEED			5

/*  ת�����  */
typedef enum
{
	TurnLeft = 0x0,
	TurnRight,
	Straight,
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
	LoseLine_TypeDef LossLine;						/*  ����  */
	Road_TypeDef NowRoad;									/*  ��ǰ·��  */
	MPU_TypeDef MPU;											/*  MPU����  */
	
	float HorizontalAE, VecticalAE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	float CarSpeed, TargetSpeed, LeftTargetSpeed, RightTargetSpeed;	/*  ��ǰ����,����Ŀ���ٶ�,������Ŀ���ٶ�  */
	int16_t OutThreshold[SENSOR_COUNT];		/*  ������ֵ  */
	int16_t MaxPWM;												/*  ���PWM  */
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


