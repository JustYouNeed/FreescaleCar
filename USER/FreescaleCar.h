/**
  *******************************************************************************************************
  * File Name: FreescaleCar.h
  * Author: Vector
  * Version: 1.2.0
  * Date: 2018-3-18
  * Brief: ���ļ��ṩ���йس��ӿ��Ƶĺ���,������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-18
	*			Mod: �����ļ�
  *
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: ɾ��С���ṹ������ֵ����
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

/*  ������ÿȦ���������  */
# define ENCONDER_LINES			512

/*  ���������ֳ���  */
# define ENCONDER_TEETH			29

/*  ���ӳ��ֳ���  */
# define WHEEL_TEETH				68


/*  �����ܳ�,��λ ��  */
# define WHEEL_GIRTH				0.2

/*  �������ľ�  */
# define WHEEL_LEN					0.16

/*  ���Ӱ뾶,��λ��  */
# define WHEEL_D						0.06




/*  С�����ߵ�ʱ��Ŀ���ٶ�  */
# define STRAIGHT_SPEED		15
# define CURVE_SPEED_LOW	20

# define DEFAULT_SPEED_KP	245
# define DEFAULT_SPEED_KD	1

# define DEFAULT_DIRECTION_KP	30
# define DEFAULT_DIRECTION_KD	300

# define DEFAULT_SPEED  18

/*  ��·����ö�ٱ���  */
typedef enum
{
	STRAIGHT = 0x0,			/*  ֱ��  */
	LEFT_CURVE,					/*  ��ת��  */
	RIGHT_CURVE,				/*  ��ת��  */
	LEFT_ISLAND,				/*  ����  */
	RIGHT_ISLAND,
	
}Road_TypeDef;

/*  ������������ṹ�嶨��  */
typedef struct
{
	PID_TypeDef VelPID;										/*  �ٶȻ�PID  */
	FuzzyPID_TypeDef DirFuzzy;						/*  ת�����ģ��PID  */
	Sensor_TypeDef Sensor[SENSOR_COUNT];	/*  ������  */
	Motor_TypeDef Motor;									/*  ���  */
	MPU_TypeDef MPU;											/*  MPU����  */
	Road_TypeDef NowRoad;
	
	float HorizontalAE, VecticalAE, AE;				/*  ������ˮƽ����ֱ�Ͳ��  */
	float CarSpeed, TargetSpeed;	/*  ��ǰ����,����Ŀ���ٶ�  */
	int16_t MaxPWM;												/*  ���PWM  */
	float Voltage;												/*  ��ص�ѹ  */
}Car_TypeDef;

extern Car_TypeDef Car;


void Car_ParaInit(void);
void Car_Control(void);
void Car_ParaStore(void);
void Car_Reset(void);
void Car_Running(void);
void Car_ControlStop(void);
void Car_ControlStart(void);
void Car_GetVoltage(void);
# endif

/********************************************  END OF FILE  *******************************************/


