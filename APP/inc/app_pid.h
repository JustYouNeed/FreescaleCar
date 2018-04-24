/**
  *******************************************************************************************************
  * File Name: app_pid.h
  * Author: Vector
  * Version: V1.2.0
  * Date: 2018-2-1
  * Brief: ���ļ��ṩ�˹���PID����������,�Լ���ر����Ķ���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-1
	*			Mod: �������ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: �޸�PID�����������,ɾ������Ҫ�ı���,Error, Sum��
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

/*  PID���ƽṹ��  */
typedef struct
{
	float SpeedKp, SpeedKi, SpeedKd;
	float DirectionKp, DirectionKi, DirectionKd;
}PID_TypeDef;


/*  ģ��PID���ƽṹ��  */
typedef struct 
{
	float ErrMax;		/*  �������  */
	float DErrMax;	/*  ���仯������  */
	volatile float Kerr;			/*  ƫ�����������,Kerr = N/ErrMax  */
	volatile float Kderr;		/*  ƫ��仯����������,Kderr = N/DErrMax  */
	
	float DeltaKpMax;		/*  �����KP��������  */
	float DeltaKiMax;		/*  KI��������  */
	float DeltaKdMax;		/*  KD��������  */
	volatile float Ku_P;					
	volatile float Ku_I;
	volatile float Ku_D;					/*  ��������,Ku_D = KdMax/N  */
	volatile float KP;
	volatile float KI;
	volatile float KD;
		
	float KPMax, KIMax, KDMax;	/*  ��ģ��PID�����������������������ֵ  */
}FuzzyPID_TypeDef;




void pid_ParaInit(void);
void pid_ReadPara(void);
void pid_StorePara(void);


void fuzzy_PIDClac(FuzzyPID_TypeDef *Fuzzy, float Error, float DError);
void fuzzy_PIDInit(FuzzyPID_TypeDef *Fuzzy);

# endif
	
	
/********************************************  END OF FILE  *******************************************/
	

