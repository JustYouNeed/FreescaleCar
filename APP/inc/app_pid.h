/**
  *******************************************************************************************************
  * File Name: app_pid.h
  * Author: Vector
  * Version: V2.3.0
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
	*		3.Author: Vector
	*			Date: 2018-4-22
	*			Mod: ����ģ��PID
	*
	*		4.Author: Vector
	*			Date: 2018-5-4
	*			Mod: �޸�PID�ṹ��,��ͨ��
	*					
  *
  *******************************************************************************************************
  */	
	
# ifndef __APP_PID_H
# define __APP_PID_H


/*  PID���ƽṹ��  */
typedef struct
{
	float Kp, Ki, Kd;		/*  ����ϵ��  */
	
	/*  ��ǰƫ��,�ϴ�ƫ��,���ϴ�ƫ��,����������Ҫ��������ʽPID  */
	float ErrorK, ErrorK_1, ErrorK_2;
	float Integral;		/*  ����  */
	float IntMax, IntRange;		/*  �������ֵ, ��������  */
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
	volatile float Ku_D;					/*  ��������,Ku_D = KdMax/N,��������������������Ŵ�  */
	volatile float KP;
	volatile float KI;
	volatile float KD;
		
	float KPMax, KIMax, KDMax;	/*  ��ģ��PID�����������������������ֵ  */
}FuzzyPID_TypeDef;




void pid_ParaInit(void);
void pid_ReadPara(void);
void pid_StorePara(void);

void pid_PIDInit(PID_TypeDef *PID, float Kp, float Ki, float Kd, float IntMax, float IntRange);
float pid_IncrementalCalc(PID_TypeDef *PID, float Error);
float pid_PositionalCalc(PID_TypeDef *PID, float Error);

void fuzzy_PIDClac(FuzzyPID_TypeDef *Fuzzy, float Error, float DError);
void fuzzy_PIDInit(FuzzyPID_TypeDef *Fuzzy);

# endif
	
	
/********************************************  END OF FILE  *******************************************/
	

