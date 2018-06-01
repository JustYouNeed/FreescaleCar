/**
  *******************************************************************************************************
  * File Name: app_pid.h
  * Author: Vector
  * Version: V2.3.0
  * Date: 2018-2-1
  * Brief: 本文件提供了关于PID函数的声明,以及相关变量的定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-1
	*			Mod: 建立本文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 修改PID结体变量名字,删除不必要的变量,Error, Sum等
	*
	*		3.Author: Vector
	*			Date: 2018-4-22
	*			Mod: 增加模糊PID
	*
	*		4.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 修改PID结构体,更通用
	*					
  *
  *******************************************************************************************************
  */	
	
# ifndef __APP_PID_H
# define __APP_PID_H


/*  PID控制结构体  */
typedef struct
{
	float Kp, Ki, Kd;		/*  三个系数  */
	
	/*  当前偏差,上次偏差,上上次偏差,后面两个主要用于增量式PID  */
	float ErrorK, ErrorK_1, ErrorK_2;
	float Integral;		/*  积分  */
	float IntMax, IntRange;		/*  积分最大值, 积分区间  */
}PID_TypeDef;


/*  模糊PID控制结构体  */
typedef struct 
{
	float ErrMax;		/*  误差上限  */
	float DErrMax;	/*  误差变化率上限  */
	volatile float Kerr;			/*  偏差的量化因子,Kerr = N/ErrMax  */
	volatile float Kderr;		/*  偏差变化率量化因子,Kderr = N/DErrMax  */
	
	float DeltaKpMax;		/*  计算的KP最大的增量  */
	float DeltaKiMax;		/*  KI最大的增量  */
	float DeltaKdMax;		/*  KD最大的增量  */
	volatile float Ku_P;					
	volatile float Ku_I;
	volatile float Ku_D;					/*  量化因子,Ku_D = KdMax/N,用来将计算出来的增量放大  */
	volatile float KP;
	volatile float KI;
	volatile float KD;
		
	float KPMax, KIMax, KDMax;	/*  由模糊PID计算后输出的三个参数的最大值  */
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
	

