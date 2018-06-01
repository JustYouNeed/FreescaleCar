/**
  *******************************************************************************************************
  * File Name: app_filter.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ������˸����˲�����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

# ifndef __APP_FILTER_H
# define __APP_FILTER_H

/*  һά�������˲��ṹ��  */
typedef struct 
{
	double Q;
	double R;
	double P;
	double Kg;
	double Output;
}Kalman1Dim_TypeDef;


typedef struct
{
	double X[2];			/*  ��������״̬X,������̬������˵,X[0]������̬��  */
	int16_t Gyro;			/*  ���Ž��ٶ�  */
	double A[2][2];		/*  AΪϵͳΪk-1��kʱ�̵�״̬ת�ƾ���  */
	double B[2];			/*  BΪϵͳ����  */
	double P[2][2];		/*  PΪϵͳЭ����  */
	double Q[2][2];		/*  QΪϵͳ����  */
	double R;					/*  ��������  */
	double Kg[2];			/*  KgΪϵͳ����  */	
	double dt;				/*  �˲�������ʱ��  */
}Kalman_TypeDef;

void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length);
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalman_struct, double Q, double R);
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input);
void filter_KanlmanInit(Kalman_TypeDef *Kalman);
void filter_KalmanFilter(Kalman_TypeDef *Kalman, double Gyro, double AccAngle);
# endif

/********************************************  END OF FILE  *******************************************/

