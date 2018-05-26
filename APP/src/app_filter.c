/**
  *******************************************************************************************************
  * File Name: app_filter.c
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ�˸����˲�����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-3-2
	*			Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Data: 2018-4-20
	*			Mod: ���ӿ������˲�����
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"

/*
*********************************************************************************************************
*                            filter_SildingAverage              
*
* Description: ������ֵ�˲�
*							 �ŵ�: 1.�������Ը����������õĵ�������,ƽ���ȸ�,�����ڵ�Ƶ�񵴵�ϵͳ
*              ȱ��: 1.�����ȵ�
*										 2.��żȻ���ֵ������Ը��ŵĵ������ýϲ�
*										 3.�����������������������Ĳ���ֵƫ��
*										 4.��������������űȽ����صĳ���
*										 5.�Ƚ��˷�RAM
* Arguments  : 1> Array[]: ���ݻ�����
*              2> Average: �˲����ֵ
*              3> Length: ���ݳ���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length)
{
	uint16_t cnt = 0;
	uint32_t sum = 0;
	
	/*  �����  */
	for(; cnt < Length; cnt ++)
	{
		sum += Array[cnt];
	}
	*Average = (uint16_t)(sum / Length);
}
/*
*********************************************************************************************************
*                                   filter_Kalman1Dim_Init       
*
* Description: ��ʼ���������ṹ�����
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalam_struct, double Q, double R)
{
	Kalam_struct->Kg = 0;              //����������
	Kalam_struct->Output = 0;          //�����
	Kalam_struct->P = 0;               //Э����
	Kalam_struct->Q = Q;               //ϵͳ����������Э����/Ԥ��ֵ���Ŷ�
	Kalam_struct->R = R;               //ϵͳ����������Э����/����ֵ���Ŷ�
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input)
{
	Kalam_Struct->P += Kalam_Struct->Q;
	
	Kalam_Struct->Output += Kalam_Struct->Kg*(input - Kalam_Struct->Output);
	
	Kalam_Struct->Kg = Kalam_Struct->P/(Kalam_Struct->P + Kalam_Struct->R);
	
	Kalam_Struct->P *= (1-Kalam_Struct->Kg);
}

/*
*********************************************************************************************************
*                       filter_KanlmanInit                   
*
* Description: ��ʼ���������˲��ṹ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_KanlmanInit(Kalman_TypeDef *Kalman)
{
	/*  ��ʼ���˲������Ϊ0  */
	Kalman->X[0] = 0;
	Kalman->X[1] = 0;
	
	/*  �˲�����������  */
	Kalman->dt = 0.005;
	
	/*  �˲���״̬ת�ƾ���  */
	Kalman->A[0][0] = 1;
	Kalman->A[0][1] = -Kalman->dt;
	Kalman->A[1][0] = 0;
	Kalman->A[1][1] = 1;
	
	/*  ��������ת�ƾ���  */
	Kalman->B[0] = Kalman->dt;
	Kalman->B[1] = 0;
	
	/*  Э�������  */
	Kalman->P[0][0] = 1;
	Kalman->P[0][1] = 0;
	Kalman->P[1][0] = 0;
	Kalman->P[1][1] = 1;
	
	/*  Ԥ��ֵ�����Ŷ�  */
	Kalman->Q[0][0] = 0.001;
	Kalman->Q[0][1] = 0;
	Kalman->Q[1][0] = 0;
	Kalman->Q[1][1] = 0.003;
	
	/*  ������������  */
	Kalman->R = 0.5;
}

/*
*********************************************************************************************************
*                       filter_KalmanFilter                   
*
* Description: �������˲�����
*             
* Arguments  : 1.Kalman: Kalman���ƽṹ��
*							 2.Gyro: �������Ľ��ٶ�
*							 3.AccAngle: �������ļ��ٶȼ�����ĽǶ�
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void filter_KalmanFilter(Kalman_TypeDef *Kalman, double Gyro, double AccAngle)
{	
	/*  ��ʽ1,X(k|k-1) = AX(k-1|k-1) + BU(k)  X, A, B, ��Ϊ����, �����������  */
	Kalman->X[0] = (Kalman->A[0][0] * Kalman->X[0] + Kalman->A[0][1] * Kalman->X[1]) + Gyro * Kalman->B[0];
	Kalman->X[1] = (Kalman->A[1][0] * Kalman->X[0] + Kalman->A[1][1] * Kalman->X[1]) + Gyro * Kalman->B[1];
	
	/*  ��ʽ2, P(k|k-1) = AP(k-1|k-1)A_T + Q */
	Kalman->P[0][0] = Kalman->P[0][0] - Kalman->P[1][0]*Kalman->dt - Kalman->P[0][1]*Kalman->dt + Kalman->P[0][0] * Kalman->dt * Kalman->dt + Kalman->Q[0][0];
	Kalman->P[0][1] = Kalman->P[0][1] - Kalman->P[1][1] * Kalman->dt + Kalman->Q[0][1];
	Kalman->P[1][0] = Kalman->P[1][0] - Kalman->P[1][1] * Kalman->dt + Kalman->Q[1][0];
	Kalman->P[1][1] = Kalman->P[1][1] + Kalman->Q[1][1];
	
	/*  ��ʽ3, Kg(k) = P(k|k-1)H_T/(HP(k|k-1)H_T + R), HΪϵ������ H = | 1 0 |  */
	Kalman->Kg[0] = Kalman->P[0][0] / (Kalman->P[0][0] + Kalman->R);
	Kalman->Kg[1] = Kalman->P[1][0] / (Kalman->P[0][0] + Kalman->R);
	
	/*  ��ʽ4, X(k|k) = X(k|k-1) + Kg(k)(Z(k) - H*X(k|k-1)), Z(k)Ϊϵͳ��������  */
	Kalman->X[0] = Kalman->X[0] + Kalman->Kg[0] * (AccAngle - Kalman->X[0]);
//	Kalman->X[1] = Kalman->X[1] + Kalman->Kg[1] * (Gyro - Kalman->X[1]);
	Kalman->Gyro = Gyro - Kalman->X[1];
	
	/*  ��ʽ5, P(k|k) = (I - Kg(k)*H)P(k|k-1)  */
	Kalman->P[0][0] = Kalman->P[0][0] * (1 - Kalman->Kg[0]);
	Kalman->P[0][1] = Kalman->P[0][1] * (1 - Kalman->Kg[0]);
	Kalman->P[1][0] = Kalman->P[1][0] - Kalman->P[0][0] * Kalman->Kg[1];
	Kalman->P[1][1] = Kalman->P[1][1] - Kalman->P[0][1] * Kalman->Kg[1];
}



/********************************************  END OF FILE  *******************************************/

