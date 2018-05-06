/**
  *******************************************************************************************************
  * File Name: app_pid.c
  * Author: Vector
  * Version: V3.0.1
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ��PID�㷨����,�Լ�PID�����Ķ�ȡ���洢����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
	*		2.Author: Vector
	*			Date: 2018-4-22
	*			Mod: ����ģ��PID
	*		
	*		3.Author: Vector
	*			Date: 2018-4-23
	*			Mod: 1.����ģ��PID��������
	*					 2.�޸�ģ��PID���㺯��,��PD,��ΪPID
	*
	*		4.Author: Vector
	*			Date: 2018-5-5
	*			Mod: ���Ӿ���PID�ļ��㺯��,λ��ʽ�Լ�����ʽ
	*
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app_pid.h"
# include "FreescaleCar.h"
# include "math.h"


# define NB		-3
# define NM		-2
# define NS		-1
# define ZO		0
# define PS		1
# define PM		2
# define PB		3

# define N	7

/*  P��ģ�������  */
static int KP_Rule[N][N] = 
{
	{PB, PB, PM, PM, PS, ZO, ZO},
	{PB, PB, PM, PS, PS, ZO, NS},
	{PB, PM, PM, PS, ZO, NS, NS},
	{PM, PM, PS, ZO, NS, NM, NM},
	{PS, PS, ZO, NS, NS, NM, NM},
	{PS, ZO, NS, NM, NM ,NM, NB},
	{ZO, ZO, NM, NM, NM, NB, NB},
};

/*  I��ģ�������  */
static int KI_Rule[N][N] = 
{
	{NB, NB, NM, NM, NS, ZO, ZO},
	{NB, NB, NM, NS, NS, ZO, ZO},
	{NB, NM, NS, NS, ZO, PS, PS},
	{NM, NM, NS, ZO, PS, PM, PM},
	{NM, NS, ZO, PS, PS, PM, PB},
	{ZO, ZO, PS, PS, PM, PB, PB},
	{ZO, ZO, PS, PM, PM, PB, PB},
};

/*  D��ģ�������  */
static int KD_Rule[N][N] = 
{
	{PS, NS, NB, NB, NB, NM, PS},
	{PS, NS, NB, NM, NM, NS, ZO},
	{ZO, NS, NM, NM, NS, NS, ZO},
	{ZO, NS, NS, NS, NS, NS, ZO},
	{ZO, ZO, ZO, ZO, ZO ,ZO, ZO},
	{PB, NS, PS, PS, PS, PS, PB},
	{PB, PM, PM, PM, PS, PS, PB},
};

/*  �����������ȱ�,�ֳ��߸�������  */
static float TriMF_Table[7][3] = 
{
	{NB, NB, NM},
	{NB, NM, NS},
	{NM, NS, ZO},
	{NS, ZO, PS},
	{ZO, PS, PM},
	{PS, PM, PB},
	{PM, PB, PB},
};
/*
*********************************************************************************************************
*                        pid_ReadPara                  
*
* Description: ��Flash�ж�ȡ�洢��PID����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_ReadPara(void)
{
//	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
//	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
//	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
//	
//	Car.PID.SpeedKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
//	Car.PID.SpeedKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
//	Car.PID.SpeedKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
}

/*
*********************************************************************************************************
*                      pid_StorePara                    
*
* Description: ��PID�����洢��Flash��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_StorePara(void)
{	
	float temp[32] = {0};
	uint8_t cnt = 0;
	
	temp[cnt++] = Car.VelPID.Kp;
	temp[cnt++] = Car.VelPID.Ki;
	temp[cnt++] = Car.VelPID.Kd;
	
	temp[cnt++] = Car.DirFuzzy.KPMax;
	temp[cnt++] = Car.DirFuzzy.KIMax;
	temp[cnt++] = Car.DirFuzzy.KDMax;
	
	DISABLE_INT();
	drv_flash_EraseSector(PID_PARA_FLASH_ADDR);	/*  �Ȳ���һ��,��Ȼ�޷�д��  */
	drv_flash_WriteSector(PID_PARA_FLASH_ADDR, (const uint8_t*)&temp, cnt * 4, 0);
	ENABLE_INT();
}

/*
*********************************************************************************************************
*                             pid_PIDInit             
*
* Description: ��ʼ��PID���ƽṹ��
*             
* Arguments  : 1.Kp,Ki,Kd: PID��ʼϵ��
*							 2.IntMax: �������ֵ
*							 3.IntRange: ��������
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_PIDInit(PID_TypeDef *PID, float Kp, float Ki, float Kd, float IntMax, float IntRange)
{
	PID->IntMax = IntMax;
	PID->IntRange = IntRange;
	
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	
	PID->ErrorK = 0;
	PID->ErrorK_1 = 0;
	PID->ErrorK_2 = 0;
	PID->Integral = 0;
}

/*
*********************************************************************************************************
*                           pid_IncrementalCalc               
*
* Description: ����ʽPID����
*             
* Arguments  : 1.PID: PID���ƽṹ��
*							 2.Error: ϵͳƫ��
*
* Reutrn     : 1.ϵͳ�������
*
* Note(s)    : ��������ʽPIDʱ,Ӧ�����ȵ��ڻ���ϵ��Ki,��Ϊ�����λ��ʽPID,Ki��Ч��Kp
*********************************************************************************************************
*/
float pid_IncrementalCalc(PID_TypeDef *PID, float Error)
{
	float OutputInc = 0;
	
	PID->ErrorK = Error;
	
	OutputInc = PID->Kp * (PID->ErrorK - PID->ErrorK_1) + 
							PID->Ki * PID->ErrorK + 
							PID->Kd * (PID->ErrorK - 2 * PID->ErrorK_1 + PID->ErrorK_2);
	
	PID->ErrorK_2 = PID->ErrorK_1;
	PID->ErrorK_1 = PID->ErrorK;
	
	return OutputInc;
}

/*
*********************************************************************************************************
*                          pid_PositionalCalc                
*
* Description: λ��ʽPID����
*             
* Arguments  : 1.PID: PID���ƽṹ��
*
* Reutrn     : 1.PID������
*
* Note(s)    : None.
*********************************************************************************************************
*/
float pid_PositionalCalc(PID_TypeDef *PID, float Error)
{
	float Output = 0;
	float ErrorDirr = 0;
	
	PID->ErrorK = Error;
	ErrorDirr = PID->ErrorK - PID->ErrorK_1;
	
	/*  ��������˻�������,��ֻ�ڸ��������  */
	if(PID->IntRange != 0)
	{
		if(PID->ErrorK > -PID->IntRange && PID->ErrorK < PID->IntRange)
			PID->Integral += PID->ErrorK;
	}
	else
	{
		PID->Integral += PID->ErrorK;
	}
	
	/*  ��������˻������ֵ  */
	if(PID->IntMax != 0)
	{
		if(PID->Integral > PID->IntMax ) PID->Integral = PID->IntMax;
		else if(PID->Integral < -PID->IntMax) PID->Integral = -PID->IntMax;
	}
	
	Output = PID->Kp * PID->ErrorK + PID->Ki * PID->Integral + PID->Kd * ErrorDirr;
	
	PID->ErrorK_1 = PID->ErrorK;
	
	return Output;
}


/*
*********************************************************************************************************
*                       fuzzy_PIDInit                   
*
* Description: ģ��PID��ʼ��
*             
* Arguments  : 1> Fuzzy: ģ��PID���ƽṹ��
*							 2> ErrMax: ƫ������ֵ
*							 3> DErrMax: ƫ��仯�����ֵ
*						   4> KpMax: KP�����ֵ
*						   5> KiMax: KI�����ֵ
*							 6> KdMax: KD�����ֵ
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void fuzzy_PIDInit(FuzzyPID_TypeDef *Fuzzy)
{	
	Fuzzy->Kerr = (N/2)/Fuzzy->ErrMax;		/*  ��������,��ƫ������ת������������  */
	Fuzzy->Kderr = (N/2)/Fuzzy->DErrMax;	/*  ��������,��ƫ��΢������ת������������  */
	
	Fuzzy->Ku_P = Fuzzy->DeltaKpMax / (N/2);
	Fuzzy->Ku_I = Fuzzy->DeltaKiMax / (N/2);
	Fuzzy->Ku_D = Fuzzy->DeltaKdMax / (N/2);
}

/*
*********************************************************************************************************
*                      fuzzy_TriMF                    
*
* Description: �����������Ⱥ���
*             
* Arguments  : 1> x: Ҫ�������ȵ�ֵ
*							 2> Left,Mid,Right: �����ε����ֵ,�м�ֵ,�ұ�ֵ
*
* Reutrn     : ������ֵ
*
* Note(s)    : None.
*********************************************************************************************************
*/
float fuzzy_TriMF(float x, float Left, float Mid, float Right)
{
	float u = 0.0f;
	
	if(x >= Left && x <= Mid)	/*  ���x��ֵ���������ε����  */
		u = (x - Left) / (Mid - Left);
	else if(x > Mid && x <= Right) /*  ���ұ�  */
		u = (Right - x) / (Right - Mid);
	else 	/*  ���ڸ���������  */
		u = 0.0f;
	
	return u;
}

/*
*********************************************************************************************************
*                      fuzzy_GaussMF                    
*
* Description: ��˹�ֲ���������
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
float fuzzy_GaussMF(float x, float ave, float sigma)
{
	float u = 0.0f;
	
	if(sigma < 0) return u;
	
	u = exp(-pow(((x - ave)/sigma), 2));
	
	return u;
}

/*
*********************************************************************************************************
*                     fuzzy_TraMF                     
*
* Description: ���������Ⱥ���
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
float fuzzy_TraMF(float x, float a, float b, float c, float d)
{
	float u = 0.0f;
	
	if(x >= a && x < b)
		u = (x - a) / (b- a);
	else if(x >= b && x < c)
		u = 1;
	else if(x >= c && x <= d)
		u = (d -x) / (d - c);
	else 
		u = 0.0f;
	
	return u;
}

/*
*********************************************************************************************************
*                                     fuzzy_PIDClac     
*
* Description: ����ģ��PID�ļ���
*             
* Arguments  : 1> Fuzzy: ģ��PID���ƽṹ��
*							 2> Error: ƫ��
*							 3> DError: ƫ���΢��
*
* Reutrn     : None.
*
* Note(s)    : ��ģ��PID������KP,KI,KD���ڸ�ֵ�����,��Ϊ�����и���,���������������ļ���Ӧ����ȷ��������,
*							 �����ʹ�õ�ʱ����Ҫ��һ�¼��Դ���,��������
*********************************************************************************************************
*/
void fuzzy_PIDClac(FuzzyPID_TypeDef *Fuzzy, float Error, float DError)
{
	volatile float e = 0, ec = 0;
	static float u_e[7], u_de[7];
	static int u_e_index[7] = {0}, u_de_index[7] = {0};
	volatile float den = 0, num = 0;
	volatile float delta_Kp = 0, delta_Ki = 0, delta_Kd = 0;
	uint8_t i = 0, j = 0;
	
	/*  �����  */
	for(i = 0; i < 7; i++)
	{
		u_e[i] = 0;
		u_de[i] = 0;
		
		u_e_index[i] = 0;
		u_de_index[i] = 0;
	}
	i = 0;
	
	/*  ��ƫ���ƫ��ı仯��ģ������������  */
	e = Error * Fuzzy->Kerr;
	ec = DError * Fuzzy->Kderr;
	
	/*  ��ƫ���������,���������������ȷ�  */
	for(i = 0; i < 7; i++)
	{
		u_e[i] = fuzzy_TriMF(e, TriMF_Table[i][0], TriMF_Table[i][1], TriMF_Table[i][2]);
		
		/*  �����Ȳ�Ϊ0,�򼤻���ع���,��󴥷�7������  */
		if(u_e[i] != 0)
			u_e_index[j++] = i;
	}
	for(; j < 7; j++) u_e_index[j] = 0;		/*  ��ʣ�µĹ���������  */
	j = 0;
	
	/*  ��Ec��������  */
	for(i = 0; i < 7; i++)
	{
		u_de[i] = fuzzy_TriMF(ec, TriMF_Table[i][0], TriMF_Table[i][1], TriMF_Table[i][2]);
		
		if(u_de[i] != 0)
			u_de_index[j++] = i;
	}
	for(; j < 7; j++) u_de_index[j] = 0;
	j = 0;
	
/********************************************   KP   *******************************************/
	/*  ��ģ��,��deltaKp,�������ķ�  */
	for(i = 0; i < 7; i++)
	{
		for(j = 0; j < 7; j++)
		{
			num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * KP_Rule[u_e_index[i]][u_de_index[j]];
			den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
		}
	}
	/*  ���Է��ֻ����den����0�����,���³������,��˼����ж�  */
	if(den == 0) 
		delta_Kp = 0;
	else
		delta_Kp = num/den;
	
	delta_Kp = Fuzzy->Ku_P * delta_Kp;
	
	/*  ��������޷�  */
	if(delta_Kp > Fuzzy->DeltaKpMax) delta_Kp = Fuzzy->DeltaKpMax;
	else if(delta_Kp < - Fuzzy->DeltaKpMax) delta_Kp = -Fuzzy->DeltaKpMax;
	Fuzzy->KP += delta_Kp;			/*  ��������KP  */
	
	/*  ��������޷�  */
	if(Fuzzy->KP < -Fuzzy->KPMax) Fuzzy->KP = -Fuzzy->KPMax;
	if(Fuzzy->KP > Fuzzy->KPMax) Fuzzy->KP = Fuzzy->KPMax;
/********************************************  END  *******************************************/
	
	den = 0; 
	num = 0;
	
/********************************************  KI  *******************************************/
	/*  ���ķ���ģ��KI  */
	for(i = 0; i < 7; i ++)
	{
		for(j = 0; j < 7; j++)
		{
			num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * KI_Rule[u_e_index[i]][u_de_index[j]];
			den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
		}
	}
	/*  ���Է��ֻ����den����0�����,���³������,��˼����ж�  */
	if(den == 0) 
		delta_Ki = 0;
	else
		delta_Ki = num/den;
	
	delta_Ki = Fuzzy->Ku_I * delta_Ki;		/*  �Ŵ�  */
	
	/*  ��������޷�  */
	if(delta_Ki > Fuzzy->DeltaKiMax) delta_Ki = Fuzzy->DeltaKiMax;
	else if(delta_Ki < - Fuzzy->DeltaKiMax) delta_Ki = -Fuzzy->DeltaKiMax;
	Fuzzy->KI += delta_Ki;			/*  ��������KI  */
	
	/*  ��������޷�  */
	if(Fuzzy->KI < -Fuzzy->KIMax) Fuzzy->KI = -Fuzzy->KIMax;
	if(Fuzzy->KI > Fuzzy->KIMax) Fuzzy->KI = Fuzzy->KIMax;
/********************************************  END  *******************************************/	
	
	den = 0; 
	num = 0;
	
/********************************************   KD   *******************************************/	
	/*  ��ģ��,���KD,�������ķ���ģ��  */
	for(i = 0; i < 7; i++)
	{
		for(j = 0; j < 7; j++)
		{
			num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * KD_Rule[u_e_index[i]][u_de_index[j]];
			den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
		}
	}
	if(den == 0) 
		delta_Kd = 0;
	else
		delta_Kd = num / den;
	delta_Kd = Fuzzy->Ku_D * delta_Kd;
	
	
	if(delta_Kd > Fuzzy->DeltaKdMax) delta_Kd = Fuzzy->DeltaKdMax;
	else if(delta_Kd < -Fuzzy->DeltaKdMax) delta_Kd = -Fuzzy->DeltaKdMax;
	Fuzzy->KD += delta_Kd;			/*  �������  */
	
	/*  ��������޷�  */
	if(Fuzzy->KD < -Fuzzy->KDMax) Fuzzy->KD = -Fuzzy->KDMax;
	if(Fuzzy->KD > Fuzzy->KDMax) Fuzzy->KD = Fuzzy->KDMax;
/********************************************  END  *******************************************/	
}
/********************************************  END OF FILE  *******************************************/

