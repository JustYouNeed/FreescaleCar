/**
  *******************************************************************************************************
  * File Name: app_pid.c
  * Author: Vector
  * Version: V3.0.1
  * Date: 2018-3-2
  * Brief: 本文件提供了PID算法函数,以及PID参数的读取、存储功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
  *
	*		2.Author: Vector
	*			Date: 2018-4-22
	*			Mod: 增加模糊PID
	*		
	*		3.Author: Vector
	*			Date: 2018-4-23
	*			Mod: 1.修正模糊PID规则表错误
	*					 2.修改模糊PID计算函数,由PD,改为PID
	*
	*		4.Author: Vector
	*			Date: 2018-5-5
	*			Mod: 增加经典PID的计算函数,位置式以及增量式
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

/*  P的模糊规则表  */
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

/*  I的模糊规则表  */
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

/*  D的模糊规则表  */
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

/*  三角形隶属度表,分成七个三角形  */
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
* Description: 从Flash中读取存储的PID参数
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
* Description: 将PID参数存储到Flash中
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
	drv_flash_EraseSector(PID_PARA_FLASH_ADDR);	/*  先擦除一遍,不然无法写入  */
	drv_flash_WriteSector(PID_PARA_FLASH_ADDR, (const uint8_t*)&temp, cnt * 4, 0);
	ENABLE_INT();
}

/*
*********************************************************************************************************
*                             pid_PIDInit             
*
* Description: 初始化PID控制结构体
*             
* Arguments  : 1.Kp,Ki,Kd: PID初始系数
*							 2.IntMax: 积分最大值
*							 3.IntRange: 积分区间
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
* Description: 增量式PID计算
*             
* Arguments  : 1.PID: PID控制结构体
*							 2.Error: 系统偏差
*
* Reutrn     : 1.系统输出增量
*
* Note(s)    : 调试增量式PID时,应该首先调节积分系数Ki,因为相对于位置式PID,Ki等效于Kp
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
* Description: 位置式PID计算
*             
* Arguments  : 1.PID: PID控制结构体
*
* Reutrn     : 1.PID计算结果
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
	
	/*  如果设置了积分区间,则只在该区间积分  */
	if(PID->IntRange != 0)
	{
		if(PID->ErrorK > -PID->IntRange && PID->ErrorK < PID->IntRange)
			PID->Integral += PID->ErrorK;
	}
	else
	{
		PID->Integral += PID->ErrorK;
	}
	
	/*  如果设置了积分最大值  */
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
* Description: 模糊PID初始化
*             
* Arguments  : 1> Fuzzy: 模糊PID控制结构体
*							 2> ErrMax: 偏差的最大值
*							 3> DErrMax: 偏差变化的最大值
*						   4> KpMax: KP的最大值
*						   5> KiMax: KI的最大值
*							 6> KdMax: KD的最大值
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void fuzzy_PIDInit(FuzzyPID_TypeDef *Fuzzy)
{	
	Fuzzy->Kerr = (N/2)/Fuzzy->ErrMax;		/*  比例因子,将偏差输入转换到基本论域  */
	Fuzzy->Kderr = (N/2)/Fuzzy->DErrMax;	/*  比例因子,将偏差微分输入转换到基本论域  */
	
	Fuzzy->Ku_P = Fuzzy->DeltaKpMax / (N/2);
	Fuzzy->Ku_I = Fuzzy->DeltaKiMax / (N/2);
	Fuzzy->Ku_D = Fuzzy->DeltaKdMax / (N/2);
}

/*
*********************************************************************************************************
*                      fuzzy_TriMF                    
*
* Description: 三角形隶属度函数
*             
* Arguments  : 1> x: 要求隶属度的值
*							 2> Left,Mid,Right: 三角形的左边值,中间值,右边值
*
* Reutrn     : 隶属度值
*
* Note(s)    : None.
*********************************************************************************************************
*/
float fuzzy_TriMF(float x, float Left, float Mid, float Right)
{
	float u = 0.0f;
	
	if(x >= Left && x <= Mid)	/*  如果x的值处于三角形的左边  */
		u = (x - Left) / (Mid - Left);
	else if(x > Mid && x <= Right) /*  在右边  */
		u = (Right - x) / (Right - Mid);
	else 	/*  不在该三角形内  */
		u = 0.0f;
	
	return u;
}

/*
*********************************************************************************************************
*                      fuzzy_GaussMF                    
*
* Description: 高斯分布求隶属度
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
* Description: 梯形隶属度函数
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
* Description: 进行模糊PID的计算
*             
* Arguments  : 1> Fuzzy: 模糊PID控制结构体
*							 2> Error: 偏差
*							 3> DError: 偏差的微分
*
* Reutrn     : None.
*
* Note(s)    : 由模糊PID计算后的KP,KI,KD存在负值的情况,因为输入有负的,但是这三个参数的极性应该是确定下来的,
*							 因此在使用的时候需要做一下极性处理,否则会出错
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
	
	/*  先清空  */
	for(i = 0; i < 7; i++)
	{
		u_e[i] = 0;
		u_de[i] = 0;
		
		u_e_index[i] = 0;
		u_de_index[i] = 0;
	}
	i = 0;
	
	/*  将偏差还有偏差的变化率模糊到基本论域  */
	e = Error * Fuzzy->Kerr;
	ec = DError * Fuzzy->Kderr;
	
	/*  求偏差的隶属度,采用三角形隶属度法  */
	for(i = 0; i < 7; i++)
	{
		u_e[i] = fuzzy_TriMF(e, TriMF_Table[i][0], TriMF_Table[i][1], TriMF_Table[i][2]);
		
		/*  隶属度不为0,则激活相关规则,最大触发7条规则  */
		if(u_e[i] != 0)
			u_e_index[j++] = i;
	}
	for(; j < 7; j++) u_e_index[j] = 0;		/*  将剩下的规则组清零  */
	j = 0;
	
	/*  求Ec的隶属度  */
	for(i = 0; i < 7; i++)
	{
		u_de[i] = fuzzy_TriMF(ec, TriMF_Table[i][0], TriMF_Table[i][1], TriMF_Table[i][2]);
		
		if(u_de[i] != 0)
			u_de_index[j++] = i;
	}
	for(; j < 7; j++) u_de_index[j] = 0;
	j = 0;
	
/********************************************   KP   *******************************************/
	/*  解模糊,求deltaKp,采用重心法  */
	for(i = 0; i < 7; i++)
	{
		for(j = 0; j < 7; j++)
		{
			num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * KP_Rule[u_e_index[i]][u_de_index[j]];
			den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
		}
	}
	/*  测试发现会出现den等于0的情况,导致程序错误,因此加上判断  */
	if(den == 0) 
		delta_Kp = 0;
	else
		delta_Kp = num/den;
	
	delta_Kp = Fuzzy->Ku_P * delta_Kp;
	
	/*  增量输出限幅  */
	if(delta_Kp > Fuzzy->DeltaKpMax) delta_Kp = Fuzzy->DeltaKpMax;
	else if(delta_Kp < - Fuzzy->DeltaKpMax) delta_Kp = -Fuzzy->DeltaKpMax;
	Fuzzy->KP += delta_Kp;			/*  计算最后的KP  */
	
	/*  最后的输出限幅  */
	if(Fuzzy->KP < -Fuzzy->KPMax) Fuzzy->KP = -Fuzzy->KPMax;
	if(Fuzzy->KP > Fuzzy->KPMax) Fuzzy->KP = Fuzzy->KPMax;
/********************************************  END  *******************************************/
	
	den = 0; 
	num = 0;
	
/********************************************  KI  *******************************************/
	/*  重心法解模糊KI  */
	for(i = 0; i < 7; i ++)
	{
		for(j = 0; j < 7; j++)
		{
			num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * KI_Rule[u_e_index[i]][u_de_index[j]];
			den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
		}
	}
	/*  测试发现会出现den等于0的情况,导致程序错误,因此加上判断  */
	if(den == 0) 
		delta_Ki = 0;
	else
		delta_Ki = num/den;
	
	delta_Ki = Fuzzy->Ku_I * delta_Ki;		/*  放大  */
	
	/*  增量输出限幅  */
	if(delta_Ki > Fuzzy->DeltaKiMax) delta_Ki = Fuzzy->DeltaKiMax;
	else if(delta_Ki < - Fuzzy->DeltaKiMax) delta_Ki = -Fuzzy->DeltaKiMax;
	Fuzzy->KI += delta_Ki;			/*  计算最后的KI  */
	
	/*  最后的输出限幅  */
	if(Fuzzy->KI < -Fuzzy->KIMax) Fuzzy->KI = -Fuzzy->KIMax;
	if(Fuzzy->KI > Fuzzy->KIMax) Fuzzy->KI = Fuzzy->KIMax;
/********************************************  END  *******************************************/	
	
	den = 0; 
	num = 0;
	
/********************************************   KD   *******************************************/	
	/*  解模糊,求解KD,采用重心法解模糊  */
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
	Fuzzy->KD += delta_Kd;			/*  计算输出  */
	
	/*  最后的输出限幅  */
	if(Fuzzy->KD < -Fuzzy->KDMax) Fuzzy->KD = -Fuzzy->KDMax;
	if(Fuzzy->KD > Fuzzy->KDMax) Fuzzy->KD = Fuzzy->KDMax;
/********************************************  END  *******************************************/	
}
/********************************************  END OF FILE  *******************************************/

