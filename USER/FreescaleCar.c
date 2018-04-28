/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.6.1
  * Date: 2018-3-3
  * Brief: 本文件用于车子数据记录、处理、保存等，同时车子控制函数也在本文件
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-3
	*			Mod: 建立文件
  *
	*		2.Author: Vector
	*			Date:	2018-3-19
	*			Mod: 1.修改车子控制逻辑,改变寻线策略
	*
	*		3.Author: Vector
	*			Date: 2018-3-24
	*			Mod: 修改速度环,方向环错误,车子速度已上两米
	*
	*		4.Author: Vector
	*			Date: 2018-3-25
	*			Mod: 车子能较稳定跑两米,参数:转向环,5600,0,18000,速度环:245,1,0
	*
	*		5.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 1.方向环速度变化率由放大10倍改为放大100倍,便于调节微分系数
	*					 2.方向环PID由常规PID改为不完全微分PID
	*					 3.新增弯道检测功能,增加新函数道路检测函数Car_RoadDetect
	*
	*		6.Author: Vector
	*			Date: 2018-3-29
	*			Mod: 1.速度环由双速度环改为单速度环,PID参数:转向 193,0,380,速度 245,1,0
	*
	*		7.Author: Vector
	*     Date: 2018-4-19
	*			Mod: 1.目标速度改用Vs = V + k*Error,比例因子:g_SpeedFactor
	*					 2.转向控制采用分段PID
	*					 3.增加第二排水平电感以及前排水平中间电感,采用前后两排的和差比的的差值来判断弯道情况
	*          4.增加识别圆环功能,采用前排中间电感,环岛准确率较高,但无环岛方向判断、出环岛处理
	*					 5.增加MPU功能,采用Z轴角速度辅助转向
	*
	*		8.Author: Vector
	*			Date: 2018-4-23
	*			Mod: 转向控制由分段PID改为模糊PID,效果较好
	*
	*		9.Author: Vector
	*			Date: 2018-4-28
	*			Mod: 1.将两个电机的速度环合为一个,且由位置式PID改为增量式PID
	*					 2.速度改为两个电机的平均转速
	*
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"
# include "app_debug.h"
# include "math.h"

/*  速度控制周期*/
# define SPEED_CONTROL_PERIOD	20	

/*  方向控制周期,单位ms  */
# define DIRCTION_CONTROL_PERIOD	5  


/*  小车控制结构体  */
Car_TypeDef Car;

Kalman1Dim_TypeDef Kalman_Gryoz;

/*  小车速度控制计数器  */
static uint16_t g_SpeedControlCounter = 0;
/*  小车速度输出控制周期  */
static uint16_t g_SpeedControlPeriod = 0;


/*  小车速度环控制PWM输出  */
static int16_t g_SpeedControlOutInc = 0;
static int16_t g_SpeedControlOutNew = 0;
static int16_t g_SpeedControlOutOld = 0;
static int16_t g_SpeedControlOut = 0;
static int16_t g_SpeedCounter = 0;
static float g_SpeedDeltaFactor = 0;

/*  小车方向控制变量  */
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlCounter = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;


/*  将Z轴角速度用于转向环Kd系数  */
const static float g_GryoZ_Kd = 0.09;


/*  跑出跑道计数器,当该值超过一定范围后停车  */
static uint16_t g_OutCounter = 0;

static float g_SpeedFactor = 0.38;//0.48;
static float g_HAEFactor = 0.08;

uint16_t g_CircleSpeedError = 0;


 

/*
*********************************************************************************************************
*                         Car_ParaInit                 
*
* Description: 车子参数初始化,从芯片Flash中读取出存储的电机参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_ParaInit(void)
{
	uint8_t i = 0;
	
		/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
	Car.PID.SpeedKp = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.SpeedKi = 0.13;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.SpeedKd = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  直道PID  */
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  初始化方向模糊PID参数  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 10;
	Car.DirFuzzy.DErrMax = 30;
	Car.DirFuzzy.ErrMax = 80;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = 38;//Car.PID.DirectionKp;
	Car.DirFuzzy.KIMax = 0;//Car.PID.DirectionKi;
	Car.DirFuzzy.KDMax = 480;//Car.PID.DirectionKd;
	fuzzy_PIDInit(&Car.DirFuzzy);
	
	/*  初始化速度控制模糊PID参数  */
	Car.LVFuzzy.DeltaKdMax = 20;
	Car.LVFuzzy.DeltaKiMax = 0.2;
	Car.LVFuzzy.DeltaKpMax = 30;
	Car.LVFuzzy.DErrMax = 50;
	Car.LVFuzzy.ErrMax = 50;
	Car.LVFuzzy.KP = 26;
	Car.LVFuzzy.KI = 0.1;
	Car.LVFuzzy.KD = 20;
	Car.LVFuzzy.KPMax = Car.PID.SpeedKp;
	Car.LVFuzzy.KIMax = Car.PID.SpeedKi;
	Car.LVFuzzy.KDMax = Car.PID.SpeedKd;
	fuzzy_PIDInit(&Car.LVFuzzy);
	
	/*  初始化速度控制模糊PID参数  */
	Car.RVFuzzy.DeltaKdMax = 20;
	Car.RVFuzzy.DeltaKiMax = 0.2;
	Car.RVFuzzy.DeltaKpMax = 30;
	Car.RVFuzzy.DErrMax = 50;
	Car.RVFuzzy.ErrMax = 50;
	Car.RVFuzzy.KP = 26;
	Car.RVFuzzy.KI = 0.1;
	Car.RVFuzzy.KD = 20;
	Car.RVFuzzy.KPMax = Car.PID.SpeedKp;
	Car.RVFuzzy.KIMax = Car.PID.SpeedKi;
	Car.RVFuzzy.KDMax = Car.PID.SpeedKd;
	fuzzy_PIDInit(&Car.RVFuzzy);
	
	
	/*  初始化车子的和差比  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  初始化车子的传感器参数,从Flash中读取标定值  */
	for(i = 0; i < SENSOR_COUNT; i ++)
	{
		Car.Sensor[i].FIFO[0] = 0;
		Car.Sensor[i].Read = 0;
		Car.Sensor[i].Write = 0;
		Car.Sensor[i].Average = 0;
		Car.Sensor[i].NormalizedValue = 0.0f;
		Car.Sensor[i].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2) * 4, uint16_t);
		Car.Sensor[i].CalibrationMin = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2 +  1 ) * 4, uint16_t);
	}
		

	/*  电机控制参数初始化  */
	Car.Motor.PWM_Frequency = 10;	/*  电机PWM频率为10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	/*  小车目标速度  */
	Car.TargetSpeed = 10;
		
	Car.MaxPWM = 950;
	
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
void Car_Running(void)
{
	bsp_led_Toggle(2);
}

/*
*********************************************************************************************************
*                                Car_ParaStore          
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
void Car_ParaStore(void)
{
	uint8_t temp[32] = {0};
	uint8_t cnt = 0;
	
	
	/*  保存车子整体目标速度  */
	temp[cnt++] = BYTE4((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.TargetSpeed);
	
	/*  保存车子左边目标速度  */
	temp[cnt++] = BYTE4((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.LeftTargetSpeed);
	
	/*  保存车子右边目标速度  */
	temp[cnt++] = BYTE4((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.RightTargetSpeed);
	
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);	/*  先擦除一遍,不然无法写入  */
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, temp, 48, 0);
}

/*
*********************************************************************************************************
*                           Car_ControlStop               
*
* Description: 停止小车控制,用于按键调参时暂停小车控制
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_ControlStop(void)
{
	DRV_DISABLE();							/*  关闭驱动  */
	bsp_tim_DeleteHardTimer(1);	/*  停止小车控制中断  */
}

/*
*********************************************************************************************************
*                        Car_Start                  
*
* Description: 重启小车控制,用于设置PID参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_ControlStart(void)
{
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
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
void Car_Reset(void)
{
	Car.PID.DirectionKp = DEFAULT_DIRECTION_KP;
	Car.PID.DirectionKd = DEFAULT_DIRECTION_KD;
	
	Car.PID.SpeedKp = DEFAULT_SPEED_KP;
	Car.PID.SpeedKd = DEFAULT_SPEED_KD;
	
	Car.TargetSpeed = DEFAULT_SPEED;
}

/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: 小车速度环控制,采用增量式PID
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_SpeedControl(void)
{		
	static  float SpeedErrorK_1=  0, SpeedErrorK_2 = 0;	
	float SpeedErrorK = 0;
	volatile int16_t LeftEnconder = 0, RightEnconder = 0;
	volatile float Kp = 0, Ki = 0, Kd = 0;
	
//	if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 20 || 
//		Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 20)
//	{
//		if(Car.HorizontalAE < 0)
//			Car.TargetSpeed = 10 + (g_SpeedCounter / 2) * 5 + g_SpeedFactor * Car.HorizontalAE;
//		else
//			Car.TargetSpeed = 10 + (g_SpeedCounter / 2) * 5 - g_SpeedFactor * Car.HorizontalAE;
//		
//		if(g_SpeedCounter++ >= 2) g_SpeedCounter = 0;

//	}
//	else 
//	if(Car.AE > 20 || Car.AE < -20)
//	{
//		if(Car.AE < 0)
//			Car.TargetSpeed = 10 + g_HAEFactor * Car.AE;
//		else
//			Car.TargetSpeed = 10 - g_HAEFactor * Car.AE;
//	}
//	else
//	{
//		Car.TargetSpeed = 10;
//	}
//	

//	/*  用于防止圆环有速度偏差  */
//	if(Car.Sensor[SENSOR_H_L].Average > 85 && Car.Sensor[SENSOR_H_R].Average > 85)
//	{
//		Car.TargetSpeed = 10;
//	}
	

	/*  因为两个编码器旋转了180度,所以极性相反  */
	LeftEnconder= 0;
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 0) ? (-Car.Motor.LeftEncoder) : Car.Motor.LeftEncoder;
	Car.Motor.LeftEncoder = 0;
	
	RightEnconder = 0;
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 1) ? (-Car.Motor.RightEncoder) : Car.Motor.RightEncoder;
	Car.Motor.RightEncoder = 0;
	
	/*  车子速度为左右两边取平均  */
	Car.CarSpeed = (LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
	
	SpeedErrorK = Car.TargetSpeed - Car.CarSpeed;
	
	Kp = 0;
	Ki = 0.12;
	Kd = 0;

//	g_SpeedControlOutInc =0;

	/*  增量式PID计算  */
	g_SpeedControlOutInc = Kp * (SpeedErrorK - SpeedErrorK_1) + 
												 Ki * SpeedErrorK + 
												 Kd * (SpeedErrorK - 2 * SpeedErrorK_1 + SpeedErrorK_2);
												 
	SpeedErrorK_2 = SpeedErrorK_1;											 
	SpeedErrorK_1 = SpeedErrorK;
	
}

/*
*********************************************************************************************************
*                              Car_SpeedControlOutput            
*
* Description: 将速度环的输出分成周期输出,防止速度突变
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_SpeedControlOutput(void)
{
	g_SpeedControlOut += g_SpeedControlOutInc;
	
	if(g_SpeedControlOut > 500) g_SpeedControlOut = 500;
	else if(g_SpeedControlOut < -500) g_SpeedControlOut = -500;
}


/*
*********************************************************************************************************
*                       Car_DirectionControl                   
*
* Description: 小车方向环控制函数,PID采用不完全微分PD控制,同时加入陀螺仪的角速度项
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirectionControl(void)
{
	static float LastError, KdOutLast = 0;
	static float k = 0.3;		/*  该值为不完全微分滤波系数  */
	float Error, ErrorDiff;		/*  偏差,偏差微分  */
	float Kp = 0, Kd = 0;		
	int16_t KpOut = 0, KdOutNow = 0, KdGryozOut = 0;

	Error = Car.HorizontalAE;
	ErrorDiff = Error - LastError;
	
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	/*  由于模糊PID算出来的值有负的,所以需要极性判断  */
	Kp = (Car.DirFuzzy.KP < 0) ? (-Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (-Car.DirFuzzy.KD) : Car.DirFuzzy.KD;
	

	/*  两边的电感值都小于阈值,说明出跑道了  */
	if(Car.Sensor[SENSOR_H_L].Average < 16 && Car.Sensor[SENSOR_H_R].Average < 16 && 
		Car.Sensor[SENSOR_V_L].Average < 16 && Car.Sensor[SENSOR_V_R].Average < 16)
		g_OutCounter++;
		
	KdOutLast = KdOutNow;		/*  由于采用不完全微分,所以需要保存上一时刻的微分  */
	KpOut = (Error) * Kp;		/*  转向环的比例  */
	KdOutNow = ErrorDiff * Kd;				/*  转向环的微分  */
	KdGryozOut = g_GryoZ_Kd * (Car.MPU.Gryoz - MPU_GRYOZ_ZERO);		/*  采用陀螺仪的角速度进行补偿,抵制转向  */
		
	
	/*  保存上次的PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  计算PWM,采用不完全微分PID  */
	g_DirectionControlOutNew = (int16_t)(k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut);
	
	/*  当偏差小于一定范围时关闭输出,可以减少系统振荡  */
	if(Car.HorizontalAE < 4 && Car.HorizontalAE > -4) g_DirectionControlOutNew = 0;

	/*  保存上个时刻的误差  */
	LastError = Car.HorizontalAE;
}

/*
*********************************************************************************************************
*                      Car_DirectionControlOutput                    
*
* Description: 方向环控制输出,将方向环的输出均分为控制周期的n等份输出,让速度平滑
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirectionControlOutput(void)
{
	int16_t DirectionOutput = 0;
	
	DirectionOutput = g_DirectionControlOutNew - g_DirciotnControlOutOld;
	g_DirectionControlOut = (int16_t)(DirectionOutput * (g_DirectionControlPeriod + 1) / DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld);
}

/*
*********************************************************************************************************
*                               Car_MotorOutput           
*
* Description: 将最后的速度叠加后输出到电机
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_MotorOutput(void)
{
	int16_t LeftPwm = 0, RightPwm = 0;
	
		/*  将速度环和转向环的PWM叠加起来  */
	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut - g_CircleSpeedError);
	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut + g_CircleSpeedError);	
	
//		if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 20 )
//	{
//		/*  将速度环和转向环的PWM叠加起来  */
//		LeftPwm = (int16_t)((1 + 0.3) *LeftPwm);
//		RightPwm = (int16_t)((1 + 0.3) * RightPwm);
//	}
//	else if(Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 20 )
//	{
//		/*  将速度环和转向环的PWM叠加起来  */
//		LeftPwm = (int16_t)((1 + 0.3) * LeftPwm);
//		RightPwm = (int16_t)((1 + 0.3) * RightPwm);		
//	}
	
		
	/*  限幅  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
		
	/*  当丢线计数器的值大于2时,说明已经冲出了跑道,停车  */
	if(g_OutCounter > 2)
		bsp_motor_SetPwm(0,0);
	else
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
	/*  输出到电机  */
		
}


/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: 小车总控制函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_Control(void)
{
	static uint16_t CarControlCunter = 0;	
	
	/*  控制计数器  */
	CarControlCunter++;
	
	/*  速度控制输出  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  方向控制输出  */
	g_DirectionControlPeriod++;
	Car_DirectionControlOutput();
	
	/*  控制状态机  */
	switch(CarControlCunter)
	{
		/*  每5ms读取一次编码器  */
		case 1: bsp_encoder_ReadCounter();break;
		
		/*  速度控制  */
		case 2:
		{
			g_SpeedControlCounter++;
			if(g_SpeedControlCounter >= SPEED_CONTROL_PERIOD/5)
			{
				g_SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
				Car_SpeedControl();
			}
		}break;/*  end of case 2  */
		
		
		/*  每5ms进行一次传感器数据处理  */
		case 3:bsp_sensor_DataProcess();break;
		
		/*  方向控制  */
		case 4:
		{
			g_DirectionControlCounter++;
			if(g_DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
			{
				g_DirectionControlCounter = 0;
				g_DirectionControlPeriod = 0;
				Car_DirectionControl();
			}
		}break;
		
		/*  每5ms进行一次电机输出  */
		case 5:
		{
			Car_MotorOutput();
			CarControlCunter=0;
		}break;
		default:
		{
			CarControlCunter = 0;
			g_DirectionControlCounter = 0;
			g_SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirectionControlPeriod = 0;
		}break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

