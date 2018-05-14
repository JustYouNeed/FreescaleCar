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
	*			Mod: 修改速度环,方向环错误
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
	*		10.Author: Vector
	*			 Date: 2018-4-29
	*			 Mod: 重写速度环,发现速度不受控制,速度环不理想
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
# define SPEED_CONTROL_PERIOD	  30	

/*  方向控制周期,单位ms  */
# define DIRCTION_CONTROL_PERIOD	5

/*  车子出跑道后的电感值  */
# define LOST_LINE_THRESHOLD		16

/*  速度转换比例因子,计算完成后速度单位为 转速  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)*ENCONDER_TEETH/WHEEL_TEETH

/*  定义速度环控制方式,方式一为位置式,二为增量式  */
# define SPEED_CONTROL_METHOD		1

/*  速度环输出的最大值,对应占空比60%  */
# define SPEED_CONTROL_MAX			600
/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/

/*  小车控制结构体  */
Car_TypeDef Car;


uint8_t g_SpeedControlON = 1;		/*  速度控制开关  */

/*  小车速度环控制PWM输出  */
static float g_SpeedControlOut = 0;						/*  最终的速度环输出  */
static float g_SpeedControlOutNew = 0;				/*  本次速度环的输出  */
static float g_SpeedControlOutOld = 0;				/*  上次速度环的输出  */
static uint16_t g_SpeedControlPeriod = 0;			/*  速度控制周期计数器,用于将速度环的输出平滑处理  */
static int16_t g_SpeedControlBangBang = 0;		/*  棒棒控制变量  */

/*  小车方向控制变量  */
static float g_DirectionControlOut = 0;				/*  最终方向环的输出  */
static float g_DirectionControlOutNew = 0;		/*  本次方向环的输出  */
static float g_DirciotnControlOutOld = 0;			/*  上次方向环的输出  */
static uint16_t g_DirectionControlPeriod = 0;	/*  方向控制周期计数器,用于将方向控制的输出平滑处理  */

static int16_t g_CurveSpeedControl = 0;
static uint8_t g_CurveStatus = 0;

static uint8_t g_NeedEnterCurve = 0;
static uint8_t g_NeedOutCurve = 0;

static uint8_t g_AlreadyEnterCurve = 0;

/*  将Z轴角速度用于转向环Kd系数  */
const static float g_GryoZ_Kd = 0.1;


/*  跑出跑道计数器,当该值超过一定范围后停车  */
static uint16_t g_LoseLineCounter = 0;

static float g_SpeedFactor = 0.16;//0.48;
static float g_HAEFactor = 0.09;

uint16_t g_CircleSpeedError = 0;

static float g_CurveDirection = 0;
static float g_CurveOffset = 0;

 

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
	float VelKp = 0, VelKi = 0, VelKd = 0;
	
	/*  首先将车子控制结构体的各个参数归零  */
	*(uint8_t*)&Car = 0;
	
# if SPEED_CONTROL_METHOD == 1
	/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
	VelKp = 80.5;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	VelKi = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	VelKd = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);

# elif SPEED_CONTROL_METHOD == 2
	
	VelKp = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	VelKi = 0.8;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	VelKd = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
# endif
//	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  直道PID  */
//	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
//	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  初始化方向模糊PID参数  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 15;
	Car.DirFuzzy.DErrMax = 30;
	Car.DirFuzzy.ErrMax = 90;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = 28;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	Car.DirFuzzy.KIMax = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.DirFuzzy.KDMax = 480;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	fuzzy_PIDInit(&Car.DirFuzzy);
	
	/*  初始化速度PID,只采用比例控制,无积分微分  */
	pid_PIDInit(&Car.VelPID, VelKp, VelKi, VelKd, 0, 0);
				
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
	
	Car.Sensor[SENSOR_V_L].CalibrationMax = 100;
	Car.Sensor[SENSOR_V_R].CalibrationMax = 100;
	Car.Sensor[SENSOR_V_L].CalibrationMin = 0;
	Car.Sensor[SENSOR_V_R].CalibrationMin = 0;
	/*  小车目标速度  */
	Car.TargetSpeed = 18;//(float)drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, uint32_t);;
		
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
	float temp[8] = {0};
	uint8_t cnt = 0;
	
	
	/*  保存车子整体目标速度  */
	temp[cnt++] = Car.TargetSpeed;
	
	DISABLE_INT();
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);	/*  先擦除一遍,不然无法写入  */
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t *)temp, cnt * 4, 0);
	ENABLE_INT();
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
void Car_GetVoltage(void)
{
	uint16_t adc = 0;
	
	adc = drv_adc_ConvOnce(BAT_CHANNEL, ADC_Resolution_8b);
	Car.Voltage = (adc* 5.0f/ 255) / 0.379f ;
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
	bsp_tim_CreateHardTimer(0, 1, Car_Control);
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
	Car.VelPID.Kp = DEFAULT_SPEED_KP;
	Car.VelPID.Kd = DEFAULT_SPEED_KD;
	
	Car.TargetSpeed = DEFAULT_SPEED;
}



/*
*********************************************************************************************************
*                           Car_SpeedControl               
*
* Description: 车子速度环控制
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
	static float LastError = 0;
	volatile int32_t LeftEnconder = 0, RightEnconder = 0;
	volatile float SpeedError = 0;

	
	/*  由于两个编码器旋转了180度,所以有一个极性差  */
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 1) ? (ftm_count_get(ftm1)) : (-ftm_count_get(ftm1));
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 0) ? (ftm_count_get(ftm0)) : (-ftm_count_get(ftm0));
	ftm_count_clean(ftm0);
	ftm_count_clean(ftm1);
	
	Car.Motor.LeftEncoder = LeftEnconder;
	Car.Motor.RightEncoder = RightEnconder;
	
	/*  将速度进行转换,计算成转/秒  */
	Car.CarSpeed = (float)(LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
	
	
	SpeedError = Car.TargetSpeed - Car.CarSpeed;
	
	
/*  如果选择位置式  */
# if SPEED_CONTROL_METHOD==1
	/*  计算输出  */
	g_SpeedControlOutOld = g_SpeedControlOutNew;
	g_SpeedControlOutNew = pid_PositionalCalc(&Car.VelPID, SpeedError);
	
# elif SPEED_CONTROL_METHOD==2		/*  增量式  */
	g_SpeedControlOut += pid_IncrementalCalc(&Car.VelPID, SpeedError);
	
	/*  对速度环的输出进行限幅  */
	if(g_SpeedControlOut > SPEED_CONTROL_MAX) g_SpeedControlOut = SPEED_CONTROL_MAX;
	else if(g_SpeedControlOut < -SPEED_CONTROL_MAX) g_SpeedControlOut = -SPEED_CONTROL_MAX;
# endif
//	if(SpeedError < -3) g_SpeedControlBangBang = -900;
//	else if(SpeedError > 5) g_SpeedControlBangBang = 100;
//	else g_SpeedControlBangBang = 0;
	
	LastError = SpeedError;
}

/*
*********************************************************************************************************
*                               Car_SpeedControlOutput           
*
* Description: 小车速度控制平滑输出函数,将速度环的输出分成多个周期输出,让速度变化更平滑
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
# if SPEED_CONTROL_METHOD == 1	/*  位置式PID  */
	volatile float SpeedControlOut = 0;

	SpeedControlOut = g_SpeedControlOutNew - g_SpeedControlOutOld;
	g_SpeedControlOut = SpeedControlOut * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_SpeedControlOutOld;

	/*  对速度环的输出进行限幅  */
	if(g_SpeedControlOut > SPEED_CONTROL_MAX) g_SpeedControlOut = SPEED_CONTROL_MAX;
	else if(g_SpeedControlOut < -SPEED_CONTROL_MAX) g_SpeedControlOut = -SPEED_CONTROL_MAX;
# endif
}

/*
*********************************************************************************************************
*                       Car_RoadDetect                   
*
* Description: 检测道路情况,判断小车当前道路
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_RoadDetect(void)
{
	static uint32_t FirstTime = 0;
	static float AngleTemp = 0;
	static uint8_t FirstPointFlag = 0, SecondPointFlag = 0;
	
		/*  两边的电感值都小于阈值,说明出跑道了  */
	if(Car.Sensor[SENSOR_H_L].Average < LOST_LINE_THRESHOLD && Car.Sensor[SENSOR_H_R].Average < LOST_LINE_THRESHOLD )
		g_LoseLineCounter++;
	

	
	
	/*  该标志为0,说明还没有找到第一个标志点  */	
	if(Car.Sensor[SENSOR_M].Average > 45 )
	{
		if(g_CurveStatus == 0 && g_NeedOutCurve == 0)		/*  表示是第一次进入圆环区域,接下来需要判断方向  */
		{
			if(Car.VecticalAE < -40)		/*  小于零,说明是左边圆环  */
			{
				FirstPointFlag = 1;
				
				Car.NowRoad = LEFT_ISLAND;
				g_CurveOffset = 0;		/*  防止上次未清除偏移量  */
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
			}
			else if(Car.VecticalAE > 40)		/*  两个垂直电感的和差比大于零,说明是右边圆环  */
			{
				Car.NowRoad = RIGHT_ISLAND;
				g_CurveOffset = 0;					/*  防止上次未清除偏移量  */
				
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
			}
		}
	}
	/*  已经找到第一个标志点了,需要找到第二个标志点,第二个标志点为中间水平电感的峰值  */
	if(g_CurveStatus == 1)	
	{
		if(Car.NowRoad == LEFT_ISLAND) /*  找到中间回归零点的值  */
		{
			if(Car.VecticalAE >= 0 && g_NeedOutCurve == 0) 
			{
				bsp_led_ON(LED_RED);
				g_CurveStatus = 2;
				AngleTemp = Car.MPU.Yaw;
				g_NeedEnterCurve = 1;
			}
		}
		else if(Car.NowRoad == RIGHT_ISLAND)
		{
			if(Car.VecticalAE <= 0 && g_NeedOutCurve == 0) 
			{
				g_NeedEnterCurve = 1;
				g_CurveStatus = 2;
				AngleTemp = Car.MPU.Yaw;
				bsp_led_ON(LED_BLUE);
			}
		}
	}
	
	if(fabs(Car.MPU.Yaw - AngleTemp) > 40 && g_NeedEnterCurve == 1) 
	{
		g_NeedOutCurve = 1;
		g_NeedEnterCurve = 0;
		g_CurveOffset = 0;
		
	}
	if(fabs(Car.MPU.Yaw - AngleTemp) > 300 && g_NeedOutCurve == 1) Car.TargetSpeed = 10;
	
	if(Car.Sensor[SENSOR_M].Average < 45 && Car.Sensor[SENSOR_H_L].Average < 95 && Car.Sensor[SENSOR_H_R].Average < 95)
	{
		g_NeedEnterCurve = 0;
		g_CurveOffset = 0;
		g_CurveStatus = 0;
		Car.TargetSpeed = 18;
		bsp_led_OFF(LED_ALL);
	}
	
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
	static float LastError = 0, KdOutLast = 0;
	static const float k = 0.3;		/*  该值为不完全微分滤波系数  */
	float Error, ErrorDiff;		/*  偏差,偏差微分  */
	float Kp = 0, Kd = 0, Gyro_Z;		
	float KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
	static float temp = 0;
	
	switch(Car.NowRoad)
	{
		case STRAIGHT:temp = 0; g_AlreadyEnterCurve = 0; 
		g_CurveOffset = 0; g_NeedEnterCurve = 0; g_NeedOutCurve = 0; g_CurveSpeedControl = 0; break;
		case LEFT_CURVE:break;
		case RIGHT_CURVE:break;
		case LEFT_ISLAND:
		{
			if(g_NeedEnterCurve == 1) 	/*  只有在需要进入圆环且没有进入圆环时需要处理  */
			{
//				bsp_beep_ON();
				g_CurveOffset = -30;
			}
		}break;
		case RIGHT_ISLAND:
		{			
			if(g_NeedEnterCurve == 1) 
			{
//				bsp_beep_ON();
				g_CurveOffset = 30;
			}
		}break;
	}
	
	Gyro_Z = Car.MPU.Gyroz - MPU_GYROZ_ZERO;
	
	Error = Car.HorizontalAE + g_CurveOffset;
	
	/*  偏差微分  */
	ErrorDiff = Error - LastError;
	
	/*  用偏差以及偏差的微分计算PID  */
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	/*  由于模糊PID算出来的值有负的,所以需要极性判断  */
	Kp = (Car.DirFuzzy.KP < 0) ? (-Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (-Car.DirFuzzy.KD) : Car.DirFuzzy.KD;
			
	KdOutLast = KdOutNow;		/*  由于采用不完全微分,所以需要保存上一时刻的微分  */
	KpOut = Error * Kp;		/*  转向环的比例  */
	KdOutNow = ErrorDiff * Kd;				/*  转向环的微分  */
	KdGryozOut = g_GryoZ_Kd * Gyro_Z;		/*  采用陀螺仪的角速度进行补偿,抵制转向  */
	
	/*  保存上次的PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  计算PWM,采用不完全微分PID  */
	g_DirectionControlOutNew = k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut;
	
	/*  当偏差小于一定范围时关闭输出,可以减少系统振荡  */
	if(Car.HorizontalAE < 4 && Car.HorizontalAE > -4) g_DirectionControlOutNew = 0;

	/*  保存上个时刻的误差  */
	LastError = Error;
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
	float DirectionOutput = 0;
	
	DirectionOutput = g_DirectionControlOutNew - g_DirciotnControlOutOld;
	g_DirectionControlOut = DirectionOutput * (g_DirectionControlPeriod + 1) / DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld;
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
	volatile int16_t LeftPwm = 0, RightPwm = 0;
	
	
	/*  将速度环和转向环的PWM叠加起来,速度环+方向环+棒棒控制+圆环控制  */
	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut + g_SpeedControlBangBang + g_CurveSpeedControl);
	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut + g_SpeedControlBangBang - g_CurveSpeedControl);	
		
	/*  限幅  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
		
	/*  当丢线计数器的值大于2时,说明已经冲出了跑道,停车  */
	if(g_LoseLineCounter > 2)
		bsp_motor_SetPwm(0,0);
	else
		bsp_motor_SetPwm(LeftPwm, RightPwm);	/*  输出到电机  */	
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
	static uint16_t CarControlCounter = 0;	
	static uint16_t SpeedControlCounter = 0;
	static uint16_t DirectionControlCounter = 0;
	
	/*  控制计数器  */
	CarControlCounter++;
	
	/*  速度控制输出  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  方向控制输出  */
	g_DirectionControlPeriod++;
	Car_DirectionControlOutput();
	
	/*  控制状态机  */
	switch(CarControlCounter)
	{
		/*  每5ms读取一次编码器  */
		case 1:
		{
			SpeedControlCounter++;
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5 && g_SpeedControlON == 1)
			{
				SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
				Car_SpeedControl();				/*  运行时长68us  */
			}
		}break;
		
		/*  速度控制  */
		case 2:
		{

		}break;/*  end of case 2  */
		
		
		/*  每5ms进行一次传感器数据处理  */
		case 3:
		{
			bsp_sensor_DataProcess();		/*  运行时长64us  */
			Car_RoadDetect();
		}break;
		
		/*  方向控制  */
		case 4:
		{
			DirectionControlCounter++;
			if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
			{
				DirectionControlCounter = 0;
				g_DirectionControlPeriod = 0;
				Car_DirectionControl();		/*  运行时长600us  */
			}
		}break;
		
		/*  每5ms进行一次电机输出  */
		case 5:
		{
			Car_MotorOutput();
			CarControlCounter=0;
		}break;
		default:		/*  程序如果跑到这里了,说明程序出错了,停车处理  */
		{
			CarControlCounter = 0;
			DirectionControlCounter = 0;
			SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirectionControlPeriod = 0;
			DRV_DISABLE();				/*  关闭电机驱动  */
			while(1);							
		}break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

