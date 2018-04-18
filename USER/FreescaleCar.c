/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.3.0
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
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"
# include "app_debug.h"

/*  小车控制结构体  */
Car_TypeDef Car;

/*  小车速度控制计数器  */
static uint16_t g_SpeedControlCounter = 0;
/*  小车速度输出控制周期  */
static uint16_t g_SpeedControlPeriod = 0;


/*  小车速度环控制PWM输出  */
//static int16_t g_SpeedControlOutNew = 0;
//static int16_t g_SpeedControlOutOld = 0;
//static int16_t g_SpeedControlOut = 0;

static int16_t g_LeftSpeedControlOutNew = 0;
static int16_t g_LeftSpeedControlOutOld = 0;
static int16_t g_LeftSpeedControlOut = 0;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static int16_t g_RightSpeedControlOut = 0;
static int16_t g_SpeedCounter = 0;

/*  小车方向控制变量  */
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlCounter = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;


/*  将Z轴角速度用于转向环Kd系数  */
const static float g_GryoZ_Kd = 0.09f;

/*  丢线计数器,由该变量来统计偏离中线一定范围的次数,该值越小,说明偏离中线越小  */
static int16_t g_LossLineCounter = 0;

static float g_CarCurveSpeedLow = 0;
static float g_CarCurveSpeedHigh = 0;
static float g_CarStraightSpeed = 0;

static uint8_t g_CarLastDirection = 0;

/*  跑出跑道计数器,当该值超过一定范围后停车  */
static uint16_t g_OutCounter = 0;
static uint16_t g_StaightFlag = 0;
static uint8_t g_LeftCurveFlag = 0;
static uint8_t g_RightCurveFlag = 0;

static float g_SpeedFactor = 0.08;

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
	
	/*  初始化车子的和差比  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
		
	Car.PID.SpeedKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.SpeedKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.SpeedKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  直道PID  */
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
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
	Car.TargetSpeed = 25;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, float);
//	Car.LeftTargetSpeed = 20;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 4, float);
//	Car.RightTargetSpeed = 40;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 8, float);
		
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
*                       Car_LeftVelocityPIDCalc                   
*
* Description: 小车左轮速度环PID计算
*             
* Arguments  : 1> LeftSpeed: 左轮速度
*
* Reutrn     : 1> 计算得到的左轮PWM
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t Car_LeftVelocityPIDCalc(int16_t LeftSpeed)
{
	static float SpeedFilter, SpeedIntegal, LastSpeedError = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  速度偏差  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed);
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 100 || SpeedError >= -100)
		SpeedIntegal += SpeedFilter;
	
	/*  积分限幅  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//积分限幅
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
//		if(SpeedError > 3)
//	{
//		/*  速度环PD控制,实际上Ki = 0  */
//		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
//								SpeedIntegal * (-Car.PID.SpeedKi));//速度环PID计算	
//	}else
//	{
				/*  速度环PD控制,实际上Ki = 0  */
		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
								SpeedIntegal * (-Car.PID.SpeedKi)) + (LastSpeedError - SpeedError) * Car.PID.SpeedKd;
//	}
		
	LastSpeedError = SpeedError;
	return Velocity;
}

/*
*********************************************************************************************************
*                       Car_RightVelocityPIDCalc                   
*
* Description: 小车右轮速度环计算
*             
* Arguments  : 1> RightSpeed: 右轮速度
*
* Reutrn     : 1> 计算得到的PWM
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t Car_RightVelocityPIDCalc(int16_t RightSpeed)
{
	static float SpeedFilter, SpeedIntegal , LastSpeedError;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  速度偏差  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed);
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 10 || SpeedError >= -10)
		SpeedIntegal += SpeedFilter;
	
	/*  积分限幅  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//积分限幅
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
	Velocity = SpeedFilter * (-Car.PID.SpeedKp) +
						SpeedIntegal * (-Car.PID.SpeedKi) - (LastSpeedError - SpeedError) * Car.PID.SpeedKd;	
	
	LastSpeedError = SpeedError;
	return Velocity;
}
/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: 小车速度环控制,采用积分分离PI控制
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
	static float SpeedFilter, SpeedIntegal;
	static uint8_t flag = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	if(Car.Sensor[SENSOR_H_L].Average > 110 && Car.Sensor[SENSOR_H_R].Average > 110)
	{
		Car.LeftTargetSpeed = Car.TargetSpeed;
		Car.RightTargetSpeed = Car.TargetSpeed;
	}
	else
	{
//		if(Car.HorizontalAE - Car.VecticalAE > 20 && flag == 0)  flag = 10;
//		
//		if(flag > 0)
//		{
//			if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 15)
//			{
//				/*  由当前误差来动态计算左右电机目标速度  */
//				Car.LeftTargetSpeed = Car.TargetSpeed + g_SpeedFactor * Car.HorizontalAE * 3;
//				Car.RightTargetSpeed = Car.TargetSpeed - g_SpeedFactor * Car.HorizontalAE;
//			}
//			else if(Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 15)
//			{
//				/*  由当前误差来动态计算左右电机目标速度  */
//				Car.LeftTargetSpeed = Car.TargetSpeed + g_SpeedFactor * Car.HorizontalAE;
//				Car.RightTargetSpeed = Car.TargetSpeed - g_SpeedFactor * Car.HorizontalAE*3;
//			}
//			flag --;
//		}
//		else
//		{
			/*  由当前误差来动态计算左右电机目标速度  */
			Car.LeftTargetSpeed = Car.TargetSpeed + g_SpeedFactor * Car.HorizontalAE;
			Car.RightTargetSpeed = Car.TargetSpeed - g_SpeedFactor * Car.HorizontalAE;
//		}
	}
	
	/*  左边电机速度环计算  */
	Car.Motor.LeftSpeed = (float)(Car.Motor.LeftEncoder * CAR_SPEED_CONSTANT);
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	Car.Motor.LeftEncoder = 0;
	
	
	g_LeftSpeedControlOutOld = g_LeftSpeedControlOutNew;
	g_LeftSpeedControlOutNew = Car_LeftVelocityPIDCalc(Car.Motor.LeftSpeed); 
	
	/*  右边电机速度环计算  */
	Car.Motor.RightSpeed = (float)(Car.Motor.RightEncoder * CAR_SPEED_CONSTANT);
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	Car.Motor.RightEncoder = 0;
	
	
	g_RightSpeedControlOutOld = g_RightSpeedControlOutNew;
	g_RightSpeedControlOutNew = Car_RightVelocityPIDCalc(Car.Motor.RightSpeed);
	
//	Car.CarSpeed = (Car.Motor.LeftSpeed + Car.Motor.RightSpeed)/2;
//	
//		/*  速度偏差  */
//	SpeedError = (float)(Car.CarSpeed - Car.TargetSpeed);
//	
//	/*  低通滤波,让速度平滑过渡  */
//	SpeedFilter *= 0.7;
//	SpeedFilter += (SpeedError * 0.3);
//	
//	if(SpeedError < 10 || SpeedError >= -10)
//		SpeedIntegal += SpeedFilter;
//	
//	/*  积分限幅  */
//	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//积分限幅
//	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
//	
//	if(SpeedError > 3)
//	{
//		/*  速度环PI控制  */
//		g_SpeedControlOutOld = g_SpeedControlOutNew;
//		g_SpeedControlOutNew = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp*4) + SpeedIntegal * (-Car.PID.SpeedKi));//速度环PID计算	
//	}else
//	{
//				/*  速度环PI控制 */
//		g_SpeedControlOutOld = g_SpeedControlOutNew;
//		g_SpeedControlOutNew = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +	SpeedIntegal * (-Car.PID.SpeedKi));
//	}
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
	int16_t SpeedControlValue;
	
//	SpeedControlValue = g_SpeedControlOutNew - g_SpeedControlOutOld;
//	g_SpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_SpeedControlOutOld;
	/*  计算左边电机输出量  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
	/*  计算右边电机输出量  */
	SpeedControlValue = g_RightSpeedControlOutNew - g_RightSpeedControlOutOld;
	g_RightSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_RightSpeedControlOutOld;
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
	static int16_t LastError, Error, KdOutLast = 0;
	float Kp = 0, Kd = 0;
	int16_t KpOut = 0, KdOutNow = 0;
	int16_t KdGryozOut = 0;
	static float k = 0.32f;
	

	
	/*  如果右边的电感归一化值大于左边的电感,说明车子在右转  */
//	if(Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue >=20) g_CarLastDirection = 1;

//	/*  车子在左转  */
//	if(Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue >=20) g_CarLastDirection = 0;
	

	

	Error = Car.HorizontalAE - LastError;

	
//	if(Car.LeftTargetSpeed == 25)
//	{
	Kp = Car.PID.DirectionKp;
	Kd = Car.PID.DirectionKd;
//	}else if(Car.LeftTargetSpeed == 35)
//	{
//		Kp = 90;
//		Kd = 180;
//	}
//	if(f_abs(Car.HorizontalAE) < 30) Kp = Car.PID.DirectionKp * 0.4;
//	else if(f_abs(Car.HorizontalAE) < 30) Kp = Car.PID.DirectionKp * 0.6;
//	else if(f_abs(Car.HorizontalAE) < 40) Kp = Car.PID.DirectionKp * 0.7;
//	else if(f_abs(Car.HorizontalAE) < 50) Kp = Car.PID.DirectionKp * 0.8;
//	else 
//	Kp = Car.PID.DirectionKp;
//	
//	if(f_abs(Error) < 3) Kd = Car.PID.DirectionKd * 0.4;
//	else if(f_abs(Error) < 5) Kd = Car.PID.DirectionKd * 0.8;
//	else 
//	Kd = Car.PID.DirectionKd;
	
	/*  两边的电感值都小于阈值,说明出跑道了  */
	if(Car.Sensor[SENSOR_H_L].Average < 9 && Car.Sensor[SENSOR_H_R].Average < 9)
		g_OutCounter++;
		
	
	KdOutLast = KdOutNow;		/*  由于采用不完全微分,所以需要保存上一时刻的微分  */
	
	

	

		KpOut = (Car.HorizontalAE - 0) * Kp;		/*  转向环的比例  */
	KdOutNow = Error * Kd;				/*  转向环的微分  */
	KdGryozOut = g_GryoZ_Kd * (Car.MPU.Gryoz - MPU_GRYOZ_ZERO);		/*  采用陀螺仪的角速度进行补偿,抵制转向  */
		
	
	/*  保存上次的PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  计算PWM,采用不完全微分PID  */
	g_DirectionControlOutNew = (int16_t)(k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut);
	
	/*  当偏差小于一定范围时关闭微分,可以减少系统振荡  */
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
		static uint8_t firstFlag = 0, secondFlag = 0, thirdFlag = 0;
	static uint32_t firstTime = 0, secondTime = 0, thirdTime = 0, circle = 0, circlecnt = 0, flag = 0;
	
	/*  将速度环和转向环的PWM叠加起来  */
	LeftPwm = (int16_t)(g_LeftSpeedControlOut + g_DirectionControlOut);
	RightPwm = (int16_t)(g_RightSpeedControlOut - g_DirectionControlOut + g_CircleSpeedError);
	
//	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut);
//	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut);
	
	/*  限幅  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
		
	
//	if(Car.Sensor[SENSOR_M].Average == 110) 
//	{
//		firstFlag ++;
//		firstTime = bsp_tim_GetRunTime();
//	}
//	
//	if(Car.Sensor[SENSOR_M].Average == 118 && firstFlag > 2)
//	{
//		secondTime = bsp_tim_GetRunTime();
//		secondFlag ++;
//		//if(secondTime - firstTime > 100)
//	}
//		if(Car.Sensor[SENSOR_M].Average == 110 && secondFlag > 2)
//	{
//		bsp_led_ON(1);
//		thirdFlag = 1;
//		thirdTime = bsp_tim_GetRunTime();

//			firstFlag = 0;
//			secondFlag = 0;
//	}

//	if(Car.Sensor[SENSOR_H_L].Average > 110 && Car.Sensor[SENSOR_H_R].Average > 110 && Car.Sensor[SENSOR_M].Average > 110)
//	{
////		
//			bsp_motor_SetPwm(300, 400);
////		bsp_tim_DelayMs(100);
//		for(circle=0;circle < 2000; circle++)
//		{
//			for(circlecnt = 0; circlecnt < 5000; circlecnt++);
//		}
//	}
//	else
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
	static uint16_t circle = 0;
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
	

