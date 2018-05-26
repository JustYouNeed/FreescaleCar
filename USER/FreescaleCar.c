/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V2.3.0
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
	*					 3.增加第二排水平电感以及前排中间电感,采用前后两排的差比和的差值来判断弯道情况
	*          4.增加圆环识别功能,采用前排中间电感,环岛准确率较高,但无环岛方向判断、出环岛处理
	*					 5.增加MPU功能,采用Z轴角速度辅助转向
	*
	*		8.Author: Vector
	*			Date: 2018-4-23
	*			Mod: 转向为分段PID改为模糊PID,效果较好
	*
	*		9.Author: Vector
	*			Date: 2018-4-28
	*			Mod: 1.将两个电机的速度环合为一个,且由位置式PID改为增量式PID
	*					 2.速度改为两个的平均转速
	*
	*		10.Author: Vector
	*			 Date: 2018-4-29
	*			 Mod: 重写速度环,发现速度环不受控制,速度环不理想
	*
	*		11.Author: Vector
	*			 Date: 2018-5-20
	*			 Mod: 1.因程序出现未知错误,车子会突然停车,且原因不明,恢复为4.19的程序
	*					  2.更改圆环识别方式,圆环识别的准确率达到90%以上,效果较好	*						
	*
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"

/*  速度控制周期*/
# define SPEED_CONTROL_PERIOD	  50	

/*  方向控制周期,单位ms  */
# define DIRCTION_CONTROL_PERIOD	5

/*  车子出跑道后的电感值  */
# define LOST_LINE_THRESHOLD		10

/*  速度转换比例因子,计算完成后速度单位为 转速  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)*ENCONDER_TEETH/WHEEL_TEETH

/*  小车控制结构体  */
Car_TypeDef Car;

/*  小车速度控制计数器  */
static uint16_t g_SpeedControlCounter = 0;
/*  小车速度输出控制周期  */
static uint16_t g_SpeedControlPeriod = 0;


/*  小车速度环控制PWM输出  */
static int16_t g_LeftSpeedControlOutNew = 0;
static int16_t g_LeftSpeedControlOutOld = 0;
static int16_t g_LeftSpeedControlOut = 0;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static int16_t g_RightSpeedControlOut = 0;

/*  小车方向控制变量  */
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;

static float g_TargetSpeed = 0;
/*  将Z轴角速度用于转向环Kd系数  */
const static float g_GryoZ_Kd = 0.1f;

/*  丢线计数器,由该变量来统计偏离中线一定范围的次数,该值越小,说明偏离中线越小  */
static int16_t g_LossLineCounter = 0;
static float g_SpeedFactor = 0.12;
static float g_IslandOffset = 0.0f;

/*  丢线积分  */
static float g_LeftLostLineInteral = 0.0f;
static float g_RightLostLineInteral = 0.0f;
static float g_LostLineRecoup = 0.0f;

static float g_DirKp = 0;
static float g_DirKi = 0;
static float g_DirKd = 0;

/*  圆环检测状态机  */
static IslandStatus_EnumTypeDef g_IslandStatus = CHECK_POINT_A;

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
	
	/*  初始化车子的和差比  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
	VelKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	VelKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	VelKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);

	/*  初始化速度PID,只采用比例控制,无积分微分  */
	pid_PIDInit(&Car.VelPID, VelKp, VelKi, VelKd, 0, 0);
	
	
	g_DirKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	g_DirKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	g_DirKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	/*  初始化方向模糊PID参数  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 15;
	Car.DirFuzzy.DErrMax = 50;
	Car.DirFuzzy.ErrMax = 160;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = g_DirKp;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	Car.DirFuzzy.KIMax = g_DirKi;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.DirFuzzy.KDMax = g_DirKd;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	fuzzy_PIDInit(&Car.DirFuzzy);
	
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
	
	Car.Sensor[SENSOR_V_L].CalibrationMax = 150;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2) * 4, uint16_t);
	Car.Sensor[SENSOR_V_L].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2 +  1 ) * 4, uint16_t);
	
	Car.Sensor[SENSOR_V_R].CalibrationMax = 150;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2) * 4, uint16_t);
	Car.Sensor[SENSOR_V_R].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2 +  1 ) * 4, uint16_t);
		
	/*  电机控制参数初始化  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	
	
	/*  小车目标速度  */
	Car.TargetSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, float);
	g_TargetSpeed = Car.TargetSpeed;
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
	bsp_led_Toggle(LED_BLUE);
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
	float temp[4] = {0};
	uint8_t cnt = 0;
	
	
	/*  保存车子整体目标速度  */
	temp[cnt++] = Car.TargetSpeed;
	
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);	/*  先擦除一遍,不然无法写入  */
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t*)&temp, cnt * 4, 0);
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
	static float SpeedFilter = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  速度偏差  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed);
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	/*  速度环PD控制,实际上Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * -Car.VelPID.Kp);
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
	static float SpeedFilter = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  速度偏差  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed);
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	Velocity = (int16_t)(SpeedFilter * -Car.VelPID.Kp);;
	
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
	if(Car.Sensor[SENSOR_H_L].Average > 50 && Car.Sensor[SENSOR_H_R].Average > 50)
	{
		Car.LeftTargetSpeed = Car.TargetSpeed;
		Car.RightTargetSpeed = Car.TargetSpeed;
	}
	else
	{
		/*  由当前误差来动态计算左右电机目标速度  */
		Car.LeftTargetSpeed = Car.TargetSpeed + g_SpeedFactor * Car.HorizontalAE;
		Car.RightTargetSpeed = Car.TargetSpeed - g_SpeedFactor * Car.HorizontalAE;
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
	
	/*  计算左边电机输出量  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
	/*  计算右边电机输出量  */
	SpeedControlValue = g_RightSpeedControlOutNew - g_RightSpeedControlOutOld;
	g_RightSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_RightSpeedControlOutOld;
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
static uint16_t resumeCnt = 0;
static float RecoupTemp = 0;
void Car_RoadDetect(void)
{
<<<<<<< HEAD
	static uint32_t FirstTime = 0;
	static float AngleTemp = 0;
<<<<<<< HEAD
=======
	static uint32_t cnt = 0;
	
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
=======
	static float FirstPoint = 0, SecondPoint = 0, EnterIslandAngle = 0;
	static uint32_t FirstPointTime = 0, SecondPointTime = 0, OutIslandTime = 0;
	
	/*  当两边的电感值都小于阈值时,说明车子已经跑出跑道了,计数,超过一定次数后停车  */
	if(Car.Sensor[SENSOR_H_L].Average < LOST_LINE_THRESHOLD && Car.Sensor[SENSOR_H_R].Average < LOST_LINE_THRESHOLD)
		g_LossLineCounter++;
>>>>>>> Mr-He
	
	if(Car.Sensor[SENSOR_H_L].Average < 10) g_LeftLostLineInteral ++;
	if(Car.Sensor[SENSOR_H_L].Average < 10) g_RightLostLineInteral ++;
	
	if(g_LeftLostLineInteral > 10) 
	{
		RecoupTemp = 20;
		bsp_beep_ON(6,2, 10);
//		Car.DirFuzzy.KPMax *= 2.5;
	}
	else if(g_RightLostLineInteral > 10) 
	{
//		Car.DirFuzzy.KPMax *= 2.5;
		bsp_beep_ON(6, 2, 10);
		RecoupTemp = -20;
	}
	
<<<<<<< HEAD

	/*  中间电感的值大于阈值,说明已经进入了圆环范围,首先判断圆环方向  */
=======
	
<<<<<<< HEAD
	/*  该标志为0,说明还没有找到第一个标志点  */	
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
	if(Car.Sensor[SENSOR_M].Average > 45 )
	{
		if(g_CurveStatus == 0)		/*  表示是第一次进入圆环区域,接下来需要判断方向  */
=======
	if(Car.Sensor[SENSOR_H_L].Average > 16 && Car.Sensor[SENSOR_H_L].Average > 16)
	{
//		
		if(g_LostLineRecoup !=0)
>>>>>>> Mr-He
		{
			resumeCnt++;
			if(g_LostLineRecoup > 0)
			{
<<<<<<< HEAD
<<<<<<< HEAD
				Car.NowRoad = LEFT_ISLAND;	
=======
				Car.NowRoad = LEFT_ISLAND;
				
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				g_CurveOffset = 0;		/*  防止上次未清除偏移量  */
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
=======
				g_LostLineRecoup = (resumeCnt + 1) / 10 * RecoupTemp - RecoupTemp;
>>>>>>> Mr-He
			}
			else if(g_LostLineRecoup < 0)
			{
<<<<<<< HEAD
				Car.NowRoad = RIGHT_ISLAND;
				g_CurveOffset = 0;					/*  防止上次未清除偏移量  */
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
=======
				g_LostLineRecoup = (resumeCnt + 1) / 10 * RecoupTemp + RecoupTemp;
>>>>>>> Mr-He
			}
		}
		else
		{
//			bsp_beep_OFF();
			Car.DirFuzzy.KPMax = g_DirKp;
			resumeCnt = 0;
		}
		g_LeftLostLineInteral = 0;
		g_RightLostLineInteral = 0;
	}
<<<<<<< HEAD
	
	/*  已经找到第一个标志点了,需要找到第二个标志点,第二个标志点为中间水平电感的峰值  */
	if(g_CurveStatus == 1)	
=======
	else
		g_LostLineRecoup = RecoupTemp;
	/*  圆环状态机  */
	switch(g_IslandStatus)
>>>>>>> Mr-He
	{
		case CHECK_POINT_A:
		{
			if(Car.AE > 0) FirstPoint++;
			else if(Car.AE < 0) FirstPoint--;
			
			if(FirstPoint > 3 || FirstPoint <-3)		/*  记录第一个峰的极性,记录五次,防止误差极性  */
			{
<<<<<<< HEAD
				g_CurveOffset = -30;
				bsp_led_ON(LED_RED);
<<<<<<< HEAD
//				AngleTemp = Car.MPU.Yaw;
=======
				AngleTemp = Car.MPU.Yaw;
				g_CurveStatus = 2;
				bsp_beep_ON();
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				g_NeedEnterCurve = 1;
=======
				FirstPointTime = bsp_tim_GetRunTime();
				g_IslandStatus = CHECK_POINT_B;
				
				bsp_beep_ON(10, 5, 5);
				if(FirstPoint < 0) bsp_led_ON(LED_RED);			/*  根据圆环方向点亮不同的灯  */
				else if(FirstPoint > 0) bsp_led_ON(LED_BLUE);
>>>>>>> Mr-He
			}
		}break;
		case CHECK_POINT_B:
		{
			/*  如果在一定时间内没有找到第二个峰,则说明为误判,进环失败  */
			if(bsp_tim_GetRunTime() - FirstPointTime > 1000) g_IslandStatus = CLEAR_FLAG;
			
			if(FirstPoint * Car.AE < 0)			/*  在找到第一个峰后,出现一个反向峰值的时候就说明是进环的时候  */
			{
<<<<<<< HEAD
<<<<<<< HEAD
				g_NeedEnterCurve = 1;
//				AngleTemp = Car.MPU.Yaw;
=======
				g_CurveOffset = 30;
				g_CurveStatus = 2;
				AngleTemp = Car.MPU.Yaw;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				bsp_led_ON(LED_BLUE);
				bsp_beep_ON();
				g_NeedEnterCurve = 1;
			}
		}
	}
	
<<<<<<< HEAD
//	if(i_abs(Car.Sensor[SENSOR_V_L].Average - Car.Sensor[SENSOR_V_R].Average) < 5 && g_NeedEnterCurve == 1 && Car.Sensor[SENSOR_M].Average < 35)   //说明没进圆环 清除所有标志保证不影响下一个圆环的进入
//	{
//		g_NeedEnterCurve = 0;
//		g_CurveOffset = 0;
//		g_NeedOutCurve = 0;
//		bsp_led_OFF(LED_ALL);
//	}
	if(i_abs(Car.Sensor[SENSOR_V_L].Average - Car.Sensor[SENSOR_V_R].Average) > 10 && g_NeedEnterCurve == 1 && Car.Sensor[SENSOR_M].Average < 35)    //说明进圆环了 按正常处理
	{
		g_NeedOutCurve = 1;
		g_AlreadyEnterCurve = 1;
		g_CurveOffset = 0;
		Car.TargetSpeed = 15;
	}
	
	if(g_AlreadyEnterCurve == 1 && Car.Sensor[SENSOR_M].Average > 40)    //已经进入圆环 将圆环总标志置零
	{
		g_CurveStatus = 0;
		Car.TargetSpeed = 10;
	}
	
//	if(fabs(Car.MPU.Yaw - AngleTemp) > 40 && g_NeedEnterCurve == 1) 
//	{
//		g_NeedOutCurve = 1;
//		g_NeedEnterCurve = 0;
//		g_CurveOffset = 0;
//	}
//	if(fabs(Car.MPU.Yaw - AngleTemp) > 300 && g_NeedOutCurve == 1) Car.TargetSpeed = 10;
	
	if(Car.Sensor[SENSOR_M].Average < 35 && Car.VecticalAE == 0 && g_CurveStatus == 0)   //(Car.Sensor[SENSOR_H_L].Average  + Car.Sensor[SENSOR_H_R].Average) < 145)
	{
		g_NeedEnterCurve = 0;
		g_AlreadyEnterCurve = 1;
		g_CurveOffset = 0;
		g_NeedOutCurve = 0;
		Car.TargetSpeed = 15;
=======
	if(fabs(Car.MPU.Pitch) > 15 && g_NeedEnterCurve == 1) 
	{
		g_NeedEnterCurve = 0;
		AngleTemp = 0;
		g_CurveOffset = 0;
		g_CurveStatus = 0;
		bsp_led_OFF(LED_ALL);
		Car.TargetSpeed = 15;
		bsp_beep_OFF();
	}
	
	if(fabs(Car.MPU.Yaw - AngleTemp) > 40 && g_NeedEnterCurve ==1)
	{
		g_CurveOffset = 0;
		g_NeedOutCurve = 1;
		g_NeedEnterCurve = 0;
		Car.TargetSpeed = 15;
	}
	
	if(g_NeedOutCurve == 1 && fabs(Car.MPU.Yaw - AngleTemp) > 200)
	{
		g_NeedOutCurve = 2;
		Car.TargetSpeed = 10;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
		bsp_led_OFF(LED_ALL);
	}
	
	if(Car.Sensor[SENSOR_M].Average < 40 && Car.Sensor[SENSOR_H_L].Average < 100 && Car.Sensor[SENSOR_H_R].Average < 100)
	{
		if(g_NeedOutCurve == 2)
=======
				g_IslandOffset = (FirstPoint < 0) ? (3) : (-3);		/*  根据圆环方向来搬中线  */
				EnterIslandAngle = Car.MPU.Yaw;				/*  记录当前角度,用于判断已经进入圆环  */
				SecondPoint = Car.AE;									/*  记录第二个峰的极性,用于判断出环  */
				SecondPointTime = bsp_tim_GetRunTime();
				g_IslandStatus = WAIT_ENTER;			/*  等待进环  */
			}
		}break;
		case WAIT_ENTER:
>>>>>>> Mr-He
		{
			if(bsp_tim_GetRunTime() - SecondPointTime > 2000) g_IslandStatus = CLEAR_FLAG;
			if(f_abs(Car.MPU.Yaw - EnterIslandAngle) > 40) 
			{
				g_IslandOffset = 0;
				OutIslandTime = bsp_tim_GetRunTime();
				g_IslandStatus = WAIT_OUT_A;		/*  等待出环第一步  */
			}
		}break;
		case WAIT_OUT_A:		
		{
			if(bsp_tim_GetRunTime() - OutIslandTime > 5 * 1000) g_IslandStatus = CLEAR_FLAG;		/*  出环超时  */
			if(SecondPoint * Car.AE > 0) g_IslandStatus = WAIT_OUT_B; /*  出环的峰极性与入环时相反，记录此时为出环标志  */
		}break;
		case WAIT_OUT_B:		/*  找到出环的第二个标志,中间电感值大于阈值或者第二个峰值为0  */
		{
			if(Car.AE == 0 || Car.Sensor[SENSOR_M].Average > 90) 	g_IslandStatus = CLEAR_FLAG;
		}break;
		case CLEAR_FLAG:		/*  进环成功或者失败后都会跳转到这个状态  */
		{
			bsp_led_OFF(LED_ALL);
			FirstPoint = 0;
			SecondPoint = 0;
			EnterIslandAngle = 0;
			FirstPointTime = 0;
			g_IslandOffset = 0;
			g_IslandStatus = CHECK_POINT_A;
		}break;
		default:g_IslandStatus = CLEAR_FLAG;break;
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
<<<<<<< HEAD
	float KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
	static float temp = 0;
	
<<<<<<< HEAD
	switch(Car.NowRoad)
	{
		case STRAIGHT:temp = 0; g_CurveOffset = 0; g_NeedEnterCurve = 0; g_NeedOutCurve = 0; g_CurveSpeedControl = 0; break;
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
=======
	Gyro_Z = Car.MPU.Gyroz;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
	
	Error = Car.HorizontalAE + g_CurveOffset;
	
=======
	int16_t KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
		
	Error = Car.HorizontalAE + g_IslandOffset + g_LostLineRecoup;
//	Error = g_LostLineRecoup;
>>>>>>> Mr-He
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
	Gyro_Z = Car.MPU.Gyroz - MPU_GYROZ_ZERO;
	KdGryozOut = g_GryoZ_Kd * Gyro_Z;		/*  采用陀螺仪的角速度进行补偿,抵制转向  */
		
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
	
	/*  将速度环和转向环的PWM叠加起来  */
	LeftPwm = (int16_t)(g_LeftSpeedControlOut + g_DirectionControlOut);
	RightPwm = (int16_t)(g_RightSpeedControlOut - g_DirectionControlOut);
	
	/*  限幅  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
	
	/*  输出到电机  */
	if(g_LossLineCounter > 2)		/*  冲出跑道停车  */
	{
		bsp_motor_SetPwm(0,0);
		bsp_beep_Pause();
	}
	else
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
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
<<<<<<< HEAD
<<<<<<< HEAD
		/*  每5ms读取一次编码器  */
		case 1:
		{
			SpeedControlCounter++;
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5 && g_SpeedControlON == 1)
			{
				SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
//				Car_SpeedControl();				/*  运行时长68us  */
			}
		}break;
		
		/*  速度控制  */
		case 2:
		{
=======
		SpeedControlCounter = 0;
		g_SpeedControlPeriod = 0;
		Car_SpeedControl();				/*  运行时长68us  */
	}
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f

	bsp_sensor_DataProcess();		/*  运行时长64us  */
	Car_RoadDetect();



	DirectionControlCounter++;
	if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
	{
		DirectionControlCounter = 0;
		g_DirectionControlPeriod = 0;
		Car_DirectionControl();		/*  运行时长600us  */
=======
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
			Car_RoadDetect();
			g_DirectionControlPeriod = 0;
			Car_DirectionControl();
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
			g_SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirectionControlPeriod = 0;
		}break;
>>>>>>> Mr-He
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

