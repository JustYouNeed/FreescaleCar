/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
<<<<<<< HEAD
  * Version: V1.0.0
<<<<<<< HEAD
  * Date: 2018-3-9
  * Brief: 本文件为三轮小车主要控制函数所在
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-9
=======
=======
  * Version: V1.1.0
>>>>>>> Mr-He
  * Date: 2018-3-3
  * Brief: 本文件用于车子数据记录、处理、保存等，同时车子控制函数也在本文件
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
<<<<<<< HEAD
>>>>>>> origin/Mr-He
=======
>>>>>>> Mr-He
>>>>>>> d476e22040494988d81fb0d65879a545a2623703
	*			Mod: 建立文件
  *
	*		2.Author: Vector
	*			Date:	2018-3-19
	*			Mod: 1.修改车子控制逻辑,改变寻线策略
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"
	
	
/*  小车控制结构体  */
Car_TypeDef Car;

/*  小车速度控制计数器  */
static uint16_t g_SpeedControlCounter = 0;
/*  小车速度输出控制周期  */
static uint16_t g_SpeedControlPeriod = 0;


/*  小车速度环控制PWM输出  */
static int16_t g_LeftSpeedControlOutNew = 0;
static int16_t g_LeftSpeedControlOutOld = 0;
static float g_LeftSpeedControlIntegral = 0;
static int16_t g_LeftSpeedControlOut;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static float g_RightSpeedControlIntegral = 0;
static int16_t g_RightSpeedControlOut;

/*  小车方向控制变量  */
static int16_t g_DirctionControlOut =0 ;
static int16_t g_DirctionControlCounter = 0;
static int16_t g_DirctionControlPeriod = 0;
static int16_t g_DirctionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;
/*
*********************************************************************************************************
<<<<<<< HEAD
*                          Car_ParaInit                
*
* Description: 将小车控制参数初始化,从Flash中读取出存储的参数
=======
*                         Car_ParaInit                 
*
* Description: 车子参数初始化,从芯片Flash中读取出存储的电机参数
<<<<<<< HEAD
>>>>>>> origin/Mr-He
=======
>>>>>>> Mr-He
>>>>>>> d476e22040494988d81fb0d65879a545a2623703
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
	Car.PID.Kp_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);	/*  直道PID  */
	Car.PID.Ki_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Kd_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Kp_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);		/*  弯道PID  */
	Car.PID.Ki_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Kd_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	Car.PID.Velocity_Kp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 24, float);
	Car.PID.Velocity_Ki = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 28, float);
	Car.PID.Velocity_Kd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 32, float);
	
	/*  初始化车子的传感器参数,从Flash中读取标定值  */
	for(i = 0; i < SENSOR_COUNT; i ++)
	{
		Car.Sensor[i].FIFO[0] = 0;
		Car.Sensor[i].Read = 0;
		Car.Sensor[i].Write = 0;
		Car.Sensor[i].Average = 0;
		Car.Sensor[i].NormalizedValue = 0.0f;
		Car.Sensor[i].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, i * 2, uint16_t);
		Car.Sensor[i].CalibrationMin = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i + SENSOR_COUNT) * 2, uint16_t);
	}
	
	/*  电机控制参数初始化  */
	Car.Motor.PWM_Frequency = 10;	/*  电机PWM频率为10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	/*  小车基本速度  */
//	Car.TargetSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, int16_t);
	Car.LeftTargetSpeed = STRAIGHT_SPEED;
	Car.RightTargetSpeed = STRAIGHT_SPEED;
	
	/*  小车初始道路为直道  */
	Car.NowRoad = Road_Straight;
	
	/*  小车开始未丢线  */
	Car.LossLine = LostLine_None;
	
	Car.MaxPWM = 800;
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
	bsp_led_Toggle(0);
}

/*
*********************************************************************************************************
*                            Car_ParaStroe              
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
void Car_ParaStroe(void)
{
//	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);
//	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t *)&Car.TargetSpeed, 2, 0);
}

/*
*********************************************************************************************************
*                                Car_TurnPIDCalc          
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
int16_t Car_TurnPIDCalc(float HorizontalAE)
{

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
void Car_RaodCalc(void)
{
	uint8_t LastDir = 0;
	static uint8_t LeftLossLineCount = 0, RightLossLineCount = 0;
	
	/*  如果右边电感的归一化值大于左边的电感,说明上一个时刻是保持右转的状态  */
	if((100 * (Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue)) >= 20) 
	{
		LastDir = TurnRight;
		LeftLossLineCount ++;
	}
	else LeftLossLineCount = 0;
	
	/*  如果左边电感的归一化值大于右边的电感,说明上一个时刻是保持左转的状态  */
	if((100 * (Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue)) >= 20)
	{
		LastDir = TurnLeft;
		RightLossLineCount ++;
	}
	else LeftLossLineCount = 0;
	
	/*  左边丢线很多,已经向左偏了  */
	if(LeftLossLineCount > 20)	
	{
		Car.LeftTargetSpeed += 1;		/*  加大左边电机目标速度  */
		Car.RightTargetSpeed -= 1;	/*  减小右边电机目标速度  */
	}
	else if(LeftLossLineCount > 50)		/*  大弯丢线  */
	{
		Car.LeftTargetSpeed += 2;		/*  加大左边电机目标速度  */
		Car.RightTargetSpeed -= 2;	/*  减小右边电机目标速度  */
	}
	else												/*  没有丢线,直道  */
	{
		Car.LeftTargetSpeed = STRAIGHT_SPEED;
		Car.RightTargetSpeed = STRAIGHT_SPEED;
	}
	
	/*  右边丢线很多,已经向右偏了  */
	if(RightLossLineCount > 20)	
	{
		Car.LeftTargetSpeed -= 1;		/*  减小左边电机目标速度  */
		Car.RightTargetSpeed += 1;	/*  加大右边电机目标速度  */
	}
	else if(RightLossLineCount > 50)		/*  大弯丢线  */
	{
		Car.LeftTargetSpeed -= 2;		/*  减小左边电机目标速度  */
		Car.RightTargetSpeed += 2;	/*  加大右边电机目标速度  */
	}
	else												/*  没有丢线,直道  */
	{
		Car.LeftTargetSpeed = STRAIGHT_SPEED;
		Car.RightTargetSpeed = STRAIGHT_SPEED;
	}
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
	static float SpeedFilter, SpeedIntegal;
	int16_t Velocity = 0;
	float SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  速度偏差  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed) * 10.0f;
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	SpeedIntegal += SpeedFilter;
	
	/*  积分限幅  */
	if(SpeedIntegal > 7200) SpeedIntegal = 7200;			//积分限幅
	else if(SpeedIntegal < -7200) SpeedIntegal = -7200;		//
	
	/*  速度环PD控制,实际上Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) +
							SpeedIntegal * (-Car.PID.Velocity_Ki)) + 
							(SpeedError - PreSpeedError)*(-Car.PID.Velocity_Kd);	//速度环PID计算	
	
	/*  保存上一时刻偏差值  */
	PreSpeedError = SpeedError;
	
	return Velocity;
}
/*
*********************************************************************************************************
*                    　             
*
* Description: 公式　左轮：W1=2W(R+L/2)d     右轮： W2=2W(R-L/2)*d
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/

















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
	static float SpeedFilter, SpeedIntegal;
	int16_t Velocity = 0;
	float SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  速度偏差  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed) * 10.0f;
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	SpeedIntegal += SpeedFilter;
	
	/*  积分限幅  */
	if(SpeedIntegal > 7200) SpeedIntegal = 7200;			//积分限幅
	else if(SpeedIntegal < -7200) SpeedIntegal = -7200;		//
	
	/*  速度环PD控制,实际上Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) +
							SpeedIntegal * (-Car.PID.Velocity_Ki)) + 
							(SpeedError - PreSpeedError)*(-Car.PID.Velocity_Kd);	//速度环PID计算	
	
	/*  保存上一时刻偏差值  */
	PreSpeedError = SpeedError;
	
	return Velocity;
}
/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: 小车速度环控制
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
	float LeftSpeedDelta = 0, RightSpeedDelta = 0;
	float LeftPValue = 0, LeftIValue = 0, RightPValue = 0, RightIValue = 0;
	
	/*  左边电机速度环计算  */
	Car.Motor.LeftSpeed = (float)(Car.Motor.LeftEncoder * CAR_SPED_CONSTANT);
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	Car.Motor.LeftEncoder = 0;
	
//	LeftSpeedDelta = (Car.TargetSpeed - Car.Motor.LeftSpeed) * 10.0f;
//	LeftPValue = LeftSpeedDelta * Car.PID.Velocity_Kp;
//	LeftIValue = LeftSpeedDelta * Car.PID.Velocity_Ki;
//	g_LeftSpeedControlIntegral += LeftIValue;
	
	g_LeftSpeedControlOutOld = g_LeftSpeedControlOutNew;
	g_LeftSpeedControlOutNew = Car_LeftVelocityPIDCalc(Car.Motor.LeftSpeed); //(int16_t)(LeftPValue + g_LeftSpeedControlIntegral);
	
	/*  右边电机速度环计算  */
	Car.Motor.RightSpeed = (float)(Car.Motor.RightEncoder * CAR_SPED_CONSTANT);
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	Car.Motor.RightEncoder = 0;
	
	
//	RightSpeedDelta = (Car.TargetSpeed - Car.Motor.RightSpeed) * 10.0f;
//	RightPValue = RightSpeedDelta * Car.PID.Velocity_Kp;
//	RightIValue = RightSpeedDelta * Car.PID.Velocity_Ki;
//	g_RightSpeedControlIntegral += RightIValue;
	
	g_RightSpeedControlOutOld = g_RightSpeedControlOutNew;
	g_RightSpeedControlOutNew = Car_RightVelocityPIDCalc(Car.Motor.RightSpeed); //(int16_t)(RightPValue + g_RightSpeedControlIntegral);
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
<<<<<<< HEAD
	int16_t TurnPwm = 0, LeftVelocityPwm = 0, RightVelocityPwm = 0;
=======
<<<<<<< HEAD
<<<<<<< HEAD

=======
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 270b57686bfc76f2a7a146d06ec08edb3e0535f5
>>>>>>> 6222d79d6b85467c792fc47192493923a283fc31
	/*  处理小车传感器的数据  */
	bsp_sensor_DataProcess();
	
	/*  计算小车电机速度  */

	bsp_led_Toggle(1);
	bsp_sensor_DataProcess();

<<<<<<< HEAD
	
=======

=======
<<<<<<< HEAD
>>>>>>> origin/Mr-He
=======
>>>>>>> Mr-He
>>>>>>> d476e22040494988d81fb0d65879a545a2623703
=======
	int16_t TurnPwm = 0, LeftVelocityPwm = 0, RightVelocityPwm = 0;
=======
	float SpeedControlValue;
>>>>>>> Mr-He
>>>>>>> 6222d79d6b85467c792fc47192493923a283fc31
	
	/*  计算左边电机输出量  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
<<<<<<< HEAD
	/*  速度计算  */
	bsp_encoder_SpeedCalc();
=======
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
void Car_DirctionControl(void)
{
	int16_t pwm = 0;
	static float LastError;
>>>>>>> Mr-He
	
	Car.PID.Error = Car.HorizontalAE - LastError;		/*  计算当前微分量  */
	
	g_DirctionControlPeriod = g_DirctionControlOutNew;
	/*  计算PWM  */
	g_DirctionControlOutNew = (int16_t)((Car.HorizontalAE*10 *  Car.PID.Kp_Straight) + (Car.PID.Error * Car.PID.Kd_Straight));
	
	/*  保存上个时刻的误差  */
	LastError = (float)(Car.HorizontalAE);
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
void Car_DirctionControlOutput(void)
{
	int16_t DirctionOutput = 0;
	
	DirctionOutput = g_DirctionControlOutNew - g_DirctionControlPeriod;
	g_DirctionControlOut = DirctionOutput * (g_DirctionControlOut + 1)/DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld;
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
	/*  将速度环和转向环的PWM叠加起来  */
	Car.Motor.LeftPwm = g_LeftSpeedControlOut - g_DirctionControlOut;
	Car.Motor.RightPwm = g_RightSpeedControlOut + g_DirctionControlOut;
	
	/*  限幅  */
	if(Car.Motor.LeftPwm > Car.MaxPWM) Car.Motor.LeftPwm = Car.MaxPWM;
	else if(Car.Motor.LeftPwm < - Car.MaxPWM) Car.Motor.LeftPwm = -Car.MaxPWM;
	
	if(Car.Motor.RightPwm > Car.MaxPWM) Car.Motor.RightPwm = Car.MaxPWM;
	else if(Car.Motor.RightPwm < - Car.MaxPWM) Car.Motor.RightPwm = -Car.MaxPWM;
	
	/*  输出到电机  */
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}

/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: 车子控制函数,解算传感器数据
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
	
	volatile int16_t TurnPwm = 0, LeftVelocityPwm = 0, RightVelocityPwm = 0;
	
	bsp_led_Toggle(2);
	CarControlCunter++;
	
	/*    */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	g_DirctionControlPeriod++;
	Car_DirctionControlOutput();
	switch(CarControlCunter)
	{
		case 1: bsp_encoder_ReadCounter();break;
		case 2:
		{
			bsp_led_Toggle(1);
			g_SpeedControlCounter++;
			if(g_SpeedControlCounter >= 4)
			{
				g_SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
				Car_SpeedControl();
			}
		}
		case 3:
		{
			g_DirctionControlCounter++;
			if(g_DirctionControlCounter >= 2)
			{
				g_DirctionControlCounter = 0;
				Car_DirctionControl();
			}
		}break;
		case 4:bsp_sensor_DataProcess();break;
		case 5:
		{
			Car_MotorOutput();
			CarControlCunter=0;
		}break;
		default:break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

