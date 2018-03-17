/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
<<<<<<< HEAD
  * Date: 2018-3-9
  * Brief: 本文件为三轮小车主要控制函数所在
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-9
=======
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
//	Car.BaseSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, int16_t);
	Car.BaseSpeed = 100;
	
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
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t *)&Car.BaseSpeed, 2, 0);
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
	int16_t pwm = 0;
	static float LastError;
	
	Car.PID.Error = HorizontalAE - LastError;		/*  计算当前微分量  */
	
	/*  计算PWM  */
	pwm = (int16_t)((HorizontalAE*10 *  Car.PID.Kp_Straight) + (Car.PID.Error * Car.PID.Kd_Straight));
	
	/*  保存上个时刻的误差  */
	LastError = (float)(Car.HorizontalAE);
	
	return pwm;
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
//void Car_RaodCalc(void)
//{
//	uint8_t LastDir = 0;
//	
//	/*  如果右边电感的归一化值大于左边的电感,说明上一个时刻是保持右转的状态  */
//	if((100 * (Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue)) >= 20) 
//		LastDir = TurnRight;
//	
//	/*  如果左边电感的归一化值大于右边的电感,说明上一个时刻是保持左转的状态  */
//	if((100 * (Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue)) >= 20)
//		LastDir = TurnLeft;
//	
//	
//}

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
	int16_t SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  速度偏差  */
	SpeedError = LeftSpeed - Car.BaseSpeed;
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.5);
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
	int16_t SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  速度偏差  */
	SpeedError = RightSpeed - Car.BaseSpeed;
	
	/*  低通滤波,让速度平滑过渡  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.5);
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
<<<<<<< HEAD
<<<<<<< HEAD
	/*  处理小车传感器的数据  */
	bsp_sensor_DataProcess();
	
	/*  计算小车电机速度  */
=======
	bsp_led_Toggle(1);
	bsp_sensor_DataProcess();
<<<<<<< HEAD
>>>>>>> origin/Mr-He
=======
>>>>>>> Mr-He
>>>>>>> d476e22040494988d81fb0d65879a545a2623703
=======
	int16_t TurnPwm = 0, LeftVelocityPwm = 0, RightVelocityPwm = 0;
	
	/*  进行传感器数据处理  */
	bsp_sensor_DataProcess();
	
	/*  速度计算  */
>>>>>>> Mr-He
	bsp_encoder_SpeedCalc();
	
	/*  速度环计算  */
	LeftVelocityPwm = Car_LeftVelocityPIDCalc(Car.Motor.LeftEncoder);
	RightVelocityPwm = Car_RightVelocityPIDCalc(Car.Motor.RightEncoder);
		
	/*  转向环计算  */
	TurnPwm =	Car_TurnPIDCalc(Car.HorizontalAE);
	
	if((TurnLeft + LeftVelocityPwm) < 0)	Car.Motor.LeftPwm = 80;
	else 	Car.Motor.LeftPwm = LeftVelocityPwm+TurnPwm;
	
	if((TurnPwm - RightVelocityPwm) < 0) Car.Motor.RightPwm = 80;
	else Car.Motor.RightPwm = RightVelocityPwm-TurnPwm;
	
	/*  对PWM进行限幅  */
	if(Car.Motor.LeftPwm >= Car.MaxPWM) Car.Motor.LeftPwm = Car.MaxPWM;
	else if(Car.Motor.LeftPwm <= -Car.MaxPWM) Car.Motor.LeftPwm = -Car.MaxPWM;
	
	if(Car.Motor.RightPwm >= Car.MaxPWM) Car.Motor.RightPwm = Car.MaxPWM;
	else if(Car.Motor.RightPwm <= -Car.MaxPWM) Car.Motor.RightPwm = -Car.MaxPWM;
	
	/*  电机控制  */
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
	
	
/********************************************  END OF FILE  *******************************************/
	

