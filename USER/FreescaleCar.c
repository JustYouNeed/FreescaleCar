/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-3
  * Brief: 本文件用于车子数据记录、处理、保存等，同时车子控制函数也在本文件
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
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
	
Car_TypeDef Car;


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
	
	/*  初始化车子的各差比  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  初始化车子的PID参数,从Flash中读取出保存的PID参数  */
	Car.PID.Kp_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);	/*  直道PID  */
	Car.PID.Ki_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Kd_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Kp_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);		/*  弯道PID  */
	Car.PID.Ki_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Kd_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
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
	Car.BaseSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, int16_t);
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
*                                Car_PIDCalc          
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
void Car_PIDCalc(void)
{
	int16_t pwm = 0;
	static float LastError;
	
	Car.PID.Error = Car.HorizontalAE - LastError;		/*  计算当前微分量  */
	
	/*  计算PWM  */
	if(Car.Sensor[SENSOR_V_L].Average < 10 && Car.Sensor[SENSOR_V_R].Average < 10)	/*  直道  */
	{
		pwm = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Straight) + (Car.PID.Sum * Car.PID.Ki_Straight) - (Car.PID.Error * Car.PID.Kd_Straight));
	}
	else			/*  弯道	*/
	{
		pwm = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Curved) + (Car.PID.Sum * Car.PID.Ki_Curved) - (Car.PID.Error * Car.PID.Kd_Curved));
	}
	
	/*  保存上个时刻的误差  */
	LastError = (float)(Car.HorizontalAE * 10);
	
	if(Car.Sensor[SENSOR_V_L].Average > 12 || Car.Sensor[SENSOR_H_R].Average > 12)
	{
		/*  将计算出来的PID与基本速度相加  */
		Car.Motor.LeftPwm = Car.BaseSpeed*2/3 - pwm;
		Car.Motor.RightPwm = Car.BaseSpeed*2/3 + pwm;
	}
	else
	{
		/*  将计算出来的PID与基本速度相加  */
		Car.Motor.LeftPwm = Car.BaseSpeed - pwm;
		Car.Motor.RightPwm = Car.BaseSpeed + pwm;
	}
	
	/*  进行限幅  */
	if(Car.Motor.LeftPwm > 600) Car.Motor.LeftPwm = 600;
	else if(Car.Motor.LeftPwm < -600) Car.Motor.LeftPwm = -600;
	
	if(Car.Motor.RightPwm > 600) Car.Motor.RightPwm = 600;
	else if(Car.Motor.RightPwm < -600) Car.Motor.RightPwm = -600;
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
	bsp_led_Toggle(1);
	bsp_sensor_DataProcess();
	bsp_encoder_SpeedCalc();
	Car_PIDCalc();
	
	
//	if(Car.Sensor[SENSOR_H_L].Average < 20 || Car.Sensor[SENSOR_V_R].Average < 20)
//		bsp_motor_Stop();
//	else
//		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
	
	
/********************************************  END OF FILE  *******************************************/
	

