/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-3
  * Brief: ���ļ����ڳ������ݼ�¼����������ȣ�ͬʱ���ӿ��ƺ���Ҳ�ڱ��ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"
	
	
/*  С�����ƽṹ��  */
Car_TypeDef Car;


/*
*********************************************************************************************************
*                         Car_ParaInit                 
*
* Description: ���Ӳ�����ʼ��,��оƬFlash�ж�ȡ���洢�ĵ������
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
	
	/*  ��ʼ�����ӵĺͲ��  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  ��ʼ�����ӵ�PID����,��Flash�ж�ȡ�������PID����  */
	Car.PID.Kp_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);	/*  ֱ��PID  */
	Car.PID.Ki_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Kd_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Kp_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);		/*  ���PID  */
	Car.PID.Ki_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Kd_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	Car.PID.Velocity_Kp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 24, float);
	Car.PID.Velocity_Ki = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 28, float);
	Car.PID.Velocity_Kd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 32, float);
	
	/*  ��ʼ�����ӵĴ���������,��Flash�ж�ȡ�궨ֵ  */
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
	
	/*  ������Ʋ�����ʼ��  */
	Car.Motor.PWM_Frequency = 10;	/*  ���PWMƵ��Ϊ10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	/*  С�������ٶ�  */
//	Car.BaseSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, int16_t);
	Car.BaseSpeed = 30;
	
	/*  С����ʼ��·Ϊֱ��  */
	Car.NowRoad = Road_Straight;
	
	/*  С����ʼδ����  */
	Car.LossLine = LostLine_None;
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
int16_t Car_PIDCalc(void)
{
	int16_t pwm = 0;
	static float LastError;
	
	Car.PID.Error = Car.HorizontalAE - LastError;		/*  ���㵱ǰ΢����  */
	
	/*  ����PWM  */
	if(Car.Sensor[SENSOR_V_L].Average < 10 && Car.Sensor[SENSOR_V_R].Average < 10)	/*  ֱ��  */
	{
		pwm = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Straight) + (Car.PID.Sum * Car.PID.Ki_Straight) - (Car.PID.Error * Car.PID.Kd_Straight));
	}
	else			/*  ���	*/
	{
		pwm = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Curved) + (Car.PID.Sum * Car.PID.Ki_Curved) - (Car.PID.Error * Car.PID.Kd_Curved));
	}
	
	/*  �����ϸ�ʱ�̵����  */
	LastError = (float)(Car.HorizontalAE * 10);
	
	return pwm;
//	if(Car.Sensor[SENSOR_V_L].Average > 12 || Car.Sensor[SENSOR_H_R].Average > 12)
//	{
//		/*  �����������PID������ٶ����  */
//		Car.Motor.LeftPwm = Car.BaseSpeed*2/3 - pwm;
//		Car.Motor.RightPwm = Car.BaseSpeed*2/3 + pwm;
//	}
//	else
//	{
//		/*  �����������PID������ٶ����  */
//		Car.Motor.LeftPwm = Car.BaseSpeed - pwm;
//		Car.Motor.RightPwm = Car.BaseSpeed + pwm;
//	}
//	
//	/*  �����޷�  */
//	if(Car.Motor.LeftPwm > 600) Car.Motor.LeftPwm = 600;
//	else if(Car.Motor.LeftPwm < -600) Car.Motor.LeftPwm = -600;
//	
//	if(Car.Motor.RightPwm > 600) Car.Motor.RightPwm = 600;
//	else if(Car.Motor.RightPwm < -600) Car.Motor.RightPwm = -600;
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
	
	/*  ����ұߵ�еĹ�һ��ֵ������ߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
	if((100 * (Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue)) >= 20) 
		LastDir = TurnRight;
	
	/*  �����ߵ�еĹ�һ��ֵ�����ұߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
	if((100 * (Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue)) >= 20)
		LastDir = TurnLeft;
	
	
}

/*
*********************************************************************************************************
*                       Car_VelocityPIDCalc                   
*
* Description: С���ٶȻ�PID����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t Car_LeftVelocityPIDCalc(int16_t LeftSpeed)
{
	static float Encoder, Encoder_Integral;
	int16_t Velocity = 0;
	int16_t Error = 0;
	static int16_t LastError;
	
	Error = LeftSpeed - Car.BaseSpeed;
	
	Encoder *= 0.7;
	Encoder += (Error * 0.3);
	Encoder_Integral += Encoder;
	
	if(Encoder_Integral > 7200) Encoder_Integral = 7200;			//�����޷�
	else if(Encoder_Integral < -7200) Encoder_Integral = -7200;		//
	
	Velocity = (int16_t)(Encoder * (-Car.PID.Velocity_Kp) + Encoder_Integral * (-Car.PID.Velocity_Ki)) + (Error - LastError)*(-Car.PID.Velocity_Kd);	//�ٶȻ�PID����	
	LastError = Error;
	
	return Velocity;
}


int16_t Car_RightVelocityPIDCalc(int16_t RightSpeed)
{
	static float Encoder, Encoder_Integral;
	int16_t Velocity = 0;
	int16_t Error = 0;
	static int16_t LastError;
	
	Error = RightSpeed - Car.BaseSpeed;
	
	Encoder *= 0.7;
	Encoder += (Error * 0.3);
	Encoder_Integral += Encoder;
	
	if(Encoder_Integral > 7200) Encoder_Integral = 7200;			//�����޷�
	else if(Encoder_Integral < -7200) Encoder_Integral = -7200;		//
	
	Velocity = (int16_t)(Encoder * (-Car.PID.Velocity_Kp) + Encoder_Integral * (-Car.PID.Velocity_Ki)) + (Error - LastError)*(-Car.PID.Velocity_Kd);	//�ٶȻ�PID����	
	LastError = Error;
	
	return Velocity;
}

/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: ���ӿ��ƺ���,���㴫��������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int16_t pwm = 0;
void Car_Control(void)
{
	static uint8_t cnt=0;
	int16_t TurnPwm = 0, LeftVelocityPwm = 0, RightVelocityPwm = 0;
	
	
	bsp_sensor_DataProcess();
	bsp_encoder_SpeedCalc();
	
	
	cnt ++;
	if(cnt >= 8)
	{
		bsp_led_Toggle(1);
		cnt = 0;
		LeftVelocityPwm = Car_LeftVelocityPIDCalc(Car.Motor.LeftEncoder);
		RightVelocityPwm = Car_RightVelocityPIDCalc(Car.Motor.RightEncoder);
		
		Car.Motor.LeftPwm += LeftVelocityPwm;
		Car.Motor.RightPwm += RightVelocityPwm;
	}
	
	TurnPwm =	Car_PIDCalc();
	Car.Motor.LeftPwm -= TurnPwm;
	Car.Motor.RightPwm += TurnPwm;
	
	if(Car.Motor.LeftPwm >= 900) Car.Motor.LeftPwm = 900;
	else if(Car.Motor.LeftPwm <= -900) Car.Motor.LeftPwm = -900;
	
		if(Car.Motor.RightPwm >= 900) Car.Motor.RightPwm = 900;
	else if(Car.Motor.RightPwm <= -900) Car.Motor.RightPwm = -900;
		
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
	
	
/********************************************  END OF FILE  *******************************************/
	

