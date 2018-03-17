/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.0.0
<<<<<<< HEAD
  * Date: 2018-3-9
  * Brief: ���ļ�Ϊ����С����Ҫ���ƺ�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-9
=======
  * Date: 2018-3-3
  * Brief: ���ļ����ڳ������ݼ�¼����������ȣ�ͬʱ���ӿ��ƺ���Ҳ�ڱ��ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
<<<<<<< HEAD
>>>>>>> origin/Mr-He
=======
>>>>>>> Mr-He
>>>>>>> d476e22040494988d81fb0d65879a545a2623703
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
<<<<<<< HEAD
*                          Car_ParaInit                
*
* Description: ��С�����Ʋ�����ʼ��,��Flash�ж�ȡ���洢�Ĳ���
=======
*                         Car_ParaInit                 
*
* Description: ���Ӳ�����ʼ��,��оƬFlash�ж�ȡ���洢�ĵ������
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
	Car.BaseSpeed = 100;
	
	/*  С����ʼ��·Ϊֱ��  */
	Car.NowRoad = Road_Straight;
	
	/*  С����ʼδ����  */
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
	
	Car.PID.Error = HorizontalAE - LastError;		/*  ���㵱ǰ΢����  */
	
	/*  ����PWM  */
	pwm = (int16_t)((HorizontalAE*10 *  Car.PID.Kp_Straight) + (Car.PID.Error * Car.PID.Kd_Straight));
	
	/*  �����ϸ�ʱ�̵����  */
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
//	/*  ����ұߵ�еĹ�һ��ֵ������ߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
//	if((100 * (Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue)) >= 20) 
//		LastDir = TurnRight;
//	
//	/*  �����ߵ�еĹ�һ��ֵ�����ұߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
//	if((100 * (Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue)) >= 20)
//		LastDir = TurnLeft;
//	
//	
//}

/*
*********************************************************************************************************
*                       Car_LeftVelocityPIDCalc                   
*
* Description: С�������ٶȻ�PID����
*             
* Arguments  : 1> LeftSpeed: �����ٶ�
*
* Reutrn     : 1> ����õ�������PWM
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
	
	/*  �ٶ�ƫ��  */
	SpeedError = LeftSpeed - Car.BaseSpeed;
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.5);
	SpeedIntegal += SpeedFilter;
	
	/*  �����޷�  */
	if(SpeedIntegal > 7200) SpeedIntegal = 7200;			//�����޷�
	else if(SpeedIntegal < -7200) SpeedIntegal = -7200;		//
	
	/*  �ٶȻ�PD����,ʵ����Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) +
							SpeedIntegal * (-Car.PID.Velocity_Ki)) + 
							(SpeedError - PreSpeedError)*(-Car.PID.Velocity_Kd);	//�ٶȻ�PID����	
	
	/*  ������һʱ��ƫ��ֵ  */
	PreSpeedError = SpeedError;
	
	return Velocity;
}

/*
*********************************************************************************************************
*                       Car_RightVelocityPIDCalc                   
*
* Description: С�������ٶȻ�����
*             
* Arguments  : 1> RightSpeed: �����ٶ�
*
* Reutrn     : 1> ����õ���PWM
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
	
	/*  �ٶ�ƫ��  */
	SpeedError = RightSpeed - Car.BaseSpeed;
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.5);
	SpeedIntegal += SpeedFilter;
	
	/*  �����޷�  */
	if(SpeedIntegal > 7200) SpeedIntegal = 7200;			//�����޷�
	else if(SpeedIntegal < -7200) SpeedIntegal = -7200;		//
	
	/*  �ٶȻ�PD����,ʵ����Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) +
							SpeedIntegal * (-Car.PID.Velocity_Ki)) + 
							(SpeedError - PreSpeedError)*(-Car.PID.Velocity_Kd);	//�ٶȻ�PID����	
	
	/*  ������һʱ��ƫ��ֵ  */
	PreSpeedError = SpeedError;
	
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
void Car_Control(void)
{
<<<<<<< HEAD
<<<<<<< HEAD
	/*  ����С��������������  */
	bsp_sensor_DataProcess();
	
	/*  ����С������ٶ�  */
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
	
	/*  ���д��������ݴ���  */
	bsp_sensor_DataProcess();
	
	/*  �ٶȼ���  */
>>>>>>> Mr-He
	bsp_encoder_SpeedCalc();
	
	/*  �ٶȻ�����  */
	LeftVelocityPwm = Car_LeftVelocityPIDCalc(Car.Motor.LeftEncoder);
	RightVelocityPwm = Car_RightVelocityPIDCalc(Car.Motor.RightEncoder);
		
	/*  ת�򻷼���  */
	TurnPwm =	Car_TurnPIDCalc(Car.HorizontalAE);
	
	if((TurnLeft + LeftVelocityPwm) < 0)	Car.Motor.LeftPwm = 80;
	else 	Car.Motor.LeftPwm = LeftVelocityPwm+TurnPwm;
	
	if((TurnPwm - RightVelocityPwm) < 0) Car.Motor.RightPwm = 80;
	else Car.Motor.RightPwm = RightVelocityPwm-TurnPwm;
	
	/*  ��PWM�����޷�  */
	if(Car.Motor.LeftPwm >= Car.MaxPWM) Car.Motor.LeftPwm = Car.MaxPWM;
	else if(Car.Motor.LeftPwm <= -Car.MaxPWM) Car.Motor.LeftPwm = -Car.MaxPWM;
	
	if(Car.Motor.RightPwm >= Car.MaxPWM) Car.Motor.RightPwm = Car.MaxPWM;
	else if(Car.Motor.RightPwm <= -Car.MaxPWM) Car.Motor.RightPwm = -Car.MaxPWM;
	
	/*  �������  */
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
	
	
/********************************************  END OF FILE  *******************************************/
	

