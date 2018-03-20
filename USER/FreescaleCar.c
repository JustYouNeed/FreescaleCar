/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
<<<<<<< HEAD
  * Version: V1.0.0
<<<<<<< HEAD
  * Date: 2018-3-9
  * Brief: ���ļ�Ϊ����С����Ҫ���ƺ�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-9
=======
=======
  * Version: V1.1.0
>>>>>>> Mr-He
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
	*		2.Author: Vector
	*			Date:	2018-3-19
	*			Mod: 1.�޸ĳ��ӿ����߼�,�ı�Ѱ�߲���
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

/*  С���ٶȿ��Ƽ�����  */
static uint16_t g_SpeedControlCounter = 0;
/*  С���ٶ������������  */
static uint16_t g_SpeedControlPeriod = 0;


/*  С���ٶȻ�����PWM���  */
static int16_t g_LeftSpeedControlOutNew = 0;
static int16_t g_LeftSpeedControlOutOld = 0;
static float g_LeftSpeedControlIntegral = 0;
static int16_t g_LeftSpeedControlOut;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static float g_RightSpeedControlIntegral = 0;
static int16_t g_RightSpeedControlOut;

/*  С��������Ʊ���  */
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
//	Car.TargetSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, int16_t);
	Car.LeftTargetSpeed = STRAIGHT_SPEED;
	Car.RightTargetSpeed = STRAIGHT_SPEED;
	
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
	
	/*  ����ұߵ�еĹ�һ��ֵ������ߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
	if((100 * (Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue)) >= 20) 
	{
		LastDir = TurnRight;
		LeftLossLineCount ++;
	}
	else LeftLossLineCount = 0;
	
	/*  �����ߵ�еĹ�һ��ֵ�����ұߵĵ��,˵����һ��ʱ���Ǳ�����ת��״̬  */
	if((100 * (Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue)) >= 20)
	{
		LastDir = TurnLeft;
		RightLossLineCount ++;
	}
	else LeftLossLineCount = 0;
	
	/*  ��߶��ߺܶ�,�Ѿ�����ƫ��  */
	if(LeftLossLineCount > 20)	
	{
		Car.LeftTargetSpeed += 1;		/*  �Ӵ���ߵ��Ŀ���ٶ�  */
		Car.RightTargetSpeed -= 1;	/*  ��С�ұߵ��Ŀ���ٶ�  */
	}
	else if(LeftLossLineCount > 50)		/*  ���䶪��  */
	{
		Car.LeftTargetSpeed += 2;		/*  �Ӵ���ߵ��Ŀ���ٶ�  */
		Car.RightTargetSpeed -= 2;	/*  ��С�ұߵ��Ŀ���ٶ�  */
	}
	else												/*  û�ж���,ֱ��  */
	{
		Car.LeftTargetSpeed = STRAIGHT_SPEED;
		Car.RightTargetSpeed = STRAIGHT_SPEED;
	}
	
	/*  �ұ߶��ߺܶ�,�Ѿ�����ƫ��  */
	if(RightLossLineCount > 20)	
	{
		Car.LeftTargetSpeed -= 1;		/*  ��С��ߵ��Ŀ���ٶ�  */
		Car.RightTargetSpeed += 1;	/*  �Ӵ��ұߵ��Ŀ���ٶ�  */
	}
	else if(RightLossLineCount > 50)		/*  ���䶪��  */
	{
		Car.LeftTargetSpeed -= 2;		/*  ��С��ߵ��Ŀ���ٶ�  */
		Car.RightTargetSpeed += 2;	/*  �Ӵ��ұߵ��Ŀ���ٶ�  */
	}
	else												/*  û�ж���,ֱ��  */
	{
		Car.LeftTargetSpeed = STRAIGHT_SPEED;
		Car.RightTargetSpeed = STRAIGHT_SPEED;
	}
}

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
	float SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed) * 10.0f;
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
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
*                    ��             
*
* Description: ��ʽ�����֣�W1=2W(R+L/2)d     ���֣� W2=2W(R-L/2)*d
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
	float SpeedError = 0;
	static int16_t PreSpeedError;
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed) * 10.0f;
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
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
*                          Car_SpeedControl                
*
* Description: С���ٶȻ�����
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
	
	/*  ��ߵ���ٶȻ�����  */
	Car.Motor.LeftSpeed = (float)(Car.Motor.LeftEncoder * CAR_SPED_CONSTANT);
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	Car.Motor.LeftEncoder = 0;
	
//	LeftSpeedDelta = (Car.TargetSpeed - Car.Motor.LeftSpeed) * 10.0f;
//	LeftPValue = LeftSpeedDelta * Car.PID.Velocity_Kp;
//	LeftIValue = LeftSpeedDelta * Car.PID.Velocity_Ki;
//	g_LeftSpeedControlIntegral += LeftIValue;
	
	g_LeftSpeedControlOutOld = g_LeftSpeedControlOutNew;
	g_LeftSpeedControlOutNew = Car_LeftVelocityPIDCalc(Car.Motor.LeftSpeed); //(int16_t)(LeftPValue + g_LeftSpeedControlIntegral);
	
	/*  �ұߵ���ٶȻ�����  */
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
* Description: ���ٶȻ�������ֳ��������,��ֹ�ٶ�ͻ��
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
	/*  ����С��������������  */
	bsp_sensor_DataProcess();
	
	/*  ����С������ٶ�  */

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
	
	/*  ������ߵ�������  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
<<<<<<< HEAD
	/*  �ٶȼ���  */
	bsp_encoder_SpeedCalc();
=======
	/*  �����ұߵ�������  */
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
	
	Car.PID.Error = Car.HorizontalAE - LastError;		/*  ���㵱ǰ΢����  */
	
	g_DirctionControlPeriod = g_DirctionControlOutNew;
	/*  ����PWM  */
	g_DirctionControlOutNew = (int16_t)((Car.HorizontalAE*10 *  Car.PID.Kp_Straight) + (Car.PID.Error * Car.PID.Kd_Straight));
	
	/*  �����ϸ�ʱ�̵����  */
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
* Description: �������ٶȵ��Ӻ���������
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
	/*  ���ٶȻ���ת�򻷵�PWM��������  */
	Car.Motor.LeftPwm = g_LeftSpeedControlOut - g_DirctionControlOut;
	Car.Motor.RightPwm = g_RightSpeedControlOut + g_DirctionControlOut;
	
	/*  �޷�  */
	if(Car.Motor.LeftPwm > Car.MaxPWM) Car.Motor.LeftPwm = Car.MaxPWM;
	else if(Car.Motor.LeftPwm < - Car.MaxPWM) Car.Motor.LeftPwm = -Car.MaxPWM;
	
	if(Car.Motor.RightPwm > Car.MaxPWM) Car.Motor.RightPwm = Car.MaxPWM;
	else if(Car.Motor.RightPwm < - Car.MaxPWM) Car.Motor.RightPwm = -Car.MaxPWM;
	
	/*  ��������  */
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
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
	

