/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.2.0
  * Date: 2018-3-3
  * Brief: ���ļ����ڳ������ݼ�¼����������ȣ�ͬʱ���ӿ��ƺ���Ҳ�ڱ��ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �����ļ�
  *
	*		2.Author: Vector
	*			Date:	2018-3-19
	*			Mod: 1.�޸ĳ��ӿ����߼�,�ı�Ѱ�߲���
	*
	*		3.Author: Vector
	*			Date: 2018-3-24
	*			Mod: �޸��ٶȻ�,���򻷴���,�����ٶ���������
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
static int16_t g_LeftSpeedControlOut = 0;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static int16_t g_RightSpeedControlOut = 0;

/*  С��������Ʊ���  */
static int16_t g_DirctionControlOut = 0;
static int16_t g_DirctionControlCounter = 0;
static int16_t g_DirctionControlPeriod = 0;
static int16_t g_DirctionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;
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
	
	/*  С��Ŀ���ٶ�  */
	Car.TargetSpeed = STRAIGHT_SPEED;
	Car.LeftTargetSpeed = STRAIGHT_SPEED;
	Car.RightTargetSpeed = STRAIGHT_SPEED;
	
	/*  С����ʼ��·Ϊֱ��  */
	Car.NowRoad = Road_Straight;
	
	/*  С����ʼδ����  */
	Car.LossLine = LostLine_None;
	
	Car.MaxPWM = 800;
	
	Car.Running = 1;
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
*                           Car_ControlStop               
*
* Description: ֹͣС������,���ڰ�������ʱ��ͣС������
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
	DRV_DISABLE();							/*  �ر�����  */
	bsp_tim_DeleteHardTimer(1);	/*  ֹͣС�������ж�  */
}

/*
*********************************************************************************************************
*                        Car_Start                  
*
* Description: ����С������,����
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
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed);
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 100 || SpeedError >= -100)
		SpeedIntegal += SpeedFilter;
	
	/*  �����޷�  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//�����޷�
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
	/*  �ٶȻ�PD����,ʵ����Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) + SpeedIntegal * (-Car.PID.Velocity_Ki));	//�ٶȻ�PID����	
		
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
	float SpeedError = 0;
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed);
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	if(SpeedError < 10 || SpeedError >= -10)
		SpeedIntegal += SpeedFilter;
	
	/*  �����޷�  */
	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//�����޷�
	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
	
	/*  �ٶȻ�PD����,ʵ����Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * (-Car.PID.Velocity_Kp) +
							SpeedIntegal * (-Car.PID.Velocity_Ki));//�ٶȻ�PID����	
		
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
	/*  ��ߵ���ٶȻ�����  */
	Car.Motor.LeftSpeed = (float)(Car.Motor.LeftEncoder * CAR_SPED_CONSTANT);
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	Car.Motor.LeftEncoder = 0;
		
	g_LeftSpeedControlOutOld = g_LeftSpeedControlOutNew;
	g_LeftSpeedControlOutNew = Car_LeftVelocityPIDCalc(Car.Motor.LeftSpeed);
	
	
	/*  �ұߵ���ٶȻ�����  */
	Car.Motor.RightSpeed = (float)(Car.Motor.RightEncoder * CAR_SPED_CONSTANT);
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	Car.Motor.RightEncoder = 0;
	
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
	int16_t SpeedControlValue;
	
	/*  ������ߵ�������  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
	/*  �����ұߵ�������  */
	SpeedControlValue = g_RightSpeedControlOutNew - g_RightSpeedControlOutOld;
	g_RightSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_RightSpeedControlOutOld;
}


/*
*********************************************************************************************************
*                       Car_DirctionControl                   
*
* Description: С�����򻷿��ƺ���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirctionControl(void)
{
	static float LastError, Error;
	
	Error = Car.HorizontalAE*10 - LastError;		/*  ���㵱ǰ΢����  */
	
	
	if(Car.Sensor[SENSOR_H_L].Average < 11 && Car.Sensor[SENSOR_H_R].Average < 11)
		Car.Running ++;
		
	/*  �����ϴε�PWM  */
	g_DirciotnControlOutOld = g_DirctionControlOutNew; 
	
	/*  ����PWM  */
	g_DirctionControlOutNew = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Straight) + (Error * Car.PID.Kd_Straight));
	
	if(Car.HorizontalAE*100 < 4 && Car.HorizontalAE*100>-4) g_DirctionControlOutNew = 0;
	/*  �����ϸ�ʱ�̵����  */
	LastError = (float)(Car.HorizontalAE * 10);
}

/*
*********************************************************************************************************
*                      Car_DirctionControlOutput                    
*
* Description: ���򻷿������,�����򻷵��������Ϊ�������ڵ�n�ȷ����,���ٶ�ƽ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_DirctionControlOutput(void)
{
	int16_t DirctionOutput = 0;
	
	DirctionOutput = (int16_t)(g_DirctionControlOutNew - g_DirciotnControlOutOld);
	g_DirctionControlOut = (int16_t)(DirctionOutput * (g_DirctionControlPeriod + 1) / DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld);
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
	int16_t LeftPwm = 0, RightPwm = 0;
	/*  ���ٶȻ���ת�򻷵�PWM��������  */
	LeftPwm = (int16_t)(g_LeftSpeedControlOut + g_DirctionControlOut);
	RightPwm = (int16_t)(g_RightSpeedControlOut - g_DirctionControlOut);
	
	/*  �޷�  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
		
	/*  ��������  */
	bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}

/*
*********************************************************************************************************
*                       Car_Control                   
*
* Description: С���ܿ��ƺ���
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
	
	if(Car.Running > 2)
	{
//		Car.LeftTargetSpeed = 0;
//		Car.RightTargetSpeed = 0;
		bsp_motor_SetPwm(0,0);
		return;
	}
	

	/*  ���Ƽ�����  */
	CarControlCunter++;
	
	/*  �ٶȿ������  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  ����������  */
	g_DirctionControlPeriod++;
	Car_DirctionControlOutput();
	
	/*  ����״̬��  */
	switch(CarControlCunter)
	{
		/*  ÿ5ms��ȡһ�α�����  */
		case 1: bsp_encoder_ReadCounter();break;
		
		/*  ÿ20ms����һ���ٶȿ���  */
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
		
		/*  ÿ10ms����һ�η������  */
		case 3:
		{
			g_DirctionControlCounter++;
			if(g_DirctionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
			{
				g_DirctionControlCounter = 0;
				g_DirctionControlPeriod = 0;
				Car_DirctionControl();
			}
		}break;/*  end of case 3  */
		/*  ÿ5ms����һ�δ��������ݴ���  */
		case 4:bsp_sensor_DataProcess();break;
		
		/*  ÿ5ms����һ�ε�����  */
		case 5:
		{
			Car_MotorOutput();
			CarControlCunter=0;
		}break;
		default:
		{
			CarControlCunter = 0;
			g_DirctionControlCounter = 0;
			g_SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirctionControlPeriod = 0;
		}break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

