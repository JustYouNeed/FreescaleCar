/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.3.0
  * Date: 2018-3-3
  * Brief: ���ļ����ڳ������ݼ�¼����������ȣ�ͬʱ���ӿ��ƺ���Ҳ�ڱ��ļ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-3
	*			Mod: �����ļ�
  *
	*		2.Author: Vector
	*			Date:	2018-3-19
	*			Mod: 1.�޸ĳ��ӿ����߼�,�ı�Ѱ�߲���
	*
	*		3.Author: Vector
	*			Date: 2018-3-24
	*			Mod: �޸��ٶȻ�,���򻷴���,�����ٶ���������
	*
	*		4.Author: Vector
	*			Date: 2018-3-25
	*			Mod: �����ܽ��ȶ�������,����:ת��,5600,0,18000,�ٶȻ�:245,1,0
	*
	*		5.Author: Vector
	*			Date: 2018-3-26
	*			Mod: 1.�����ٶȱ仯���ɷŴ�10����Ϊ�Ŵ�100��,���ڵ���΢��ϵ��
	*					 2.����PID�ɳ���PID��Ϊ����ȫ΢��PID
	*					 3.���������⹦��,�����º�����·��⺯��Car_RoadDetect
	*
	*		6.Author: Vector
	*			Date: 2018-3-29
	*			Mod: 1.�ٶȻ���˫�ٶȻ���Ϊ���ٶȻ�,PID����:ת�� 193,0,380,�ٶ� 245,1,0
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

/*  С�����ƽṹ��  */
Car_TypeDef Car;

/*  С���ٶȿ��Ƽ�����  */
static uint16_t g_SpeedControlCounter = 0;
/*  С���ٶ������������  */
static uint16_t g_SpeedControlPeriod = 0;


/*  С���ٶȻ�����PWM���  */
static int16_t g_SpeedControlOutNew = 0;
static int16_t g_SpeedControlOutOld = 0;
static int16_t g_SpeedControlOut = 0;

static int16_t g_LeftSpeedControlOutNew = 0;
static int16_t g_LeftSpeedControlOutOld = 0;
static int16_t g_LeftSpeedControlOut = 0;
static int16_t g_RightSpeedControlOutNew = 0;
static int16_t g_RightSpeedControlOutOld = 0;
static int16_t g_RightSpeedControlOut = 0;
static int16_t g_SpeedCounter = 0;

/*  С��������Ʊ���  */
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlCounter = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;


/*  ��Z����ٶ�����ת��Kdϵ��  */
const static float g_GryoZ_Kd = 0.09f;

/*  ���߼�����,�ɸñ�����ͳ��ƫ������һ����Χ�Ĵ���,��ֵԽС,˵��ƫ������ԽС  */
static int16_t g_LossLineCounter = 0;

static float g_CarCurveSpeedLow = 0;
static float g_CarCurveSpeedHigh = 0;
static float g_CarStraightSpeed = 0;

static uint8_t g_CarLastDirection = 0;

/*  �ܳ��ܵ�������,����ֵ����һ����Χ��ͣ��  */
static uint16_t g_OutCounter = 0;
static uint16_t g_StaightFlag = 0;
static uint8_t g_LeftCurveFlag = 0;
static uint8_t g_RightCurveFlag = 0;

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
		
	Car.PID.SpeedKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.SpeedKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.SpeedKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  ֱ��PID  */
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  ��ʼ�����ӵĴ���������,��Flash�ж�ȡ�궨ֵ  */
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
	
//	Car.HorizontalAEMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, SENSOR_COUNT * 8 , float);
//	Car.VecticalAEMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, SENSOR_COUNT * 8 + 4, float);
//	
//	Car.Sensor[SENSOR_H_L].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 0, uint16_t);
//	Car.Sensor[SENSOR_H_L].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 4, uint16_t);
//	
//	Car.Sensor[SENSOR_H_R].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 8, uint16_t);
//	Car.Sensor[SENSOR_H_R].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 12, uint16_t);
//	
//	Car.Sensor[SENSOR_V_L].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 16, uint16_t);
//	Car.Sensor[SENSOR_V_L].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 20, uint16_t);
//	
//	Car.Sensor[SENSOR_V_R].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 24, uint16_t);
//	Car.Sensor[SENSOR_V_R].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, 28, uint16_t);
	

	/*  ������Ʋ�����ʼ��  */
	Car.Motor.PWM_Frequency = 10;	/*  ���PWMƵ��Ϊ10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	/*  С��Ŀ���ٶ�  */
	Car.TargetSpeed = 25;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, float);
	Car.LeftTargetSpeed = 25;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 4, float);
	Car.RightTargetSpeed = 25;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 8, float);
	
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
	
	
	/*  ���泵������Ŀ���ٶ�  */
	temp[cnt++] = BYTE4((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.TargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.TargetSpeed);
	
	/*  ���泵�����Ŀ���ٶ�  */
	temp[cnt++] = BYTE4((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.LeftTargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.LeftTargetSpeed);
	
	/*  ���泵���ұ�Ŀ���ٶ�  */
	temp[cnt++] = BYTE4((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE3((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE2((uint32_t)Car.RightTargetSpeed);
	temp[cnt++] = BYTE1((uint32_t)Car.RightTargetSpeed);
	
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);	/*  �Ȳ���һ��,��Ȼ�޷�д��  */
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, temp, 48, 0);
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
* Description: ����С������,��������PID����
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
	
		if(SpeedError > 3)
	{
		/*  �ٶȻ�PD����,ʵ����Ki = 0  */
		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
								SpeedIntegal * (-Car.PID.SpeedKi));//�ٶȻ�PID����	
	}else
	{
				/*  �ٶȻ�PD����,ʵ����Ki = 0  */
		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
								SpeedIntegal * (-Car.PID.SpeedKi));
	}
		
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
	
	if(SpeedError > 3)
	{
		/*  �ٶȻ�PD����,ʵ����Ki = 0  */
		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
								SpeedIntegal * (-Car.PID.SpeedKi));//�ٶȻ�PID����	
	}else
	{
				/*  �ٶȻ�PD����,ʵ����Ki = 0  */
		Velocity = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +
								SpeedIntegal * (-Car.PID.SpeedKi));
	}
		
	return Velocity;
}
/*
*********************************************************************************************************
*                          Car_SpeedControl                
*
* Description: С���ٶȻ�����,���û��ַ���PI����
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
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	
	/*  ��ߵ���ٶȻ�����  */
	Car.Motor.LeftSpeed = (float)(Car.Motor.LeftEncoder * CAR_SPEED_CONSTANT);
	if(drv_gpio_ReadPin(LEFTENCONDER_DIR_PIN) == 0) Car.Motor.LeftSpeed = -Car.Motor.LeftSpeed;
	Car.Motor.LeftEncoder = 0;
	
	
	g_LeftSpeedControlOutOld = g_LeftSpeedControlOutNew;
	g_LeftSpeedControlOutNew = Car_LeftVelocityPIDCalc(Car.Motor.LeftSpeed); 
	
	/*  �ұߵ���ٶȻ�����  */
	Car.Motor.RightSpeed = (float)(Car.Motor.RightEncoder * CAR_SPEED_CONSTANT);
	if(drv_gpio_ReadPin(RIGHTENCONDER_DIR_PIN) == 1) Car.Motor.RightSpeed = -Car.Motor.RightSpeed;
	Car.Motor.RightEncoder = 0;
	
	
	g_RightSpeedControlOutOld = g_RightSpeedControlOutNew;
	g_RightSpeedControlOutNew = Car_RightVelocityPIDCalc(Car.Motor.RightSpeed);
	
//	Car.CarSpeed = (Car.Motor.LeftSpeed + Car.Motor.RightSpeed)/2;
//	
//		/*  �ٶ�ƫ��  */
//	SpeedError = (float)(Car.CarSpeed - Car.TargetSpeed);
//	
//	/*  ��ͨ�˲�,���ٶ�ƽ������  */
//	SpeedFilter *= 0.7;
//	SpeedFilter += (SpeedError * 0.3);
//	
//	if(SpeedError < 10 || SpeedError >= -10)
//		SpeedIntegal += SpeedFilter;
//	
//	/*  �����޷�  */
//	if(SpeedIntegal > 3000) SpeedIntegal = 3000;			//�����޷�
//	else if(SpeedIntegal < -3000) SpeedIntegal = -3000;		//
//	
//	if(SpeedError > 3)
//	{
//		/*  �ٶȻ�PI����  */
//		g_SpeedControlOutOld = g_SpeedControlOutNew;
//		g_SpeedControlOutNew = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp*4) + SpeedIntegal * (-Car.PID.SpeedKi));//�ٶȻ�PID����	
//	}else
//	{
//				/*  �ٶȻ�PI���� */
//		g_SpeedControlOutOld = g_SpeedControlOutNew;
//		g_SpeedControlOutNew = (int16_t)(SpeedFilter * (-Car.PID.SpeedKp) +	SpeedIntegal * (-Car.PID.SpeedKi));
//	}
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
	
//	SpeedControlValue = g_SpeedControlOutNew - g_SpeedControlOutOld;
//	g_SpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_SpeedControlOutOld;
	/*  ������ߵ�������  */
	SpeedControlValue = g_LeftSpeedControlOutNew - g_LeftSpeedControlOutOld;
	g_LeftSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_LeftSpeedControlOutOld;
	
	/*  �����ұߵ�������  */
	SpeedControlValue = g_RightSpeedControlOutNew - g_RightSpeedControlOutOld;
	g_RightSpeedControlOut = SpeedControlValue * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_RightSpeedControlOutOld;
}


/*
*********************************************************************************************************
*                       Car_DirectionControl                   
*
* Description: С�����򻷿��ƺ���,PID���ò���ȫ΢��PD����,ͬʱ���������ǵĽ��ٶ���
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
	static float k = 0.3f;
	
	/*  ����ұߵĵ�й�һ��ֵ������ߵĵ��,˵����������ת  */
//	if(Car.Sensor[SENSOR_H_R].NormalizedValue - Car.Sensor[SENSOR_H_L].NormalizedValue >=20) g_CarLastDirection = 1;

//	/*  ��������ת  */
//	if(Car.Sensor[SENSOR_H_L].NormalizedValue - Car.Sensor[SENSOR_H_R].NormalizedValue >=20) g_CarLastDirection = 0;
	
	Error = Car.HorizontalAE - LastError;		/*  ���㵱ǰ΢����  */
	
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
	
	/*  ���ߵĵ��ֵ��С����ֵ,˵�����ܵ���  */
	if(Car.Sensor[SENSOR_H_L].Average < 9 && Car.Sensor[SENSOR_H_R].Average < 9)
		g_OutCounter++;
		
	
	KdOutLast = KdOutNow;		/*  ���ڲ��ò���ȫ΢��,������Ҫ������һʱ�̵�΢��  */
	KpOut = Car.HorizontalAE * Kp;		/*  ת�򻷵ı���  */
	KdOutNow = Error * Kd;				/*  ת�򻷵�΢��  */
	KdGryozOut = g_GryoZ_Kd * (Car.MPU.Gryoz - MPU_GRYOZ_ZERO);		/*  ���������ǵĽ��ٶȽ��в���,����ת��  */
		
	
	/*  �����ϴε�PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  ����PWM,���ò���ȫ΢��PID  */
	g_DirectionControlOutNew = (int16_t)(k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut);
	
	/*  ��ƫ��С��һ����Χʱ�ر�΢��,���Լ���ϵͳ��  */
	if(Car.HorizontalAE < 4 && Car.HorizontalAE > -4) g_DirectionControlOutNew = 0;

	/*  �����ϸ�ʱ�̵����  */
	LastError = Car.HorizontalAE;
}

/*
*********************************************************************************************************
*                      Car_DirectionControlOutput                    
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
	LeftPwm = (int16_t)(g_LeftSpeedControlOut + g_DirectionControlOut);
	RightPwm = (int16_t)(g_RightSpeedControlOut - g_DirectionControlOut);
	
//	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut);
//	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut);
	
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



//void Car_RoadDetect(void)
//{
//	uint8_t TurnDirection = 0;
//	
//	/*  �������ߵĵ��ֵ��,˵�����������,���м���,�������PID  */
//	if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 10 || 
//		 Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 10)
//	{
//		Car.TargetSpeed = CURVE_SPEED_LOW;
//		Car.PID.DirectionKp = BIG_CURVE_KP;
//		Car.PID.DirectionKd = BIG_CURVE_KD;
//	}
//	
//	/*  ���ߵĵ��ֵ���������ߵĸ���  */
//	if(Car.Sensor[SENSOR_H_L].Average > 90 && Car.Sensor[SENSOR_H_R].Average > 90)
//	{
//		/*  �����ƫ��,���ۻ�ƫ��,��ֵԽС,��˵��ƫ������Խ��  */
//		if(Car.HorizontalAE < 20)
//			g_LossLineCounter++;
//		else
//			g_LossLineCounter = 0;
//		
//		/*  ������ֵ��С,˵���Ѿ�����,����  */
//		if(g_LossLineCounter < 5)
//		{
//			Car.TargetSpeed = g_CarCurveSpeedLow;
//			Car.PID.DirectionKp = STRAIGHT_KP;
//			Car.PID.DirectionKd = STRAIGHT_KD;
//		}
//		else if(g_LossLineCounter > 5 && g_LossLineCounter < 30)	/*  ƫ�����߽ϴ�  */
//		{
//			Car.TargetSpeed = g_CarCurveSpeedLow;
//			Car.PID.DirectionKp = BIG_CURVE_KP;
//			Car.PID.DirectionKd = BIG_CURVE_KD;
//		}
//		else if(g_LossLineCounter > 30 && g_LossLineCounter < 50)	/*  ƫ�����߲��Ǻܶ�  */
//		{
//			Car.TargetSpeed = g_CarCurveSpeedHigh;
//			Car.PID.DirectionKp = SMALL_CURVE_KP;
//			Car.PID.DirectionKd = SMALL_CURVE_KD;
//		}else if(g_LossLineCounter > 50 && g_LossLineCounter < 60)	/*  ��ƫ��,���Ǻ�С  */
//		{
//			Car.TargetSpeed = g_CarStraightSpeed;
//			Car.PID.DirectionKp = STRAIGHT_KP;
//			Car.PID.DirectionKd = STRAIGHT_KD;
//		}
//		else 		/*  û��ƫ������  */
//		{
//			Car.TargetSpeed = g_CarStraightSpeed;
//			Car.PID.DirectionKp = STRAIGHT_KP;
//			Car.PID.DirectionKd = STRAIGHT_KD;
//		}
//	}
//}

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
	
	/*  ���Ƽ�����  */
	CarControlCunter++;
	
	/*  �ٶȿ������  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  ����������  */
	g_DirectionControlPeriod++;
	Car_DirectionControlOutput();
	
	if(g_OutCounter > 2)		/*  ���ӳ���ܵ�������Ŀ���ٶ�Ϊ0,ͬʱ�ر�ת�����,�Ա����ͣ��  */
	{
		bsp_motor_SetPwm(0,0);
//		Car.TargetSpeed = 0;
		Car.LeftTargetSpeed = 0;
		Car.RightTargetSpeed = 0;
		g_DirectionControlOut = 0;
		g_DirciotnControlOutOld = 0;
		g_DirectionControlOutNew = 0;
		Car.PID.DirectionKd = 0;
		Car.PID.DirectionKp = 0;
		return;
	}
	else
	{
		if((Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average) > 15)
		{
			g_LeftCurveFlag++;
			if(g_LeftCurveFlag < 15)
			{
				Car.TargetSpeed = 10;
				Car.LeftTargetSpeed = 5;
				Car.RightTargetSpeed =70;
			}else 
			{
				Car.TargetSpeed = 25;
				Car.LeftTargetSpeed = 25;
				Car.RightTargetSpeed = 35;
			}
			g_StaightFlag = 0; 
		}else if((Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average) > 15)
		{
			g_RightCurveFlag++;
			if(g_RightCurveFlag < 15)
			{
				Car.TargetSpeed = 10;
				Car.LeftTargetSpeed = 5;
				Car.RightTargetSpeed =10;
			}else 
			{
				Car.TargetSpeed = 25;
				Car.LeftTargetSpeed = 35;
				Car.RightTargetSpeed = 25;
			}
			g_StaightFlag = 0; 
		}
		else		/*  ֱ������,����ֵ����һ����ֵ��ʱ�����Ϊ��ֱ��  */
		{
			g_StaightFlag ++;
			g_LeftCurveFlag = 0;
			g_RightCurveFlag = 0;
			if(g_StaightFlag > 3)
			{
				Car.TargetSpeed = 45;
				Car.LeftTargetSpeed = 40;
				Car.RightTargetSpeed = 40;
			}
		}
	}
	
	/*  ����״̬��  */
	switch(CarControlCunter)
	{
		/*  ÿ5ms��ȡһ�α�����  */
		case 1: bsp_encoder_ReadCounter();break;
		
		/*  �ٶȿ���  */
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
		
		
		/*  ÿ5ms����һ�δ��������ݴ���  */
		case 3:bsp_sensor_DataProcess();break;
		
		/*  �������  */
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
		
		/*  ÿ5ms����һ�ε�����  */
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
	

