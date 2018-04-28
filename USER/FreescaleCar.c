/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V1.6.1
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
	*		7.Author: Vector
	*     Date: 2018-4-19
	*			Mod: 1.Ŀ���ٶȸ���Vs = V + k*Error,��������:g_SpeedFactor
	*					 2.ת����Ʋ��÷ֶ�PID
	*					 3.���ӵڶ���ˮƽ����Լ�ǰ��ˮƽ�м���,����ǰ�����ŵĺͲ�ȵĵĲ�ֵ���ж�������
	*          4.����ʶ��Բ������,����ǰ���м���,����׼ȷ�ʽϸ�,���޻��������жϡ�����������
	*					 5.����MPU����,����Z����ٶȸ���ת��
	*
	*		8.Author: Vector
	*			Date: 2018-4-23
	*			Mod: ת������ɷֶ�PID��Ϊģ��PID,Ч���Ϻ�
	*
	*		9.Author: Vector
	*			Date: 2018-4-28
	*			Mod: 1.������������ٶȻ���Ϊһ��,����λ��ʽPID��Ϊ����ʽPID
	*					 2.�ٶȸ�Ϊ���������ƽ��ת��
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
# include "math.h"

/*  �ٶȿ�������*/
# define SPEED_CONTROL_PERIOD	20	

/*  �����������,��λms  */
# define DIRCTION_CONTROL_PERIOD	5  


/*  С�����ƽṹ��  */
Car_TypeDef Car;

Kalman1Dim_TypeDef Kalman_Gryoz;

/*  С���ٶȿ��Ƽ�����  */
static uint16_t g_SpeedControlCounter = 0;
/*  С���ٶ������������  */
static uint16_t g_SpeedControlPeriod = 0;


/*  С���ٶȻ�����PWM���  */
static int16_t g_SpeedControlOutInc = 0;
static int16_t g_SpeedControlOutNew = 0;
static int16_t g_SpeedControlOutOld = 0;
static int16_t g_SpeedControlOut = 0;
static int16_t g_SpeedCounter = 0;
static float g_SpeedDeltaFactor = 0;

/*  С��������Ʊ���  */
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlCounter = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;


/*  ��Z����ٶ�����ת��Kdϵ��  */
const static float g_GryoZ_Kd = 0.09;


/*  �ܳ��ܵ�������,����ֵ����һ����Χ��ͣ��  */
static uint16_t g_OutCounter = 0;

static float g_SpeedFactor = 0.38;//0.48;
static float g_HAEFactor = 0.08;

uint16_t g_CircleSpeedError = 0;


 

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
	
		/*  ��ʼ�����ӵ�PID����,��Flash�ж�ȡ�������PID����  */
	Car.PID.SpeedKp = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.SpeedKi = 0.13;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.SpeedKd = 0;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  ֱ��PID  */
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  ��ʼ������ģ��PID����  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 10;
	Car.DirFuzzy.DErrMax = 30;
	Car.DirFuzzy.ErrMax = 80;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = 38;//Car.PID.DirectionKp;
	Car.DirFuzzy.KIMax = 0;//Car.PID.DirectionKi;
	Car.DirFuzzy.KDMax = 480;//Car.PID.DirectionKd;
	fuzzy_PIDInit(&Car.DirFuzzy);
	
	/*  ��ʼ���ٶȿ���ģ��PID����  */
	Car.LVFuzzy.DeltaKdMax = 20;
	Car.LVFuzzy.DeltaKiMax = 0.2;
	Car.LVFuzzy.DeltaKpMax = 30;
	Car.LVFuzzy.DErrMax = 50;
	Car.LVFuzzy.ErrMax = 50;
	Car.LVFuzzy.KP = 26;
	Car.LVFuzzy.KI = 0.1;
	Car.LVFuzzy.KD = 20;
	Car.LVFuzzy.KPMax = Car.PID.SpeedKp;
	Car.LVFuzzy.KIMax = Car.PID.SpeedKi;
	Car.LVFuzzy.KDMax = Car.PID.SpeedKd;
	fuzzy_PIDInit(&Car.LVFuzzy);
	
	/*  ��ʼ���ٶȿ���ģ��PID����  */
	Car.RVFuzzy.DeltaKdMax = 20;
	Car.RVFuzzy.DeltaKiMax = 0.2;
	Car.RVFuzzy.DeltaKpMax = 30;
	Car.RVFuzzy.DErrMax = 50;
	Car.RVFuzzy.ErrMax = 50;
	Car.RVFuzzy.KP = 26;
	Car.RVFuzzy.KI = 0.1;
	Car.RVFuzzy.KD = 20;
	Car.RVFuzzy.KPMax = Car.PID.SpeedKp;
	Car.RVFuzzy.KIMax = Car.PID.SpeedKi;
	Car.RVFuzzy.KDMax = Car.PID.SpeedKd;
	fuzzy_PIDInit(&Car.RVFuzzy);
	
	
	/*  ��ʼ�����ӵĺͲ��  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
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
		

	/*  ������Ʋ�����ʼ��  */
	Car.Motor.PWM_Frequency = 10;	/*  ���PWMƵ��Ϊ10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	/*  С��Ŀ���ٶ�  */
	Car.TargetSpeed = 10;
		
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
*                          Car_SpeedControl                
*
* Description: С���ٶȻ�����,��������ʽPID
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
	static  float SpeedErrorK_1=  0, SpeedErrorK_2 = 0;	
	float SpeedErrorK = 0;
	volatile int16_t LeftEnconder = 0, RightEnconder = 0;
	volatile float Kp = 0, Ki = 0, Kd = 0;
	
//	if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 20 || 
//		Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 20)
//	{
//		if(Car.HorizontalAE < 0)
//			Car.TargetSpeed = 10 + (g_SpeedCounter / 2) * 5 + g_SpeedFactor * Car.HorizontalAE;
//		else
//			Car.TargetSpeed = 10 + (g_SpeedCounter / 2) * 5 - g_SpeedFactor * Car.HorizontalAE;
//		
//		if(g_SpeedCounter++ >= 2) g_SpeedCounter = 0;

//	}
//	else 
//	if(Car.AE > 20 || Car.AE < -20)
//	{
//		if(Car.AE < 0)
//			Car.TargetSpeed = 10 + g_HAEFactor * Car.AE;
//		else
//			Car.TargetSpeed = 10 - g_HAEFactor * Car.AE;
//	}
//	else
//	{
//		Car.TargetSpeed = 10;
//	}
//	

//	/*  ���ڷ�ֹԲ�����ٶ�ƫ��  */
//	if(Car.Sensor[SENSOR_H_L].Average > 85 && Car.Sensor[SENSOR_H_R].Average > 85)
//	{
//		Car.TargetSpeed = 10;
//	}
	

	/*  ��Ϊ������������ת��180��,���Լ����෴  */
	LeftEnconder= 0;
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 0) ? (-Car.Motor.LeftEncoder) : Car.Motor.LeftEncoder;
	Car.Motor.LeftEncoder = 0;
	
	RightEnconder = 0;
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 1) ? (-Car.Motor.RightEncoder) : Car.Motor.RightEncoder;
	Car.Motor.RightEncoder = 0;
	
	/*  �����ٶ�Ϊ��������ȡƽ��  */
	Car.CarSpeed = (LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
	
	SpeedErrorK = Car.TargetSpeed - Car.CarSpeed;
	
	Kp = 0;
	Ki = 0.12;
	Kd = 0;

//	g_SpeedControlOutInc =0;

	/*  ����ʽPID����  */
	g_SpeedControlOutInc = Kp * (SpeedErrorK - SpeedErrorK_1) + 
												 Ki * SpeedErrorK + 
												 Kd * (SpeedErrorK - 2 * SpeedErrorK_1 + SpeedErrorK_2);
												 
	SpeedErrorK_2 = SpeedErrorK_1;											 
	SpeedErrorK_1 = SpeedErrorK;
	
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
	g_SpeedControlOut += g_SpeedControlOutInc;
	
	if(g_SpeedControlOut > 500) g_SpeedControlOut = 500;
	else if(g_SpeedControlOut < -500) g_SpeedControlOut = -500;
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
	static float LastError, KdOutLast = 0;
	static float k = 0.3;		/*  ��ֵΪ����ȫ΢���˲�ϵ��  */
	float Error, ErrorDiff;		/*  ƫ��,ƫ��΢��  */
	float Kp = 0, Kd = 0;		
	int16_t KpOut = 0, KdOutNow = 0, KdGryozOut = 0;

	Error = Car.HorizontalAE;
	ErrorDiff = Error - LastError;
	
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	/*  ����ģ��PID�������ֵ�и���,������Ҫ�����ж�  */
	Kp = (Car.DirFuzzy.KP < 0) ? (-Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (-Car.DirFuzzy.KD) : Car.DirFuzzy.KD;
	

	/*  ���ߵĵ��ֵ��С����ֵ,˵�����ܵ���  */
	if(Car.Sensor[SENSOR_H_L].Average < 16 && Car.Sensor[SENSOR_H_R].Average < 16 && 
		Car.Sensor[SENSOR_V_L].Average < 16 && Car.Sensor[SENSOR_V_R].Average < 16)
		g_OutCounter++;
		
	KdOutLast = KdOutNow;		/*  ���ڲ��ò���ȫ΢��,������Ҫ������һʱ�̵�΢��  */
	KpOut = (Error) * Kp;		/*  ת�򻷵ı���  */
	KdOutNow = ErrorDiff * Kd;				/*  ת�򻷵�΢��  */
	KdGryozOut = g_GryoZ_Kd * (Car.MPU.Gryoz - MPU_GRYOZ_ZERO);		/*  ���������ǵĽ��ٶȽ��в���,����ת��  */
		
	
	/*  �����ϴε�PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  ����PWM,���ò���ȫ΢��PID  */
	g_DirectionControlOutNew = (int16_t)(k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut);
	
	/*  ��ƫ��С��һ����Χʱ�ر����,���Լ���ϵͳ��  */
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
	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut - g_CircleSpeedError);
	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut + g_CircleSpeedError);	
	
//		if(Car.Sensor[SENSOR_H_L].Average - Car.Sensor[SENSOR_H_R].Average > 20 )
//	{
//		/*  ���ٶȻ���ת�򻷵�PWM��������  */
//		LeftPwm = (int16_t)((1 + 0.3) *LeftPwm);
//		RightPwm = (int16_t)((1 + 0.3) * RightPwm);
//	}
//	else if(Car.Sensor[SENSOR_H_R].Average - Car.Sensor[SENSOR_H_L].Average > 20 )
//	{
//		/*  ���ٶȻ���ת�򻷵�PWM��������  */
//		LeftPwm = (int16_t)((1 + 0.3) * LeftPwm);
//		RightPwm = (int16_t)((1 + 0.3) * RightPwm);		
//	}
	
		
	/*  �޷�  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
		
	/*  �����߼�������ֵ����2ʱ,˵���Ѿ�������ܵ�,ͣ��  */
	if(g_OutCounter > 2)
		bsp_motor_SetPwm(0,0);
	else
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
	/*  ��������  */
		
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
	
	/*  ���Ƽ�����  */
	CarControlCunter++;
	
	/*  �ٶȿ������  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  ����������  */
	g_DirectionControlPeriod++;
	Car_DirectionControlOutput();
	
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
	

