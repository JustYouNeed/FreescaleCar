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
	*			Mod: �޸��ٶȻ�,���򻷴���
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
	*		10.Author: Vector
	*			 Date: 2018-4-29
	*			 Mod: ��д�ٶȻ�,�����ٶȲ��ܿ���,�ٶȻ�������
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
# define SPEED_CONTROL_PERIOD	  30	

/*  �����������,��λms  */
# define DIRCTION_CONTROL_PERIOD	5  

/*  ���ӳ��ܵ���ĵ��ֵ  */
# define LOST_LINE_THRESHOLD		16


/*  �ٶ�ת����������,������ɺ��ٶȵ�λΪ ת��  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)*ENCONDER_TEETH/WHEEL_TEETH
/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/

/*  С�����ƽṹ��  */
Car_TypeDef Car;

/*  С���ٶȻ�����PWM���  */
static float g_SpeedControlOut = 0;						/*  ���յ��ٶȻ����  */
static float g_SpeedControlOutNew = 0;				/*  �����ٶȻ������  */
static float g_SpeedControlOutOld = 0;				/*  �ϴ��ٶȻ������  */
static uint16_t g_SpeedControlPeriod = 0;			/*  �ٶȿ������ڼ�����,���ڽ��ٶȻ������ƽ������  */
static int16_t g_SpeedControlBangBang = 0;		/*  �������Ʊ���  */

/*  С��������Ʊ���  */
static float g_DirectionControlOut = 0;				/*  ���շ��򻷵����  */
static float g_DirectionControlOutNew = 0;		/*  ���η��򻷵����  */
static float g_DirciotnControlOutOld = 0;			/*  �ϴη��򻷵����  */
static uint16_t g_DirectionControlPeriod = 0;	/*  ����������ڼ�����,���ڽ�������Ƶ����ƽ������  */

static int16_t g_CurveSpeedControl = 0;
static uint8_t g_CurveStatus = 0;
static uint8_t g_NeedEnterCurve = 0;
static uint8_t g_NeedOutCurve = 0;
/*  ��Z����ٶ�����ת��Kdϵ��  */
const static float g_GryoZ_Kd = 0.08;


/*  �ܳ��ܵ�������,����ֵ����һ����Χ��ͣ��  */
static uint16_t g_LoseLineCounter = 0;

static float g_SpeedFactor = 0.16;//0.48;
static float g_HAEFactor = 0.08;

uint16_t g_CircleSpeedError = 0;

static float g_CurveDirection = 0;


 

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
	Car.PID.SpeedKp = 80.5;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.SpeedKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.SpeedKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);	/*  ֱ��PID  */
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  ��ʼ������ģ��PID����  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 10;
	Car.DirFuzzy.DErrMax = 50;
	Car.DirFuzzy.ErrMax = 80;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = 30;//Car.PID.DirectionKp;
	Car.DirFuzzy.KIMax = 0;//Car.PID.DirectionKi;
	Car.DirFuzzy.KDMax = 480;//Car.PID.DirectionKd;
	fuzzy_PIDInit(&Car.DirFuzzy);
	
	
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
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	/*  С��Ŀ���ٶ�  */
	Car.TargetSpeed = 15;
		
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
void Car_GetVoltage(void)
{
	uint16_t adc = 0;
	
	adc = drv_adc_ConvOnce(BAT_CHANNEL, ADC_Resolution_8b);
	Car.Voltage = (adc* 5.0f/ 255) / 0.379f ;
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
*                           Car_SpeedControl               
*
* Description: �����ٶȻ�����
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
	static float SpeedErrorK_1 = 0, SpeedErrorK_2 = 0;
	int32_t LeftEnconder = 0, RightEnconder = 0;
	float SpeedError = 0;
	int8_t temp;
	
		
	/*  ����������������ת��180��,������һ�����Բ�  */
	LeftEnconder = (READ_DIR(LEFTENCONDER_DIR_PIN) == 1) ? (Car.Motor.LeftEncoder) : (-Car.Motor.LeftEncoder);
	RightEnconder = (READ_DIR(RIGHTENCONDER_DIR_PIN) == 0) ? (Car.Motor.RightEncoder) : (-Car.Motor.RightEncoder);
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	
	/*  ���ٶȽ���ת��,�����ת/��  */
	Car.CarSpeed = (LeftEnconder + RightEnconder) / 2 * CAR_SPEED_CONSTANT;
	
	
	SpeedError = Car.TargetSpeed - Car.CarSpeed;
	
	
//	g_SpeedControlOut += Car.PID.SpeedKp * (SpeedError - SpeedErrorK_1) + Car.PID.SpeedKi * SpeedError + Car.PID.SpeedKd * (SpeedError - 2*SpeedErrorK_1 + SpeedErrorK_2);
	
	
//	SpeedErrorK_2 = SpeedErrorK_1;
//	SpeedErrorK_1 = SpeedError;
	
	/*  �������  */
	g_SpeedControlOutOld = g_SpeedControlOutNew;
	g_SpeedControlOutNew = SpeedError * Car.PID.SpeedKp;
	
//	if(SpeedError < -3) g_SpeedControlBangBang = -600;
//	else if(SpeedError > 5) g_SpeedControlBangBang = 100;
//	else g_SpeedControlBangBang = 0;
}

/*
*********************************************************************************************************
*                               Car_SpeedControlOutput           
*
* Description: С���ٶȿ���ƽ���������,���ٶȻ�������ֳɶ���������,���ٶȱ仯��ƽ��
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
	float SpeedControlOut = 0;
	
	SpeedControlOut = g_SpeedControlOutNew - g_SpeedControlOutOld;
	g_SpeedControlOut = SpeedControlOut * (g_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_SpeedControlOutOld;
}

/*
*********************************************************************************************************
*                       Car_RoadDetect                   
*
* Description: ����·���,�ж�С����ǰ��·
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void Car_RoadDetect(void)
{
	static uint32_t FirstTime = 0;
	
	
		/*  ���ߵĵ��ֵ��С����ֵ,˵�����ܵ���  */
	if(Car.Sensor[SENSOR_H_L].Average < 16 && Car.Sensor[SENSOR_H_R].Average < 16 )
		g_LoseLineCounter++;
	
	if(Car.Sensor[SENSOR_M].Average < 75 && Car.Sensor[SENSOR_H_L].Average < 100 && Car.Sensor[SENSOR_H_R].Average < 100)
	{
		Car.NowRoad = STRAIGHT;
	}
	
	
	/*  �ñ�־Ϊ0,˵����û���ҵ���һ����־��  */
	if(Car.Sensor[SENSOR_M].Average > 55 && g_CurveStatus == 0)
	{
		
		/*  �Ѿ��ҵ��˵�һ����־��,����׼������Բ��  */
		g_CurveStatus = 1;
		if(Car.Sensor[SENSOR_V_L].Average >100 && Car.Sensor[SENSOR_V_R].Average < 100)
			Car.NowRoad = LEFT_ISLAND;
		else if(Car.Sensor[SENSOR_V_R].Average > 100 && Car.Sensor[SENSOR_V_L].Average < 100)
			Car.NowRoad = RIGHT_ISLAND;
		FirstTime = bsp_tim_GetRunTime();
	}
	
//	if(Car.Sensor[SENSOR_M].Average > 85 && g_CurveStatus == 1)  g_CurveStatus = 2;
	
	if(Car.Sensor[SENSOR_M].Average > 55) 
	{
		bsp_beep_ON();
		if(Car.Sensor[SENSOR_V_L].Average > 80 ) g_CurveDirection++;
		else if(Car.Sensor[SENSOR_V_R].Average >80 ) g_CurveDirection --;
	}
	
	/*  �ҵ��˵�һ����־��,���ҵ��˵ڶ�����־��,�����Բ��  */
	if(g_CurveStatus == 1 && Car.Sensor[SENSOR_M].Average > 50 && Car.Sensor[SENSOR_M].Average < 80 && (bsp_tim_GetRunTime() - FirstTime) > 100)
	{		
//		if(g_CurveDirection < 0) Car.NowRoad = LEFT_ISLAND;
//		else Car.NowRoad = RIGHT_ISLAND;
		g_NeedEnterCurve = 1;		/*  ��ʾ��Ҫ����  */
	}
	
	if(g_NeedEnterCurve == 1 && Car.Sensor[SENSOR_M].Average < 75)
		g_NeedOutCurve = 1;
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
	static float LastError = 0, KdOutLast = 0;
	static const float k = 0.3;		/*  ��ֵΪ����ȫ΢���˲�ϵ��  */
	float Error, ErrorDiff;		/*  ƫ��,ƫ��΢��  */
	float Kp = 0, Kd = 0, Gyro_Z;		
	float KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
	static float temp = 0;
	
	switch(Car.NowRoad)
	{
		case STRAIGHT:temp = 0; g_CurveSpeedControl = 0; break;
		case LEFT_CURVE:break;
		case RIGHT_CURVE:break;
		case LEFT_ISLAND:
		{
			if(g_NeedEnterCurve == 1) 
			{
				bsp_beep_ON();
				g_CurveSpeedControl -= 600;
				temp += 50;
				Car.HorizontalAE += temp;
			}
		}break;
		case RIGHT_ISLAND:
		{
				bsp_beep_ON();
				g_CurveSpeedControl += 600;
				temp -= 50;
				Car.HorizontalAE += temp;
		}break;
	}
	
	Gyro_Z = Car.MPU.Gryoz;
	
	Error = Car.HorizontalAE;
	
	/*  ƫ��΢��  */
	ErrorDiff = Error - LastError;
	
//	Car_GetAngle();
	/*  ��ƫ���Լ�ƫ���΢�ּ���PID  */
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	/*  ����ģ��PID�������ֵ�и���,������Ҫ�����ж�  */
	Kp = (Car.DirFuzzy.KP < 0) ? (-Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (-Car.DirFuzzy.KD) : Car.DirFuzzy.KD;
			
	KdOutLast = KdOutNow;		/*  ���ڲ��ò���ȫ΢��,������Ҫ������һʱ�̵�΢��  */
	KpOut = Error * Kp;		/*  ת�򻷵ı���  */
	KdOutNow = ErrorDiff * Kd;				/*  ת�򻷵�΢��  */
	KdGryozOut = g_GryoZ_Kd * (Gyro_Z - MPU_GRYOZ_ZERO);		/*  ���������ǵĽ��ٶȽ��в���,����ת��  */
	
	/*  �����ϴε�PWM  */
	g_DirciotnControlOutOld = g_DirectionControlOutNew; 
	
	/*  ����PWM,���ò���ȫ΢��PID  */
	g_DirectionControlOutNew = k*KdOutLast + (1 - k)*KdOutNow + KpOut + KdGryozOut;
	
	/*  ��ƫ��С��һ����Χʱ�ر����,���Լ���ϵͳ��  */
	if(Car.HorizontalAE < 4 && Car.HorizontalAE > -4) g_DirectionControlOutNew = 0;

	/*  �����ϸ�ʱ�̵����  */
	LastError = Error;
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
	float DirectionOutput = 0;
	
	DirectionOutput = g_DirectionControlOutNew - g_DirciotnControlOutOld;
	g_DirectionControlOut = (DirectionOutput * (g_DirectionControlPeriod + 1) / DIRCTION_CONTROL_PERIOD + g_DirciotnControlOutOld);
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
	LeftPwm = (int16_t)(g_SpeedControlOut + g_DirectionControlOut );//+ g_SpeedControlBangBang + g_CurveSpeedControl);
	RightPwm = (int16_t)(g_SpeedControlOut - g_DirectionControlOut );//+ g_SpeedControlBangBang - g_CurveSpeedControl);	
		
	/*  �޷�  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
		
	/*  �����߼�������ֵ����2ʱ,˵���Ѿ�������ܵ�,ͣ��  */
	if(g_LoseLineCounter > 2)
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
	static uint16_t CarControlCounter = 0;	
	static uint16_t SpeedControlCounter = 0;
	static uint16_t DirectionControlCounter = 0;
	/*  ���Ƽ�����  */
	CarControlCounter++;
	
	/*  �ٶȿ������  */
	g_SpeedControlPeriod++;
	Car_SpeedControlOutput();
	
	/*  ����������  */
	g_DirectionControlPeriod++;
	Car_DirectionControlOutput();
	
	/*  ����״̬��  */
	switch(CarControlCounter)
	{
		/*  ÿ5ms��ȡһ�α�����  */
		case 1:
		{
			bsp_encoder_ReadCounter();
		}break;
		
		/*  �ٶȿ���  */
		case 2:
		{
			SpeedControlCounter++;
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5)
			{
				SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
				Car_SpeedControl();				/*  ����ʱ��68us  */
			}
		}break;/*  end of case 2  */
		
		
		/*  ÿ5ms����һ�δ��������ݴ���  */
		case 3:
		{
			bsp_sensor_DataProcess();		/*  ����ʱ��64us  */
			Car_RoadDetect();
		}break;
		
		/*  �������  */
		case 4:
		{
			DirectionControlCounter++;
			if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
			{
				DirectionControlCounter = 0;
				g_DirectionControlPeriod = 0;
				Car_DirectionControl();		/*  ����ʱ��600us  */
			}
		}break;
		
		/*  ÿ5ms����һ�ε�����  */
		case 5:
		{
			Car_MotorOutput();
			CarControlCounter=0;
		}break;
		default:		/*  ��������ܵ�������,˵�����������,ͣ������  */
		{
			CarControlCounter = 0;
			DirectionControlCounter = 0;
			SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirectionControlPeriod = 0;
			DRV_DISABLE();				/*  �رյ������  */
			while(1);							
		}break;
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

