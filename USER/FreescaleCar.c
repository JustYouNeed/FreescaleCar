/**
  *******************************************************************************************************
  * File Name: FreescaleCar.c
  * Author: Vector
  * Version: V2.3.0
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
	*					 3.���ӵڶ���ˮƽ����Լ�ǰ���м���,����ǰ�����ŵĲ�Ⱥ͵Ĳ�ֵ���ж�������
	*          4.����Բ��ʶ����,����ǰ���м���,����׼ȷ�ʽϸ�,���޻��������жϡ�����������
	*					 5.����MPU����,����Z����ٶȸ���ת��
	*
	*		8.Author: Vector
	*			Date: 2018-4-23
	*			Mod: ת��Ϊ�ֶ�PID��Ϊģ��PID,Ч���Ϻ�
	*
	*		9.Author: Vector
	*			Date: 2018-4-28
	*			Mod: 1.������������ٶȻ���Ϊһ��,����λ��ʽPID��Ϊ����ʽPID
	*					 2.�ٶȸ�Ϊ������ƽ��ת��
	*
	*		10.Author: Vector
	*			 Date: 2018-4-29
	*			 Mod: ��д�ٶȻ�,�����ٶȻ����ܿ���,�ٶȻ�������
	*
	*		11.Author: Vector
	*			 Date: 2018-5-20
	*			 Mod: 1.��������δ֪����,���ӻ�ͻȻͣ��,��ԭ����,�ָ�Ϊ4.19�ĳ���
	*					  2.����Բ��ʶ��ʽ,Բ��ʶ���׼ȷ�ʴﵽ90%����,Ч���Ϻ�	*						
	*
  *******************************************************************************************************
  */	
	
	
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "FreescaleCar.h"

/*  �ٶȿ�������*/
# define SPEED_CONTROL_PERIOD	  50	

/*  �����������,��λms  */
# define DIRCTION_CONTROL_PERIOD	5

/*  ���ӳ��ܵ���ĵ��ֵ  */
# define LOST_LINE_THRESHOLD		10

/*  �ٶ�ת����������,������ɺ��ٶȵ�λΪ ת��  */
# define CAR_SPEED_CONSTANT	(1000.0/SPEED_CONTROL_PERIOD/ENCONDER_LINES)*ENCONDER_TEETH/WHEEL_TEETH

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
static int16_t g_DirectionControlOut = 0;
static int16_t g_DirectionControlPeriod = 0;
static int16_t g_DirectionControlOutNew = 0;
static int16_t g_DirciotnControlOutOld = 0;

static float g_TargetSpeed = 0;
/*  ��Z����ٶ�����ת��Kdϵ��  */
const static float g_GryoZ_Kd = 0.1f;

/*  ���߼�����,�ɸñ�����ͳ��ƫ������һ����Χ�Ĵ���,��ֵԽС,˵��ƫ������ԽС  */
static int16_t g_LossLineCounter = 0;
static float g_SpeedFactor = 0.12;
static float g_IslandOffset = 0.0f;

/*  ���߻���  */
static float g_LeftLostLineInteral = 0.0f;
static float g_RightLostLineInteral = 0.0f;
static float g_LostLineRecoup = 0.0f;

static float g_DirKp = 0;
static float g_DirKi = 0;
static float g_DirKd = 0;

/*  Բ�����״̬��  */
static IslandStatus_EnumTypeDef g_IslandStatus = CHECK_POINT_A;

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
	float VelKp = 0, VelKi = 0, VelKd = 0;
	
	/*  ��ʼ�����ӵĺͲ��  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  ��ʼ�����ӵ�PID����,��Flash�ж�ȡ�������PID����  */
	VelKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	VelKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	VelKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);

	/*  ��ʼ���ٶ�PID,ֻ���ñ�������,�޻���΢��  */
	pid_PIDInit(&Car.VelPID, VelKp, VelKi, VelKd, 0, 0);
	
	
	g_DirKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	g_DirKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	g_DirKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	/*  ��ʼ������ģ��PID����  */
	Car.DirFuzzy.DeltaKdMax = 25;
	Car.DirFuzzy.DeltaKiMax = 0;
	Car.DirFuzzy.DeltaKpMax = 15;
	Car.DirFuzzy.DErrMax = 50;
	Car.DirFuzzy.ErrMax = 160;
	Car.DirFuzzy.KP = 12;
	Car.DirFuzzy.KD = 180;
	Car.DirFuzzy.KPMax = g_DirKp;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	Car.DirFuzzy.KIMax = g_DirKi;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.DirFuzzy.KDMax = g_DirKd;//drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	fuzzy_PIDInit(&Car.DirFuzzy);
	
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
	
	Car.Sensor[SENSOR_V_L].CalibrationMax = 150;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2) * 4, uint16_t);
	Car.Sensor[SENSOR_V_L].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2 +  1 ) * 4, uint16_t);
	
	Car.Sensor[SENSOR_V_R].CalibrationMax = 150;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2) * 4, uint16_t);
	Car.Sensor[SENSOR_V_R].CalibrationMin = 0;//drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i * 2 +  1 ) * 4, uint16_t);
		
	/*  ������Ʋ�����ʼ��  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	
	
	
	/*  С��Ŀ���ٶ�  */
	Car.TargetSpeed = drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 0, float);
	g_TargetSpeed = Car.TargetSpeed;
//	Car.LeftTargetSpeed = 20;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 4, float);
//	Car.RightTargetSpeed = 40;//drv_flash_ReadSector(CAR_PARA_FLASH_ADDR, 8, float);
		
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
	bsp_led_Toggle(LED_BLUE);
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
	float temp[4] = {0};
	uint8_t cnt = 0;
	
	
	/*  ���泵������Ŀ���ٶ�  */
	temp[cnt++] = Car.TargetSpeed;
	
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);	/*  �Ȳ���һ��,��Ȼ�޷�д��  */
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t*)&temp, cnt * 4, 0);
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
	static float SpeedFilter = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(LeftSpeed - Car.LeftTargetSpeed);
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	/*  �ٶȻ�PD����,ʵ����Ki = 0  */
	Velocity = (int16_t)(SpeedFilter * -Car.VelPID.Kp);
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
	static float SpeedFilter = 0;
	int16_t Velocity = 0;
	float SpeedError = 0;
	
	/*  �ٶ�ƫ��  */
	SpeedError = (float)(RightSpeed - Car.RightTargetSpeed);
	
	/*  ��ͨ�˲�,���ٶ�ƽ������  */
	SpeedFilter *= 0.7;
	SpeedFilter += (SpeedError * 0.3);
	
	Velocity = (int16_t)(SpeedFilter * -Car.VelPID.Kp);;
	
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
	if(Car.Sensor[SENSOR_H_L].Average > 50 && Car.Sensor[SENSOR_H_R].Average > 50)
	{
		Car.LeftTargetSpeed = Car.TargetSpeed;
		Car.RightTargetSpeed = Car.TargetSpeed;
	}
	else
	{
		/*  �ɵ�ǰ�������̬�������ҵ��Ŀ���ٶ�  */
		Car.LeftTargetSpeed = Car.TargetSpeed + g_SpeedFactor * Car.HorizontalAE;
		Car.RightTargetSpeed = Car.TargetSpeed - g_SpeedFactor * Car.HorizontalAE;
	}
	
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
static uint16_t resumeCnt = 0;
static float RecoupTemp = 0;
void Car_RoadDetect(void)
{
<<<<<<< HEAD
	static uint32_t FirstTime = 0;
	static float AngleTemp = 0;
<<<<<<< HEAD
=======
	static uint32_t cnt = 0;
	
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
=======
	static float FirstPoint = 0, SecondPoint = 0, EnterIslandAngle = 0;
	static uint32_t FirstPointTime = 0, SecondPointTime = 0, OutIslandTime = 0;
	
	/*  �����ߵĵ��ֵ��С����ֵʱ,˵�������Ѿ��ܳ��ܵ���,����,����һ��������ͣ��  */
	if(Car.Sensor[SENSOR_H_L].Average < LOST_LINE_THRESHOLD && Car.Sensor[SENSOR_H_R].Average < LOST_LINE_THRESHOLD)
		g_LossLineCounter++;
>>>>>>> Mr-He
	
	if(Car.Sensor[SENSOR_H_L].Average < 10) g_LeftLostLineInteral ++;
	if(Car.Sensor[SENSOR_H_L].Average < 10) g_RightLostLineInteral ++;
	
	if(g_LeftLostLineInteral > 10) 
	{
		RecoupTemp = 20;
		bsp_beep_ON(6,2, 10);
//		Car.DirFuzzy.KPMax *= 2.5;
	}
	else if(g_RightLostLineInteral > 10) 
	{
//		Car.DirFuzzy.KPMax *= 2.5;
		bsp_beep_ON(6, 2, 10);
		RecoupTemp = -20;
	}
	
<<<<<<< HEAD

	/*  �м��е�ֵ������ֵ,˵���Ѿ�������Բ����Χ,�����ж�Բ������  */
=======
	
<<<<<<< HEAD
	/*  �ñ�־Ϊ0,˵����û���ҵ���һ����־��  */	
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
	if(Car.Sensor[SENSOR_M].Average > 45 )
	{
		if(g_CurveStatus == 0)		/*  ��ʾ�ǵ�һ�ν���Բ������,��������Ҫ�жϷ���  */
=======
	if(Car.Sensor[SENSOR_H_L].Average > 16 && Car.Sensor[SENSOR_H_L].Average > 16)
	{
//		
		if(g_LostLineRecoup !=0)
>>>>>>> Mr-He
		{
			resumeCnt++;
			if(g_LostLineRecoup > 0)
			{
<<<<<<< HEAD
<<<<<<< HEAD
				Car.NowRoad = LEFT_ISLAND;	
=======
				Car.NowRoad = LEFT_ISLAND;
				
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				g_CurveOffset = 0;		/*  ��ֹ�ϴ�δ���ƫ����  */
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
=======
				g_LostLineRecoup = (resumeCnt + 1) / 10 * RecoupTemp - RecoupTemp;
>>>>>>> Mr-He
			}
			else if(g_LostLineRecoup < 0)
			{
<<<<<<< HEAD
				Car.NowRoad = RIGHT_ISLAND;
				g_CurveOffset = 0;					/*  ��ֹ�ϴ�δ���ƫ����  */
				Car.TargetSpeed = 10;
				g_CurveStatus = 1;
=======
				g_LostLineRecoup = (resumeCnt + 1) / 10 * RecoupTemp + RecoupTemp;
>>>>>>> Mr-He
			}
		}
		else
		{
//			bsp_beep_OFF();
			Car.DirFuzzy.KPMax = g_DirKp;
			resumeCnt = 0;
		}
		g_LeftLostLineInteral = 0;
		g_RightLostLineInteral = 0;
	}
<<<<<<< HEAD
	
	/*  �Ѿ��ҵ���һ����־����,��Ҫ�ҵ��ڶ�����־��,�ڶ�����־��Ϊ�м�ˮƽ��еķ�ֵ  */
	if(g_CurveStatus == 1)	
=======
	else
		g_LostLineRecoup = RecoupTemp;
	/*  Բ��״̬��  */
	switch(g_IslandStatus)
>>>>>>> Mr-He
	{
		case CHECK_POINT_A:
		{
			if(Car.AE > 0) FirstPoint++;
			else if(Car.AE < 0) FirstPoint--;
			
			if(FirstPoint > 3 || FirstPoint <-3)		/*  ��¼��һ����ļ���,��¼���,��ֹ����  */
			{
<<<<<<< HEAD
				g_CurveOffset = -30;
				bsp_led_ON(LED_RED);
<<<<<<< HEAD
//				AngleTemp = Car.MPU.Yaw;
=======
				AngleTemp = Car.MPU.Yaw;
				g_CurveStatus = 2;
				bsp_beep_ON();
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				g_NeedEnterCurve = 1;
=======
				FirstPointTime = bsp_tim_GetRunTime();
				g_IslandStatus = CHECK_POINT_B;
				
				bsp_beep_ON(10, 5, 5);
				if(FirstPoint < 0) bsp_led_ON(LED_RED);			/*  ����Բ�����������ͬ�ĵ�  */
				else if(FirstPoint > 0) bsp_led_ON(LED_BLUE);
>>>>>>> Mr-He
			}
		}break;
		case CHECK_POINT_B:
		{
			/*  �����һ��ʱ����û���ҵ��ڶ�����,��˵��Ϊ����,����ʧ��  */
			if(bsp_tim_GetRunTime() - FirstPointTime > 1000) g_IslandStatus = CLEAR_FLAG;
			
			if(FirstPoint * Car.AE < 0)			/*  ���ҵ���һ�����,����һ�������ֵ��ʱ���˵���ǽ�����ʱ��  */
			{
<<<<<<< HEAD
<<<<<<< HEAD
				g_NeedEnterCurve = 1;
//				AngleTemp = Car.MPU.Yaw;
=======
				g_CurveOffset = 30;
				g_CurveStatus = 2;
				AngleTemp = Car.MPU.Yaw;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
				bsp_led_ON(LED_BLUE);
				bsp_beep_ON();
				g_NeedEnterCurve = 1;
			}
		}
	}
	
<<<<<<< HEAD
//	if(i_abs(Car.Sensor[SENSOR_V_L].Average - Car.Sensor[SENSOR_V_R].Average) < 5 && g_NeedEnterCurve == 1 && Car.Sensor[SENSOR_M].Average < 35)   //˵��û��Բ�� ������б�־��֤��Ӱ����һ��Բ���Ľ���
//	{
//		g_NeedEnterCurve = 0;
//		g_CurveOffset = 0;
//		g_NeedOutCurve = 0;
//		bsp_led_OFF(LED_ALL);
//	}
	if(i_abs(Car.Sensor[SENSOR_V_L].Average - Car.Sensor[SENSOR_V_R].Average) > 10 && g_NeedEnterCurve == 1 && Car.Sensor[SENSOR_M].Average < 35)    //˵����Բ���� ����������
	{
		g_NeedOutCurve = 1;
		g_AlreadyEnterCurve = 1;
		g_CurveOffset = 0;
		Car.TargetSpeed = 15;
	}
	
	if(g_AlreadyEnterCurve == 1 && Car.Sensor[SENSOR_M].Average > 40)    //�Ѿ�����Բ�� ��Բ���ܱ�־����
	{
		g_CurveStatus = 0;
		Car.TargetSpeed = 10;
	}
	
//	if(fabs(Car.MPU.Yaw - AngleTemp) > 40 && g_NeedEnterCurve == 1) 
//	{
//		g_NeedOutCurve = 1;
//		g_NeedEnterCurve = 0;
//		g_CurveOffset = 0;
//	}
//	if(fabs(Car.MPU.Yaw - AngleTemp) > 300 && g_NeedOutCurve == 1) Car.TargetSpeed = 10;
	
	if(Car.Sensor[SENSOR_M].Average < 35 && Car.VecticalAE == 0 && g_CurveStatus == 0)   //(Car.Sensor[SENSOR_H_L].Average  + Car.Sensor[SENSOR_H_R].Average) < 145)
	{
		g_NeedEnterCurve = 0;
		g_AlreadyEnterCurve = 1;
		g_CurveOffset = 0;
		g_NeedOutCurve = 0;
		Car.TargetSpeed = 15;
=======
	if(fabs(Car.MPU.Pitch) > 15 && g_NeedEnterCurve == 1) 
	{
		g_NeedEnterCurve = 0;
		AngleTemp = 0;
		g_CurveOffset = 0;
		g_CurveStatus = 0;
		bsp_led_OFF(LED_ALL);
		Car.TargetSpeed = 15;
		bsp_beep_OFF();
	}
	
	if(fabs(Car.MPU.Yaw - AngleTemp) > 40 && g_NeedEnterCurve ==1)
	{
		g_CurveOffset = 0;
		g_NeedOutCurve = 1;
		g_NeedEnterCurve = 0;
		Car.TargetSpeed = 15;
	}
	
	if(g_NeedOutCurve == 1 && fabs(Car.MPU.Yaw - AngleTemp) > 200)
	{
		g_NeedOutCurve = 2;
		Car.TargetSpeed = 10;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
		bsp_led_OFF(LED_ALL);
	}
	
	if(Car.Sensor[SENSOR_M].Average < 40 && Car.Sensor[SENSOR_H_L].Average < 100 && Car.Sensor[SENSOR_H_R].Average < 100)
	{
		if(g_NeedOutCurve == 2)
=======
				g_IslandOffset = (FirstPoint < 0) ? (3) : (-3);		/*  ����Բ��������������  */
				EnterIslandAngle = Car.MPU.Yaw;				/*  ��¼��ǰ�Ƕ�,�����ж��Ѿ�����Բ��  */
				SecondPoint = Car.AE;									/*  ��¼�ڶ�����ļ���,�����жϳ���  */
				SecondPointTime = bsp_tim_GetRunTime();
				g_IslandStatus = WAIT_ENTER;			/*  �ȴ�����  */
			}
		}break;
		case WAIT_ENTER:
>>>>>>> Mr-He
		{
			if(bsp_tim_GetRunTime() - SecondPointTime > 2000) g_IslandStatus = CLEAR_FLAG;
			if(f_abs(Car.MPU.Yaw - EnterIslandAngle) > 40) 
			{
				g_IslandOffset = 0;
				OutIslandTime = bsp_tim_GetRunTime();
				g_IslandStatus = WAIT_OUT_A;		/*  �ȴ�������һ��  */
			}
		}break;
		case WAIT_OUT_A:		
		{
			if(bsp_tim_GetRunTime() - OutIslandTime > 5 * 1000) g_IslandStatus = CLEAR_FLAG;		/*  ������ʱ  */
			if(SecondPoint * Car.AE > 0) g_IslandStatus = WAIT_OUT_B; /*  �����ķ弫�����뻷ʱ�෴����¼��ʱΪ������־  */
		}break;
		case WAIT_OUT_B:		/*  �ҵ������ĵڶ�����־,�м���ֵ������ֵ���ߵڶ�����ֵΪ0  */
		{
			if(Car.AE == 0 || Car.Sensor[SENSOR_M].Average > 90) 	g_IslandStatus = CLEAR_FLAG;
		}break;
		case CLEAR_FLAG:		/*  �����ɹ�����ʧ�ܺ󶼻���ת�����״̬  */
		{
			bsp_led_OFF(LED_ALL);
			FirstPoint = 0;
			SecondPoint = 0;
			EnterIslandAngle = 0;
			FirstPointTime = 0;
			g_IslandOffset = 0;
			g_IslandStatus = CHECK_POINT_A;
		}break;
		default:g_IslandStatus = CLEAR_FLAG;break;
	}
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
<<<<<<< HEAD
	float KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
	static float temp = 0;
	
<<<<<<< HEAD
	switch(Car.NowRoad)
	{
		case STRAIGHT:temp = 0; g_CurveOffset = 0; g_NeedEnterCurve = 0; g_NeedOutCurve = 0; g_CurveSpeedControl = 0; break;
		case LEFT_CURVE:break;
		case RIGHT_CURVE:break;
		case LEFT_ISLAND:
		{
			if(g_NeedEnterCurve == 1) 	/*  ֻ������Ҫ����Բ����û�н���Բ��ʱ��Ҫ����  */
			{
//				bsp_beep_ON();
				g_CurveOffset = -30;
			}
		}break;
		case RIGHT_ISLAND:
		{			
			if(g_NeedEnterCurve == 1) 
			{
//				bsp_beep_ON();
				g_CurveOffset = 30;
			}
		}break;
	}
	
	Gyro_Z = Car.MPU.Gyroz - MPU_GYROZ_ZERO;
=======
	Gyro_Z = Car.MPU.Gyroz;
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f
	
	Error = Car.HorizontalAE + g_CurveOffset;
	
=======
	int16_t KpOut = 0, KdOutNow = 0, KdGryozOut = 0;
		
	Error = Car.HorizontalAE + g_IslandOffset + g_LostLineRecoup;
//	Error = g_LostLineRecoup;
>>>>>>> Mr-He
	/*  ƫ��΢��  */
	ErrorDiff = Error - LastError;
	
	/*  ��ƫ���Լ�ƫ���΢�ּ���PID  */
	fuzzy_PIDClac(&Car.DirFuzzy, Error, ErrorDiff);
	
	/*  ����ģ��PID�������ֵ�и���,������Ҫ�����ж�  */
	Kp = (Car.DirFuzzy.KP < 0) ? (-Car.DirFuzzy.KP) : Car.DirFuzzy.KP;
	Kd = (Car.DirFuzzy.KD < 0) ? (-Car.DirFuzzy.KD) : Car.DirFuzzy.KD;
		
	KdOutLast = KdOutNow;		/*  ���ڲ��ò���ȫ΢��,������Ҫ������һʱ�̵�΢��  */
	KpOut = Error * Kp;		/*  ת�򻷵ı���  */
	KdOutNow = ErrorDiff * Kd;				/*  ת�򻷵�΢��  */
	Gyro_Z = Car.MPU.Gyroz - MPU_GYROZ_ZERO;
	KdGryozOut = g_GryoZ_Kd * Gyro_Z;		/*  ���������ǵĽ��ٶȽ��в���,����ת��  */
		
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
	
	/*  �޷�  */
	if(LeftPwm > Car.MaxPWM) LeftPwm = Car.MaxPWM;
	else if(LeftPwm < -Car.MaxPWM) LeftPwm = -Car.MaxPWM;
	
	if(RightPwm > Car.MaxPWM) RightPwm = Car.MaxPWM;
	else if(RightPwm < -Car.MaxPWM) RightPwm = -Car.MaxPWM;
	
	Car.Motor.LeftPwm = LeftPwm;
	Car.Motor.RightPwm = RightPwm;
	
	
	/*  ��������  */
	if(g_LossLineCounter > 2)		/*  ����ܵ�ͣ��  */
	{
		bsp_motor_SetPwm(0,0);
		bsp_beep_Pause();
	}
	else
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
<<<<<<< HEAD
<<<<<<< HEAD
		/*  ÿ5ms��ȡһ�α�����  */
		case 1:
		{
			SpeedControlCounter++;
			if(SpeedControlCounter >= SPEED_CONTROL_PERIOD/5 && g_SpeedControlON == 1)
			{
				SpeedControlCounter = 0;
				g_SpeedControlPeriod = 0;
//				Car_SpeedControl();				/*  ����ʱ��68us  */
			}
		}break;
		
		/*  �ٶȿ���  */
		case 2:
		{
=======
		SpeedControlCounter = 0;
		g_SpeedControlPeriod = 0;
		Car_SpeedControl();				/*  ����ʱ��68us  */
	}
>>>>>>> 421d93c508597dfbf2aec154fc71db21e673828f

	bsp_sensor_DataProcess();		/*  ����ʱ��64us  */
	Car_RoadDetect();



	DirectionControlCounter++;
	if(DirectionControlCounter >= DIRCTION_CONTROL_PERIOD/5)
	{
		DirectionControlCounter = 0;
		g_DirectionControlPeriod = 0;
		Car_DirectionControl();		/*  ����ʱ��600us  */
=======
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
			Car_RoadDetect();
			g_DirectionControlPeriod = 0;
			Car_DirectionControl();
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
			g_SpeedControlCounter = 0;
			g_SpeedControlPeriod = 0;
			g_DirectionControlPeriod = 0;
		}break;
>>>>>>> Mr-He
	}
}
	
	
/********************************************  END OF FILE  *******************************************/
	

