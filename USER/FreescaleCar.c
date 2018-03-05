/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
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
void Car_ParaInit(void)
{
	uint8_t i = 0;
	
	/*  ��ʼ�����ӵĸ����  */
	Car.HorizontalAE = 0;
	Car.VecticalAE = 0;
		
	/*  ��ʼ�����ӵ�PID����,��Flash�ж�ȡ�������PID����  */
	Car.PID.Kp_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);	/*  ֱ��PID  */
	Car.PID.Ki_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Kd_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Kp_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);		/*  ���PID  */
	Car.PID.Ki_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Kd_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
	
	/*  ��ʼ�����ӵĴ���������,��Flash�ж�ȡ�궨ֵ  */
	for(i = 0; i < SENSOR_COUNT; i ++)
	{
		Car.Sensor[i].FIFO[0] = 0;
		Car.Sensor[i].Read = 0;
		Car.Sensor[i].Write = 0;
		Car.Sensor[i].Average = 0;
		Car.Sensor[i].NormalizedValue = 0;
		Car.Sensor[i].CalibrationMax = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, i * 2, uint16_t);
		Car.Sensor[i].CalibrationMin = drv_flash_ReadSector(SENSOR_PARA_FLASH_ADDR, (i + SENSOR_COUNT - 1) * 2, uint16_t);
	}
	
	/*  ������Ʋ�����ʼ��  */
	Car.Motor.PWM_Frequency = 10;	/*  ���PWMƵ��Ϊ10KHz  */
	Car.Motor.LeftPwm = 0;
	Car.Motor.RightPwm = 0;
	Car.Motor.LeftEncoder = 0;
	Car.Motor.RightEncoder = 0;
	Car.Motor.LeftSpeed = 0;
	Car.Motor.RightSpeed = 0;
	
	Car.BaseSpeed = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
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
void Car_ParaStroe(void)
{
	drv_flash_EraseSector(CAR_PARA_FLASH_ADDR);
	drv_flash_WriteSector(CAR_PARA_FLASH_ADDR, (const uint8_t *)&Car.BaseSpeed, 2, 0);
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
void Car_PIDCalc(void)
{
	int16_t pwm = 0;
	static float LastError;
	
	Car.PID.Error = Car.HorizontalAE - LastError;		/*  ���㵱ǰ΢����  */
	
	/*  ����PWM  */
	pwm = (int16_t)((Car.HorizontalAE * 10 *  Car.PID.Kp_Straight) + (Car.PID.Sum * Car.PID.Ki_Straight) + (Car.PID.Error * Car.PID.Kd_Straight));
	
	/*  �����ϸ�ʱ�̵����  */
	LastError = (float)(Car.HorizontalAE * 10);
	
	/*  �����������PID������ٶ����  */
	Car.Motor.LeftPwm = Car.BaseSpeed - pwm;
	Car.Motor.RightPwm = Car.BaseSpeed + pwm;
	
	/*  �����޷�  */
	if(Car.Motor.LeftPwm > 600) Car.Motor.LeftPwm = 600;
	else if(Car.Motor.LeftPwm < -600) Car.Motor.LeftPwm = -600;
	
	if(Car.Motor.RightPwm > 600) Car.Motor.RightPwm = 600;
	else if(Car.Motor.RightPwm < -600) Car.Motor.RightPwm = -600;
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
//	bsp_led_Toggle(2);
	bsp_sensor_DataProcess();
	Car_PIDCalc();
	
	
//	if(Car.Sensor[SENSOR_ID_1].Average < 20 || Car.Sensor[SENSOR_ID_4].Average < 20)
//		bsp_motor_Stop();
//	else
		bsp_motor_SetPwm(Car.Motor.LeftPwm, Car.Motor.RightPwm);
}
	
	
/********************************************  END OF FILE  *******************************************/
	

