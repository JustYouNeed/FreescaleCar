/**
  *******************************************************************************************************
  * File Name: main.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ��׼C������ں���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
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
# include "display.h"

/*
*********************************************************************************************************
*                               main           
*
* Description: ��׼C������ں���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int main(void)
{	
	bsp_Config();
	
	Car_ParaInit();
	
	displayInit();
	
	
	/*  ѡ���Ƿ���ҪУ׼������  */
	if(drv_gpio_ReadPin(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();

	/*  ע�������ʱ������,�����������ϴ�����,����50ms,����ִ��ʱ��43.2us  */
	bsp_tim_CreateSoftTimer(0, 51, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  ����ɨ������,����20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED״ָ̬ʾ����,����500ms  */
//	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
	/*  ÿ������һ�µ�ص�ѹ  */
//	bsp_tim_CreateSoftTimer(3, 1000, Car_GetVoltage, TIMER_MODE_AUTO);
	
	/*  ÿ5ms��ȡһ�ν��ٶ�  */
	bsp_tim_CreateHardTimer(1,5, bsp_mpu_GetAngle);

	/*  ����С������  */
	bsp_tim_CreateHardTimer(0, 5, Car_Control);

	while(1)
	{
		displayTask();				/*  ����ִ��ʱ��36ms  */
		bsp_tim_DelayMs(100);
	}
}
