# include "bsp.h"
# include "FreescaleCar.h"
# include "display.h"




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
int main(void)
{	
	bsp_Config();
	
	Car_ParaInit();
	
	displayInit();
	
	/*  ѡ���Ƿ���ҪУ׼������  */
	if(drv_gpio_ReadPin(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();

	/*  ע�������ʱ������,�����������ϴ�����,����50ms  */
	bsp_tim_CreateSoftTimer(0, 51, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  ����ɨ������,����20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED״ָ̬ʾ����,����500ms  */
//	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(2, 10, bsp_beep_Thread, TIMER_MODE_AUTO);
	
<<<<<<< HEAD
	
	bsp_tim_CreateSoftTimer(2, 100, displayTask, TIMER_MODE_AUTO);
	
//	/*  ÿ5ms��ȡһ�ν��ٶ�  */
//	bsp_tim_CreateHardTimer(1,5, bsp_mpu_GetAngle);

	/*  ����С������  */
	bsp_tim_CreateHardTimer(0, 5, Car_Control);

	while(1)
	{
//		bsp_mpu_GetAngle();
=======
	/*  Ӳ����ʱ������,���ӿ�������,����20ms  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
	bsp_tim_CreateHardTimer(0,5, bsp_mpu_GetAngle);
	while(1)
	{
		displayTask();
>>>>>>> Mr-He
		bsp_tim_DelayMs(100);
	}
}
