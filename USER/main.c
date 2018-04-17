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
	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
	/*  Ӳ����ʱ������,���ӿ�������,����20ms  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
	bsp_tim_CreateHardTimer(0,5, ReadGryo);
	DRV_ENABLE();
//	bsp_motor_SetPwm(400,400);
	setShow_ui(MAIN_UI);
	while(1)
	{
		displayTask();
		bsp_tim_DelayMs(100);
	}
}
