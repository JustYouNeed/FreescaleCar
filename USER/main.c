# include "app_debug.h"
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
	if(bsp_switch_GetValue() == 14)
		bsp_sensor_Calibration();

	/*  ע�������ʱ������,�����������ϴ�����,����50ms  */
	bsp_tim_CreateSoftTimer(0, 51, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  ����ɨ������,����20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED״ָ̬ʾ����,����500ms  */
	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
	/*  Ӳ����ʱ������,���ӿ�������,����20ms  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
	DRV_ENABLE();
	
	setShow_ui(MAIN_UI);
	while(1)
	{
		displayTask();
//		bsp_mpu_ReadGyro(&Car.MPU.Gryox, &Car.MPU.Gryoy, &Car.MPU.Gryoz);
//		if(Car.MPU.Gryoz < 0)
//		{
//			oled_showChar(0,0,'-',6,12,1);
//			oled_showNum(3,0, -Car.MPU.Gryoz, 5,6,12);
//		}else
//			oled_showNum(0,0, Car.MPU.Gryoz, 5,6,12);
//		oled_refreshGram();
		bsp_tim_DelayMs(100);
	}
}
