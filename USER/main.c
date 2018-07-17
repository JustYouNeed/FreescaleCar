# include "bsp.h"
# include "FreescaleCar.h"
# include "display.h"

/*  延时基准100ms*DELAY_TIME  */
# define DELAY_TIME	20
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
	uint8_t DelayStart = 0;
	
	bsp_Config();
	
	Car_ParaInit();
	
	displayInit();
	
	/*  选择是否需要校准传感器  */
	if(drv_gpio_ReadPin(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();

	/*  注册软件定时器任务,传感器数据上传任务,周期50ms  */
	bsp_tim_CreateSoftTimer(0, 51, app_ano_CarDataReport, TIMER_MODE_AUTO);
	
	/*  按键扫描任务,周期20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED状态指示任务,周期500ms  */
//	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
//	bsp_tim_CreateSoftTimer(2, 10, bsp_beep_Thread, TIMER_MODE_AUTO);
	
//	while(DelayStart++ < DELAY_TIME)
//	{
//		bsp_led_Toggle(LED_ALL);
//		bsp_tim_DelayMs(100);
//	}
	/*  硬件定时器任务,车子控制任务,周期20ms  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);
	bsp_tim_CreateHardTimer(0,5, bsp_mpu_GetAngle);
	
//	bsp_motor_SetPwm(-500,-200);
//	DRV_ENABLE();
	while(1)
	{
		displayTask();
		bsp_tim_DelayMs(100);
		
	}
}
