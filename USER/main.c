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
	
	
	/*  选择是否需要校准传感器  */
	if(drv_gpio_ReadPin(KEY_OK_PIN) == 0)
		bsp_sensor_Calibration();

	/*  注册软件定时器任务,传感器数据上传任务,周期50ms,任务执行时长43.2us  */
	bsp_tim_CreateSoftTimer(0, 51, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  按键扫描任务,周期20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED状态指示任务,周期500ms  */
	bsp_tim_CreateSoftTimer(2, 200, Car_Running, TIMER_MODE_AUTO);
	
	bsp_tim_CreateSoftTimer(3, 5000, Car_GetVoltage, TIMER_MODE_AUTO);
	
	/*  开启小车控制  */
	Car_ControlStart();	
	
	bsp_tim_CreateHardTimer(0,5, ReadGryo);

//	bsp_motor_SetPwm(400,400);
	setShow_ui(MAIN_UI);
	while(1)
	{
		displayTask();				/*  任务执行时长36ms  */
		bsp_tim_DelayMs(100);
	}
}
