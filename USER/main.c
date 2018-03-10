# include "bsp.h"
# include "app.h"
# include "app_debug.h"
# include "app_pid.h"
# include "FreescaleCar.h"


/*  各定时器任务运行时长  

debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/



void test_LedTest(void)
{
	bsp_led_Toggle(0);
}

int main(void)
{	
	bsp_Config();
	
	Car_ParaInit();

	/*  选择是否需要校准传感器  */
	if(bsp_switch_GetValue() == 14)
		bsp_sensor_Calibration();

	/*  注册软件定时器任务,传感器数据上传任务,周期50ms  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  按键扫描任务,周期20ms  */
//	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED状态指示任务,周期500ms  */
	bsp_tim_CreateSoftTimer(2, 453, Car_Running, TIMER_MODE_AUTO);
	
	/*  OLED参数显示任务,周期500ms  */
	bsp_tim_CreateSoftTimer(3, 501, debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  编码器速度计算,周期10ms  */
	bsp_tim_CreateSoftTimer(4, 11, bsp_encoder_SpeedCalc, TIMER_MODE_AUTO);
	
	/*  硬件定时器任务,车子控制任务,周期20ms  */
	bsp_tim_CreateHardTimer(1, 6, Car_Control);
	
	bsp_motor_SetPwm(-300, -300);
	DRV_ENABLE();
	while(1);
//	{
//		key = bsp_switch_GetValue();
//		bsp_oled_ShowInteger(0, 0, key, 16);
//		bsp_led_Toggle(0);
//		bsp_tim_DelayMs(50);
//	}
}
