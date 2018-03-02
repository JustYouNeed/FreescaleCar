# include "bsp.h"
# include "app.h"
# include "app_debug.h"
# include "app_pid.h"
# include "FreescaleCar.h"


/*  各定时器任务运行时长  

app_debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/


uint32_t EncoderCounter = 0;

void test_LedTest(void)
{
	bsp_led_Toggle(1);
}

int main(void)
{	
	Car_ParaInit();
	bsp_Config();
	
//	if(bsp_switch_GetValue() == 0xff)
//		bsp_sensor_Calibration();
	
	bsp_tim_CreateSoftTimer(1, 20, bsp_key_Scan, TIMER_MODE_AUTO);
	bsp_tim_CreateSoftTimer(2, 333, test_LedTest, TIMER_MODE_AUTO);
	bsp_tim_CreateSoftTimer(0, 50, app_debug_SensorDataReport, TIMER_MODE_AUTO);
	bsp_tim_CreateSoftTimer(3, 500, app_debug_ShowPara, TIMER_MODE_AUTO);
	
	bsp_tim_CreateHardTimer(0, 10, bsp_encoder_SpeedCalc);
	bsp_tim_CreateHardTimer(1, 20, Car_Control);
	
	while(1);
}
