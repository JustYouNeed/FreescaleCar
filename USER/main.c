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

int main(void)
{	
	/*  主控基本初始化  */
	bsp_Config();
	
	/*  小车参数初始化  */
	Car_ParaInit();
	
	/*  如果需要校准传感器  */
//	if(bsp_switch_GetValue() == 0xff)
//		bsp_sensor_Calibration();
	
	/*  创建一个软件定时器,周期50ms,用于向上位机报告车子信息  */
	bsp_tim_CreateSoftTimer(0, 50, app_debug_SensorDataReport, TIMER_MODE_AUTO);
	
	/*  创建一个软件定时器,周期20ms,用于按键扫描  */
	bsp_tim_CreateSoftTimer(1, 20, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  创建一个软件定时器,周期500ms, 用于显示车子信息  */
	bsp_tim_CreateSoftTimer(3, 500, app_debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  创建一个硬件定时器,周期3ms,用于小车周期性控制  */
	bsp_tim_CreateHardTimer(1, 3, Car_Control);
	
	while(1);
}
