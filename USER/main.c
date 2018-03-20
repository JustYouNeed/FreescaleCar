# include "app_debug.h"
# include "FreescaleCar.h"
<<<<<<< HEAD
# include "bsp_sensor.h"
# include "app_debug.h"

=======
# include "button.h"
# include "gui_menu.h"
# include "messagebox.h"
# include "gui_setting.h"
#include "scrollbar.h"
#include "lcmdrv.h"
# include "menu.h"
# include "display.h"
# include "pid_set.h"
>>>>>>> Mr-He
/*  各定时器任务运行时长  

debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/

extern uint8_t setkp;


int main(void)
{	
	bsp_Config();
	
//	Car_ParaInit();

//	/*  选择是否需要校准传感器  */
//	if(bsp_switch_GetValue() == 14)
//		bsp_sensor_Calibration();

	displayInit();
//	test_Button();

	/*  注册软件定时器任务,传感器数据上传任务,周期50ms  */
//	bsp_tim_CreateSoftTimer(0, 51, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  按键扫描任务,周期20ms  */
	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED状态指示任务,周期500ms  */
//	bsp_tim_CreateSoftTimer(0, 200, Car_Running, TIMER_MODE_AUTO);
	
	/*  OLED参数显示任务,周期500ms  */
//	bsp_tim_CreateSoftTimer(0, 200, Menu_Run, TIMER_MODE_AUTO);
	
	/*  编码器速度计算,周期10ms  */
//	bsp_tim_CreateSoftTimer(4, 11, bsp_encoder_SpeedCalc, TIMER_MODE_AUTO);
	
	/*  硬件定时器任务,车子控制任务,周期20ms  */
<<<<<<< HEAD

=======
<<<<<<< HEAD
<<<<<<< HEAD
//<<<<<<< HEAD
=======
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 270b57686bfc76f2a7a146d06ec08edb3e0535f5
>>>>>>> 6222d79d6b85467c792fc47192493923a283fc31
	bsp_tim_CreateHardTimer(1, 6, Car_Control);

	bsp_motor_SetPwm(-600, -600);
	bsp_tim_CreateHardTimer(1, 8, Car_Control);
	DRV_ENABLE();
=======
//	bsp_tim_CreateHardTimer(1, 1, Car_Control);
//	DRV_ENABLE();
>>>>>>> Mr-He
	
//	bsp_motor_SetPwm(200, 160);
//	bsp_key_ClearFIFO();
	while(1)
	{
//		key = bsp_key_GetKey();

//		if(key == KEY_DOWN_PRESS)
//		{
//			bsp_led_Toggle(1);
//			if(MenuCount<4)MenuCount++;
//		}
//		else if(key == KEY_UP_PRESS)
//		{
//			bsp_led_Toggle(2);
//			if(MenuCount>0)MenuCount--;
//		}
//		else if(key == KEY_OK_PRESS && MainMenu[MenuCount].Function)
//		{
//			MainMenu[MenuCount].Function();
//		}

		Menu_Run();
		bsp_tim_DelayMs(50);
		if(setkp==1)
			SpeedKp_Set();
//		oled_clear();
	}
}

