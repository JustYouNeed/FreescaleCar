# include "app_debug.h"
# include "FreescaleCar.h"
# include "button.h"
# include "gui_menu.h"
# include "messagebox.h"
# include "gui_setting.h"
#include "scrollbar.h"
#include "lcmdrv.h"
# include "menu.h"
# include "display.h"
# include "pid_set.h"
# include "font.h"
/*  ����ʱ����������ʱ��  

debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/

extern uint8_t setkp;


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
		
	/*  �������ٶȼ���,����10ms  */
//	bsp_tim_CreateSoftTimer(4, 11, bsp_encoder_SpeedCalc, TIMER_MODE_AUTO);
	
	/*  Ӳ����ʱ������,���ӿ�������,����20ms  */
	bsp_tim_CreateHardTimer(1, 1, Car_Control);

	setShow_ui(MAIN_UI);

	while(1)
	{
		displayTask();
//		oled_refreshGram();
//		if(mpu_dmp_get_data(&Car.MPU.Pitch, &Car.MPU.Roll, &Car.MPU.Yaw) == 0)
//		{
//			bsp_led_Toggle(1);
//			bsp_mpu_ReadAcc(&Car.MPU.Accx, &Car.MPU.Accy, &Car.MPU.Accz);
//			bsp_mpu_ReadGyro(&Car.MPU.Gryox, &Car.MPU.Gryoy, &Car.MPU.Gryoz);
//		}
		bsp_tim_DelayMs(100);
	}
}
