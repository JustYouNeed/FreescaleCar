# include "bsp.h"
# include "app.h"
# include "app_debug.h"
# include "app_pid.h"
# include "FreescaleCar.h"


/*  ����ʱ����������ʱ��  

app_debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/

int main(void)
{	
	/*  ���ػ�����ʼ��  */
	bsp_Config();
	
	/*  С��������ʼ��  */
	Car_ParaInit();
	
	/*  �����ҪУ׼������  */
//	if(bsp_switch_GetValue() == 0xff)
//		bsp_sensor_Calibration();
	
	/*  ����һ�������ʱ��,����50ms,��������λ�����泵����Ϣ  */
	bsp_tim_CreateSoftTimer(0, 50, app_debug_SensorDataReport, TIMER_MODE_AUTO);
	
	/*  ����һ�������ʱ��,����20ms,���ڰ���ɨ��  */
	bsp_tim_CreateSoftTimer(1, 20, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  ����һ�������ʱ��,����500ms, ������ʾ������Ϣ  */
	bsp_tim_CreateSoftTimer(3, 500, app_debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  ����һ��Ӳ����ʱ��,����3ms,����С�������Կ���  */
	bsp_tim_CreateHardTimer(1, 3, Car_Control);
	
	while(1);
}
