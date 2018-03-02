# include "bsp.h"

uint8_t TimerTaskRunMutexSignal = 0;

void bsp_Config(void)
{
//	DISABLE_INT();
	drv_flash_Init();
	bsp_tim_SoftConfig();
	bsp_uart_Config();
	bsp_oled_Config();
	bsp_led_Config();
	bsp_key_Config();
	bsp_switch_Config();
	bsp_encoder_Config();
	bsp_sensor_Config();
	bsp_motor_Config();
//	bsp_sensor_Calibration();
	ENABLE_INT();
}

