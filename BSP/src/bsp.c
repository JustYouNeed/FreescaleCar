# include "bsp.h"

uint8_t TimerTaskRunMutexSignal = 0;

void bsp_Config(void)
{
	drv_flash_Init();				/*  оƬ�ڲ�Flash��ʼ��  */
	bsp_tim_SoftConfig();		/*  �����ʱ����ʼ��,���õδ�ʱ��,��ʱ����1ms  */
	bsp_uart_Config();			/*  ���Դ��ڳ�ʼ��,������115200  */
	bsp_oled_Config();			/*  OLED��Ļ��ʼ��  */
	bsp_led_Config();				/*  LED�Ƴ�ʼ��  */
	bsp_key_Config();				/*  ����������ʼ��  */
	bsp_switch_Config();		/*  ���뿪�س�ʼ��  */
	bsp_mpu_Config();
	bsp_motor_Config();			/*  ������Ƴ�ʼ��  */
	bsp_encoder_Config();		/*  �����������ʼ��  */
	bsp_sensor_Config();		/*  ��Ŵ�������ʼ��  */
	bsp_beep_Config();
//	bsp_bat_Config();
	ENABLE_INT();						/*  �����ж�  */
}



float f_abs(float num)
{
	return (num < 0)?(-num):num;
}

