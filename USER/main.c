# include "bsp.h"
# include "app.h"
# include "app_debug.h"
# include "app_pid.h"
# include "FreescaleCar.h"


/*  ����ʱ����������ʱ��  

debug_ShowPara: 10ms
bsp_encoder_SpeedCalc: 4us
bsp_key_Scan: 6.4us

*/
void ftm_pwm_init(uint8_t ftmn, uint8_t ch, uint32_t freq, uint32_t duty)
{
    uint32_t clk_hz ;
    uint16_t mod;
    uint8_t  ps;
    uint16_t cv;
    
//    ftm_pwm_mux(ftmn,ch);
    
    // �� CPWMS = 0 �����ض���
    clk_hz = SystemBusClock;
    //clk_hz/(ps*mod) = freq        =>>     clk_hz/freq = (ps*mod)      =>>
    //clk_hz/freq < ((1<<n)*65536)  =>>    (clk_hz/freq)/65536 < (1>>n) =>> ((clk_hz/freq)/65536)>>n < 1
    mod = (clk_hz >> 16 ) / freq ;          // ��ʱ�� mod ����һ��
    ps = 0;
    while((mod >> ps) >= 1)                 // �� (mod >> ps) < 1 ���˳� while ѭ�� ������ PS ����Сֵ
        ps++;
    if(ps>0x07) return ;                    //�������÷�Χ��ֱ�Ӳ�����������������ԭ��һ��������PWMƵ��̫�ͣ���������Ƶ��̫�ߵ���
    
    mod = (clk_hz >> ps) / freq;            // �� MOD ��ֵ
//    period[ftmn] = mod;
    switch(ftmn)                            // ��ֵ CNTIN ��Ϊ0 �������ȣ�CnV - CNTIN ���� CnV ���� �������ˡ�
    {
        // EPWM������ �� MOD - CNTIN + 0x0001 == MOD - 0 + 1
        // �� CnV = (MOD - 0 + 1) * ռ�ձ� = (MOD - 0 + 1) * duty/ FTM_PRECISON
    case 0:
        cv = (duty * (mod - 0 + 1)) / 1000;
        break;

    case 1:
        cv = (duty * (mod - 0 + 1)) / 1000;
        break;

    case 2:
        cv = (duty * (mod - 0 + 1)) / 1000;
        break;
    default:
        break;
    }
    
    FTM2->MOD = mod;                  //����PWM����
    
    FTM2->CONTROLS[ch].CnSC &= ~FTM_CnSC_ELSA_MASK;
    FTM2->CONTROLS[ch].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    
    // set FTM clock to system clock 
   FTM2->SC = ( 0
                       //| FTM_SC_CPWMS_MASK       //0�����ض��� ��1�� ���Ķ��� ��ע���˱�ʾ 0��
                       | FTM_SC_PS(ps)             //��Ƶ���ӣ���Ƶϵ�� = 2^PS
                       | FTM_SC_CLKS(1)            //ʱ��ѡ�� 0��ûѡ��ʱ�ӣ����ã� 1��bus ʱ�ӣ� 2��MCGFFCLK�� 3��EXTCLK�� ��SIM_SOPT4 ѡ������ܽ� FTM_CLKINx��
                       //| FTM_SC_TOIE_MASK        //����ж�ʹ�ܣ�ע���˱�ʾ ��ֹ����жϣ�
                     );

    
    if(2 == ftmn)FTM2->CNTIN = 0;         //���ü�������ֵ��һ��û������;������
    FTM2->CONTROLS[ch].CnV = cv;             //����ռ�ձ�
    FTM2->CNT = 0;                           //��������д����ֵ�������CNTIN��ֵ
    
}


void test_LedTest(void)
{
	bsp_led_Toggle(1);
}

int main(void)
{	
	bsp_Config();
	
		ftm_pwm_init(2, 1, 10000, 10);
	Car_ParaInit();
		bsp_sensor_Config();		/*  ��Ŵ�������ʼ��  */

	/*  ѡ���Ƿ���ҪУ׼������  */
	if(bsp_switch_GetValue() == 14)
		bsp_sensor_Calibration();

	/*  ע�������ʱ������,�����������ϴ�����,����50ms  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  ����ɨ������,����20ms  */
//	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED״ָ̬ʾ����,����500ms  */
	bsp_tim_CreateSoftTimer(2, 453, test_LedTest, TIMER_MODE_AUTO);
	
	/*  OLED������ʾ����,����500ms  */
	bsp_tim_CreateSoftTimer(3, 501, debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  �������ٶȼ���,����10ms  */
//	bsp_tim_CreateSoftTimer(4, 11, bsp_encoder_SpeedCalc, TIMER_MODE_AUTO);
	
	/*  Ӳ����ʱ������,���ӿ�������,����20ms  */
	bsp_tim_CreateHardTimer(1, 15, Car_Control);
	
//	bsp_motor_SetPwm(200, -200);
	while(1);
//	{
//		key = bsp_switch_GetValue();
//		bsp_oled_ShowInteger(0, 0, key, 16);
//		bsp_led_Toggle(0);
//		bsp_tim_DelayMs(50);
//	}
}
