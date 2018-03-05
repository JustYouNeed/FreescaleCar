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
void ftm_pwm_init(uint8_t ftmn, uint8_t ch, uint32_t freq, uint32_t duty)
{
    uint32_t clk_hz ;
    uint16_t mod;
    uint8_t  ps;
    uint16_t cv;
    
//    ftm_pwm_mux(ftmn,ch);
    
    // 以 CPWMS = 0 ，边沿对齐
    clk_hz = SystemBusClock;
    //clk_hz/(ps*mod) = freq        =>>     clk_hz/freq = (ps*mod)      =>>
    //clk_hz/freq < ((1<<n)*65536)  =>>    (clk_hz/freq)/65536 < (1>>n) =>> ((clk_hz/freq)/65536)>>n < 1
    mod = (clk_hz >> 16 ) / freq ;          // 临时用 mod 缓存一下
    ps = 0;
    while((mod >> ps) >= 1)                 // 等 (mod >> ps) < 1 才退出 while 循环 ，即求 PS 的最小值
        ps++;
    if(ps>0x07) return ;                    //超出设置范围，直接不设置跳出本函数，原因一般是由于PWM频率太低，或者总线频率太高导致
    
    mod = (clk_hz >> ps) / freq;            // 求 MOD 的值
//    period[ftmn] = mod;
    switch(ftmn)                            // 初值 CNTIN 设为0 ，脉冲宽度：CnV - CNTIN ，即 CnV 就是 脉冲宽度了。
    {
        // EPWM的周期 ： MOD - CNTIN + 0x0001 == MOD - 0 + 1
        // 则 CnV = (MOD - 0 + 1) * 占空比 = (MOD - 0 + 1) * duty/ FTM_PRECISON
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
    
    FTM2->MOD = mod;                  //设置PWM周期
    
    FTM2->CONTROLS[ch].CnSC &= ~FTM_CnSC_ELSA_MASK;
    FTM2->CONTROLS[ch].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    
    // set FTM clock to system clock 
   FTM2->SC = ( 0
                       //| FTM_SC_CPWMS_MASK       //0：边沿对齐 ，1： 中心对齐 （注释了表示 0）
                       | FTM_SC_PS(ps)             //分频因子，分频系数 = 2^PS
                       | FTM_SC_CLKS(1)            //时钟选择， 0：没选择时钟，禁用； 1：bus 时钟； 2：MCGFFCLK； 3：EXTCLK（ 由SIM_SOPT4 选择输入管脚 FTM_CLKINx）
                       //| FTM_SC_TOIE_MASK        //溢出中断使能（注释了表示 禁止溢出中断）
                     );

    
    if(2 == ftmn)FTM2->CNTIN = 0;         //设置计数器初值，一般没特殊用途就清零
    FTM2->CONTROLS[ch].CnV = cv;             //设置占空比
    FTM2->CNT = 0;                           //计数器，写任意值都会加载CNTIN的值
    
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
		bsp_sensor_Config();		/*  电磁传感器初始化  */

	/*  选择是否需要校准传感器  */
	if(bsp_switch_GetValue() == 14)
		bsp_sensor_Calibration();

	/*  注册软件定时器任务,传感器数据上传任务,周期50ms  */
	bsp_tim_CreateSoftTimer(0, 49, debug_CarDataReport, TIMER_MODE_AUTO);
	
	/*  按键扫描任务,周期20ms  */
//	bsp_tim_CreateSoftTimer(1, 21, bsp_key_Scan, TIMER_MODE_AUTO);
	
	/*  LED状态指示任务,周期500ms  */
	bsp_tim_CreateSoftTimer(2, 453, test_LedTest, TIMER_MODE_AUTO);
	
	/*  OLED参数显示任务,周期500ms  */
	bsp_tim_CreateSoftTimer(3, 501, debug_ShowPara, TIMER_MODE_AUTO);
	
	/*  编码器速度计算,周期10ms  */
//	bsp_tim_CreateSoftTimer(4, 11, bsp_encoder_SpeedCalc, TIMER_MODE_AUTO);
	
	/*  硬件定时器任务,车子控制任务,周期20ms  */
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
