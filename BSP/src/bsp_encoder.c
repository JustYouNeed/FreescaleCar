/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V2.1.1
  * Date: 2018-2-28
  * Brief: 本文件提供了有关编码器的基本操作函数,如初始化、读取编码器值等,编码器采用中断方式计数
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-2-28
	*     Mod: 建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.将原本独立的编码器数据整合到Car结构体中,便于管理
	*
	*		3.Author: Vector
	*			Date: 2018-3-16
	*			Mod: 1.将编码器计数方式由KBI中断改为FTM计数器
	*					 2.删除电机编码器中断回调函数
	*				   3.修改电机速度计算方式,由计算两次数值差改为读取计数器后清零方式
	*
	*		4.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 去除速度计算功能,只读取编码器的值
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "FreescaleCar.h"

/*
*********************************************************************************************************
*                              bsp_encoder_SpeedCalc            
*
* Description: 由编码器的值计算电机速度
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_ReadCounter(void)
{	
	/*  读取FTM计数器值  */
	Car.Motor.RightEncoder = (uint32_t)FTM0->CNT;
	Car.Motor.LeftEncoder = (uint32_t)FTM1->CNT;
	
	/*  清空计数器  */
	FTM0->CNT = 0;
	FTM1->CNT = 0;
}


/*
*********************************************************************************************************
*                            bsp_encoder_Config              
*
* Description: 初始化编码器
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*  初始化左边编码器,采用FTM0,引脚E0  */
	SIM->SCGC |= SIM_SCGC_FTM0_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS(1);
	FTM0->SC &= ~(3);
	FTM0->SC |= (3 << 3);
	FTM0->CNT = 0;
	
	/*  初始化左边编码器,采用FTM1,引脚E7  */
	SIM->SCGC |= SIM_SCGC_FTM1_MASK;
	SIM->PINSEL &= ~SIM_PINSEL_FTM1CLKPS_MASK;
	SIM->PINSEL |= SIM_PINSEL_FTM1CLKPS(2);
	FTM1->SC |= FTM_SC_PS(0);
	FTM1->SC |= FTM_SC_CLKS(3);  
	FTM1->CNT = 0;
	
	
	/*  初始化编码器方向引脚  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = LEFTENCONDER_DIR_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  右边编码器方向引脚  */
	GPIO_InitStruct.GPIO_Pin = RIGHTENCONDER_DIR_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
//	

//	ftm_count_init(ftm0);
//	ftm_count_init(ftm1);
//	
	drv_gpio_PullCmd(LEFTENCONDER_DIR_PIN, ENABLE);
	drv_gpio_PullCmd(RIGHTENCONDER_DIR_PIN, ENABLE);
}


/********************************************  END OF FILE  *******************************************/


