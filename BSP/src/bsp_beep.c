/**
  *******************************************************************************************************
  * File Name: bsp_beep.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-18
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# define BEEP_PIN	GPIO_Pin_B1

# define BEEP_ENABLE()	drv_gpio_WritePin(BEEP_PIN, GPIO_PIN_RESET)
# define BEEP_DISABLE()	drv_gpio_WritePin(BEEP_PIN, GPIO_PIN_SET)

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static BEEP_TypeDef Beep;

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_B0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = DISABLE;
	drv_gpio_Init(&GPIO_InitStruct);
	
	drv_gpio_WritePin(GPIO_Pin_B0, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_B1;
	drv_gpio_Init(&GPIO_InitStruct);
	
	BEEP_DISABLE();
	Beep.ucMute = 0;
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_ON(uint16_t BeepTime, uint16_t StopTime, uint16_t Cycle)
{
	if(BeepTime == 0 || Beep.ucMute == 1) return;
	
	Beep.usBeepTime = BeepTime;
	Beep.usStopTime = StopTime;
	Beep.usCycle = Cycle;
	Beep.usCycleCount = 0;
	Beep.usCount = 0;
	Beep.ucState = 0;
	Beep.ucEnable = ENABLE;
	
	BEEP_ENABLE();
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_OFF(void)
{
	Beep.ucEnable = DISABLE;
	
	if((Beep.usStopTime == 0) || (Beep.usCycle == 0))
	{
		BEEP_DISABLE();
	}
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_KeyTone(void)
{
	bsp_beep_ON(5, 1, 1);
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_Pause(void)
{
	bsp_beep_OFF();
	Beep.ucMute = ENABLE;
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_Resume(void)
{
	bsp_beep_OFF();
	Beep.ucMute = DISABLE;
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_beep_Thread(void)
{
	/*  ���������û�д򿪻���û��ֹͣʱ��,���߿����˾���,����Ҫ����  */
	if((Beep.ucEnable == DISABLE) || (Beep.usStopTime == 0) || (Beep.ucMute == ENABLE)) return;
	
	/*  ��Ҫ����  */
	if(Beep.ucState == 0)
	{
		if(Beep.usStopTime > 0)	/*  ��ֹͣʱ��,��Ϸ���  */
		{
			if(++ Beep.usCount >= Beep.usBeepTime)
			{
				BEEP_DISABLE();
				Beep.usCount = 0;
				Beep.ucState = 1;
			}
		}
		else
		{
			/*  û��ֹͣʱ��,��������  */
		}
	}
	else if(Beep.ucState == 1)
	{
		if(++ Beep.usCount >= Beep.usStopTime)
		{
			if(Beep.usCycle >0)	/*  һֱ����,ֱ������OFF  */
			{
				if(++ Beep.usCycleCount >= Beep.usCycle)
				{
					/*  ѭ��������,ֹͣ����  */
					Beep.ucEnable = DISABLE;
				}
				if(Beep.ucEnable == DISABLE)
				{
					Beep.usStopTime = 0;
					return;
				}
			}
			
			Beep.usCount = 0;
			Beep.ucState = 0;
			BEEP_ENABLE();		/*  ����������  */
		}
	}
}



/********************************************  END OF FILE  *******************************************/