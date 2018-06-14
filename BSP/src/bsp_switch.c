/**
  *******************************************************************************************************
  * File Name: bsp_switch.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ�������ϰ��뿪�صĻ�����������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"


# define BIT1_PIN			GPIO_Pin_F0
# define BIT2_PIN			GPIO_Pin_F1
# define BIT3_PIN			GPIO_Pin_G4
# define BIT4_PIN			GPIO_Pin_G7

# define MAG_SWITCH1_PIN	GPIO_Pin_F2
# define MAG_SWITCH2_PIN	GPIO_Pin_F3
/*
*********************************************************************************************************
*                     bsp_switch_Config                     
*
* Description: ��ʼ�������ϵİ��뿪��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_switch_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*  ��ʼ�����뿪������  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = BIT1_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT1_PIN, ENABLE);
	drv_gpio_WritePin(BIT1_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT2_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT2_PIN, ENABLE);
	drv_gpio_WritePin(BIT2_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT3_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT3_PIN, ENABLE);
	drv_gpio_WritePin(BIT3_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = BIT4_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
	drv_gpio_PullCmd(BIT4_PIN, ENABLE);
	drv_gpio_WritePin(BIT4_PIN, GPIO_PIN_SET);
	
	/*  �ɻɹ����ų�ʼ��,ͣ�����  */
	GPIO_InitStruct.GPIO_Pin = MAG_SWITCH1_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
//	drv_gpio_PullCmd(MAG_SWITCH1_PIN, ENABLE);
	drv_gpio_WritePin(MAG_SWITCH1_PIN, GPIO_PIN_SET);
	
	GPIO_InitStruct.GPIO_Pin = MAG_SWITCH2_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
//	drv_gpio_PullCmd(MAG_SWITCH2_PIN, ENABLE);
	drv_gpio_WritePin(MAG_SWITCH2_PIN, GPIO_PIN_SET);
}


/*
*********************************************************************************************************
*                         bsp_switch_GetValue                 
*
* Description: ��ȡ���뿪�ص�״̬
*             
* Arguments  : None.
*
* Reutrn     : 1> ���뿪�ص�״̬
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_switch_GetValue(void)
{
	uint8_t value = 0;
	
//	value |= (drv_gpio_ReadPin(BIT1_PIN) == 1) ? (1 << 0) : (value);
//	value |= (drv_gpio_ReadPin(BIT2_PIN) == 1) ? (1 << 1) : (value);
//	value |= (drv_gpio_ReadPin(BIT3_PIN) == 1) ? (1 << 2) : (value);
//	value |= (drv_gpio_ReadPin(BIT4_PIN) == 1) ? (1 << 3) : (value);
	value |= (uint8_t)((drv_gpio_ReadPin(BIT1_PIN) << 0) | (drv_gpio_ReadPin(BIT2_PIN) << 1));
	value |= (uint8_t)((drv_gpio_ReadPin(BIT3_PIN) << 2) | (drv_gpio_ReadPin(BIT4_PIN) << 3));
	
	return value;
}

uint8_t bsp_switch_GetStopState(void)
{
	return ((drv_gpio_ReadPin(MAG_SWITCH1_PIN) == 0) && (drv_gpio_ReadPin(MAG_SWITCH2_PIN) == 0));
}

/********************************************  END OF FILE  *******************************************/

