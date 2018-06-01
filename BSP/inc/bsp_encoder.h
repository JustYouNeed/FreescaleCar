/**
  *******************************************************************************************************
  * File Name: bsp_enconder.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-28
  * Brief: ���ļ��ṩ�˳��ӱ���������������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-18
	*			Mod: �������ļ�
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_ENCODER_H
# define __BSP_ENCODER_H


# define LEFTENCONDER_DIR_PIN		GPIO_Pin_B2

# define RIGHTENCONDER_DIR_PIN		GPIO_Pin_E6

# define READ_DIR(pin)		drv_gpio_ReadPin(pin)

void bsp_encoder_Config(void);
void bsp_encoder_ReadCounter(void);

# endif

/********************************************  END OF FILE  *******************************************/

