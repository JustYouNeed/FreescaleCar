/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-12
  * Brief: ���ļ�Ϊ���ذ�����������
  *******************************************************************************************************
  * History
  *		1.Author:	Vector
	* 		Date: 2018-2-12
	*			Mod: �����ļ�
	*  
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.�޸İ���FIFO�ṹ������,��Key_Fifo_Str�޸�ΪKeyFIFO_TypeDef
	*					 2.�޸İ���״̬�ṹ������,��Key_Str�޸�ΪKey_TypeDef
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_KEY_H
# define __BSP_KEY_H


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

/*  ������������  */
# define KEY_UP_PIN				GPIO_Pin_A0
# define KEY_DOWN_PIN			GPIO_Pin_A1
# define KEY_OK_PIN				GPIO_Pin_A2

/*  ��������  */
# define BSP_KEY_COUNT		3

/*  ����ʱ�䶨��  */
# define KEY_LONG_TIME		220
# define KEY_FILTER_TIME	1

/*  ����FIFO���  */
# define KEY_FIFO_SIZE		10
# define KEY_PRESS				1
# define KEY_UNPRESS			0

/*  ����IDö�ٱ���  */
typedef enum{
	KEY_ID_UP = 0x00,
	KEY_ID_OK,
	KEY_ID_DOWN,
}KEY_ID_ENUM;


/*  ����״̬ö�ٱ���  */
typedef enum
{
	KEY_NONE = 0x00,
	
	KEY_UP_PRESS,
	KEY_UP_UP,
	KEY_UP_LONG,
	
	KEY_OK_PRESS,
	KEY_OK_UP,
	KEY_OK_LONG,
	
	KEY_DOWN_PRESS,
	KEY_DOWN_UP,
	KEY_DOWN_LONG
}KEY_STAT_ENUM;


/*  ����FIFO�ṹ�嶨��  */
typedef struct 
{
	uint8_t Fifo[KEY_FIFO_SIZE];
	uint8_t Read;
	uint8_t Write;
	
	uint8_t IsConfig;
}KeyFIFO_TypeDef;

/*
	ÿ��������Ӧ1��ȫ�ֵĽṹ�������
*/
typedef struct
{
	uint8_t (*IsKeyPressFunc)(void);
	
	uint8_t Count;
	uint8_t State;
	uint8_t RepeatSpeed;
	uint8_t RepeatCount;
	uint16_t LongCount;
	uint16_t LongTime;
}Key_TypeDef;


void bsp_key_Config(void);
void bsp_key_Scan(void);
void bsp_key_PutKeyToFIFO(uint8_t KeyValue);
void bsp_key_ClearFIFO(void);
uint8_t bsp_key_GetKey(void);
uint8_t bsp_key_GetKeyState(KEY_ID_ENUM KeyId);

# endif

/********************************************  END OF FILE  *******************************************/

