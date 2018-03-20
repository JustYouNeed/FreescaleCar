/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.1
  * Date: 2018-2-12
  * Brief: 本文件为主控按键驱动声明
  *******************************************************************************************************
  * History
  *		1.Author:	Vector
	* 		Date: 2018-2-12
	*			Mod: 建立文件
	*  
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.修改按键FIFO结构体名字,由Key_Fifo_Str修改为KeyFIFO_TypeDef
	*					 2.修改按键状态结构体名字,由Key_Str修改为Key_TypeDef
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

/*  按键引脚声明  */
# define KEY_UP_PIN				GPIO_Pin_D3
# define KEY_DOWN_PIN			GPIO_Pin_E4
# define KEY_OK_PIN				GPIO_Pin_D4

/*  按键数量  */
# define BSP_KEY_COUNT		3

/*  长按时间定义  */
# define KEY_LONG_TIME		4000
# define KEY_FILTER_TIME	5

/*  按键FIFO深度  */
# define KEY_FIFO_SIZE		10
# define KEY_PRESS				0
# define KEY_UNPRESS			1

/*  按键ID枚举变量  */
typedef enum{
	KEY_ID_UP = 0x00,
	KEY_ID_OK,
	KEY_ID_DOWN,
}KEY_ID_ENUM;


/*  按键状态枚举变量  */
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


/*  按键FIFO结构体定义  */
typedef struct 
{
	uint8_t Fifo[KEY_FIFO_SIZE];
	uint8_t Read;
	uint8_t Write;
	
	uint8_t IsConfig;
}KeyFIFO_TypeDef;

/*
	每个按键对应1个全局的结构体变量。
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

