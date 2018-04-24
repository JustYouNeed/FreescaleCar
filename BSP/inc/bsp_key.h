# ifndef __BSP_KEY_H
# define __BSP_KEY_H


# include "bsp.h"

	
# define KEY_UP_PIN			GPIO_Pin_D3
# define KEY_OK_PIN			GPIO_Pin_D4
# define KEY_DOWN_PIN		GPIO_Pin_E4

/*  按键长按时间, 单位为ms,取决于多久时间检测一个按键,
		即多长时间调用一个bsp_key_Scan函数*/
# define KEY_LONG_TIME		50
# define KEY_FILTER_TIME	5
# define KEY_FIFO_SIZE	10


/*  按键ID定义,主要用于函数bsp_  */
typedef enum{
	KEY_ID_UP = 0x00,
	KEY_ID_OK,
	KEY_ID_DOWN,
	KEY_COUNT,
}KEYID_EnumTypeDef;

/*  按键键值定义  */
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
}KEYValue_EnumTypeDef;


/*  按键结构体  */
typedef struct 
{
	uint8_t Fifo[KEY_FIFO_SIZE];
	uint8_t Read;
	uint8_t Write;
}KeyFIFO_TypeDef;

/*  每个按键对应1个全局的结构体变量 */
typedef struct
{
	uint8_t (*IsKeyPressFunc)(void);	/*  函数指针,用于判断按键是否被按下  */
	
	uint8_t FilterCount;	/*  按键滤波计数器  */
	uint16_t LongCount;		/*  按键长按计数器  */
	uint16_t LongTime;		/*  按键长度时间,0表示不检测长按  */
	uint8_t  State;				/*  按键当前状态,按下还是弹起  */
	uint8_t RepeatSpeed;	/*  按键连发周期  */
	uint8_t RepeatCount;	/*  按键连发计数器  */
}Key_TypeDef;


void bsp_key_Config(void);
void bsp_key_Scan(void);
void bsp_key_Clear(void);
void bsp_key_SetPara(uint8_t KeyID, uint16_t LongTime, uint16_t RepeatSpeed);

void bsp_key_PutKey(uint8_t KeyValue);
uint8_t bsp_key_GetKey(void);
uint8_t bsp_key_GetState(KEYID_EnumTypeDef KeyId);


# endif


