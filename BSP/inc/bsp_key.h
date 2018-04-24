# ifndef __BSP_KEY_H
# define __BSP_KEY_H


# include "bsp.h"

	
# define KEY_UP_PIN			GPIO_Pin_D3
# define KEY_OK_PIN			GPIO_Pin_D4
# define KEY_DOWN_PIN		GPIO_Pin_E4

/*  ��������ʱ��, ��λΪms,ȡ���ڶ��ʱ����һ������,
		���೤ʱ�����һ��bsp_key_Scan����*/
# define KEY_LONG_TIME		50
# define KEY_FILTER_TIME	5
# define KEY_FIFO_SIZE	10


/*  ����ID����,��Ҫ���ں���bsp_  */
typedef enum{
	KEY_ID_UP = 0x00,
	KEY_ID_OK,
	KEY_ID_DOWN,
	KEY_COUNT,
}KEYID_EnumTypeDef;

/*  ������ֵ����  */
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


/*  �����ṹ��  */
typedef struct 
{
	uint8_t Fifo[KEY_FIFO_SIZE];
	uint8_t Read;
	uint8_t Write;
}KeyFIFO_TypeDef;

/*  ÿ��������Ӧ1��ȫ�ֵĽṹ����� */
typedef struct
{
	uint8_t (*IsKeyPressFunc)(void);	/*  ����ָ��,�����жϰ����Ƿ񱻰���  */
	
	uint8_t FilterCount;	/*  �����˲�������  */
	uint16_t LongCount;		/*  ��������������  */
	uint16_t LongTime;		/*  ��������ʱ��,0��ʾ����ⳤ��  */
	uint8_t  State;				/*  ������ǰ״̬,���»��ǵ���  */
	uint8_t RepeatSpeed;	/*  ������������  */
	uint8_t RepeatCount;	/*  ��������������  */
}Key_TypeDef;


void bsp_key_Config(void);
void bsp_key_Scan(void);
void bsp_key_Clear(void);
void bsp_key_SetPara(uint8_t KeyID, uint16_t LongTime, uint16_t RepeatSpeed);

void bsp_key_PutKey(uint8_t KeyValue);
uint8_t bsp_key_GetKey(void);
uint8_t bsp_key_GetState(KEYID_EnumTypeDef KeyId);


# endif


