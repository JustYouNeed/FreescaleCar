/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-23
  * Brief: 本文件提供了有关操作按键的函数,支持按键的多种状态检测
	*					1.按键按下检测
	*					2.按键长按检测
	*					3.按键弹起检测
	*					4.按键长按自动连发
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-23
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# define KEY_TONE		1


/*  检测按键是否按键的函数  */
static uint8_t IsKeyUpPress(void) { return (drv_gpio_ReadPin(KEY_UP_PIN) == 1)?0:1;}
static uint8_t IsKeyOkPress(void) { return (drv_gpio_ReadPin(KEY_OK_PIN) == 1)?0:1;}
static uint8_t IsKeyDownPress(void) { return (drv_gpio_ReadPin(KEY_DOWN_PIN) == 1)?0:1;}


/*  按键FIFO  */
static KeyFIFO_TypeDef		 bsp_key_fifo;   /* the key fifo struct */

/*  按键结构体组,对应板子上的三个按键  */
static Key_TypeDef		     bsp_key[KEY_COUNT];  /* struct of each key */


/*
*********************************************************************************************************
*                           bsp_key_GPIOConfig               
*
* Description: 初始化按键引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_HDrv = DISABLE;
	GPIO_InitStructure.GPIO_PuPd = DISABLE;

	GPIO_InitStructure.GPIO_Pin = KEY_UP_PIN;
	drv_gpio_Init(&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = KEY_OK_PIN;
	drv_gpio_Init(&GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = KEY_DOWN_PIN;
	drv_gpio_Init(&GPIO_InitStructure);	
}


/*
*********************************************************************************************************
*                        bsp_key_FifoConfig                  
*
* Description: 初始化按键FIFO
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_FifoConfig(void)
{
	uint8_t i = 0;
	
	/*  清空按键FIFO  */
	bsp_key_fifo.Read = 0;  
	bsp_key_fifo.Write = 0; 
	
	/*  循环初始化每一个按键FIFO  */
	for(i = 0; i < KEY_COUNT; i++)
	{
		bsp_key[i].LongTime = KEY_LONG_TIME;
		bsp_key[i].FilterCount = KEY_FILTER_TIME / 2;
		bsp_key[i].State = KEY_NONE;
		bsp_key[i].RepeatCount = 0;
		bsp_key[i].RepeatSpeed = 10;
	}
	
	bsp_key[KEY_ID_UP].IsKeyPressFunc = IsKeyUpPress;
	bsp_key[KEY_ID_OK].IsKeyPressFunc = IsKeyOkPress;
	bsp_key[KEY_ID_DOWN].IsKeyPressFunc = IsKeyDownPress;
}

/*
*********************************************************************************************************
*                          bsp_key_Config                
*
* Description: 初始化按键
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_Config(void)
{
	bsp_key_GPIOConfig();
	bsp_key_FifoConfig();
}

/*
*********************************************************************************************************
*                          bsp_key_SetPara                
*
* Description: 设置按键参数
*             
* Arguments  : 1> KeyID: 按键ID,在bsp_key.h中定义
*							 2> LongTime: 长按事件时间
*							 3> RepeatSpeed: 按键连发速度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_SetPara(uint8_t KeyID, uint16_t LongTime, uint16_t RepeatSpeed)
{
	bsp_key[KeyID].LongTime = LongTime;
	bsp_key[KeyID].LongCount = 0;
	
	bsp_key[KeyID].RepeatSpeed = RepeatSpeed;
	bsp_key[KeyID].RepeatCount = 0;
}


/*
*********************************************************************************************************
*                          bsp_GetKey                
*
* Description: 从按键FIFO中获取一个按键值
*             
* Arguments  : None.
*
* Reutrn     : 按键值,其值在bsp_key.h中定义
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_key_GetKey(void)
{
	uint8_t key;
	if(bsp_key_fifo.Read == bsp_key_fifo.Write) 
	{
		return KEY_NONE;
	}
	else
	{
		key = bsp_key_fifo.Fifo[bsp_key_fifo.Read];
		if( ++bsp_key_fifo.Read >= KEY_FIFO_SIZE) bsp_key_fifo.Read = 0;
		return key;
	}
}

/*
*********************************************************************************************************
*                        bsp_PutKey                  
*
* Description: 推送一个按键值到按键FIFO中
*             
* Arguments  : 1.> KeyValue: 按键值
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_PutKey(uint8_t KeyValue)
{
	bsp_key_fifo.Fifo[bsp_key_fifo.Write] = KeyValue;
	
	if(++bsp_key_fifo.Write >= KEY_FIFO_SIZE)
		bsp_key_fifo.Write = 0;
}


/*
*********************************************************************************************************
*                          bsp_GetKeyState                
*
* Description: 获取一个按键的状态
*             
* Arguments  : 1> KeyId: 按键ID,其值在bsp_key.h中定义
*
* Reutrn     : 对应按键的状态
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_key_GetState(KEYID_EnumTypeDef KeyId)
{
	return bsp_key[KeyId].State;
}

/*
*********************************************************************************************************
*                         bsp_key_Clear                 
*
* Description: 清空按键FIFO
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_Clear(void)
{
	bsp_key_fifo.Read = bsp_key_fifo.Write;
}

/*
*********************************************************************************************************
*                        bsp_key_Detect                  
*
* Description: 检测按键,非阻塞
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数为本文件内部函数,外部禁止调用
*********************************************************************************************************
*/
void bsp_key_Detect(uint8_t Id)
{
	Key_TypeDef *pKey;
	
	pKey = &bsp_key[Id];	/*  先获取到按键的结构体  */
	
	/*  判断是否按下  */
	if(pKey->IsKeyPressFunc())
	{
		
		if(pKey->FilterCount < KEY_FILTER_TIME) 
			pKey->FilterCount  = KEY_FILTER_TIME;
		else if(pKey->FilterCount < 2 * KEY_FILTER_TIME) 
			pKey->FilterCount ++;
		else
		{
			if(pKey->State == 0)	/*  如果上个时刻的按键状态为弹起状态  */
			{
				/*  按键被按下  */				
				pKey->State = 1;
				
				/*  推送按键值到按键FIFO  */
				bsp_key_PutKey((uint8_t)(3 * Id + 1));
				# if KEY_TONE
					bsp_beep_KeyTone();
				# endif
			}
			
			/*  长按时间大于零,说明开启了长按检测功能  */
			if(pKey->LongTime > 0)
			{
				/*  检测到了长按  */
				if(pKey->LongCount < pKey->LongTime)
				{
					if(++pKey->LongCount == pKey->LongTime)
					{
						/*  推送长按消息到按键FIFO  */
						bsp_key_PutKey((uint8_t)(3 * Id + 3));
						# if KEY_TONE
							bsp_beep_KeyTone();
						# endif
					}
				}
				else	/*  如果已经超过了长按时间,则看看是否开启了按键连发  */
				{
					/*  开启了按键连发功能  */
					if(pKey->RepeatSpeed > 0)
					{
						if(++pKey->RepeatCount >= pKey->RepeatSpeed)
						{
							pKey->RepeatCount = 0;
							bsp_key_PutKey((uint8_t)(3 * Id + 1));
							
							# if KEY_TONE
								bsp_beep_KeyTone();
							# endif
						}
					}
				}
			}
		}
	}
	else	/*  没有被按下则将按键状态设置为弹起状态,并推送到FIFO  */
	{
		if(pKey->FilterCount > KEY_FILTER_TIME)
			pKey->FilterCount  = KEY_FILTER_TIME;
		else if(pKey->FilterCount != 0)
			pKey->FilterCount -- ;
		else
		{
			if(pKey->State == 1)
			{
				pKey->State = 0;
				bsp_key_PutKey((uint8_t)(3 * Id + 2));
			}
		}
		
		pKey->RepeatCount = 0;
		pKey->LongCount = 0;
	}
}

/*
*********************************************************************************************************
*                            bsp_key_Scan              
*
* Description: 按键扫描函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 本函数为按键扫描函数,在使用bsp_key_GetKey之前应先调用本函数,或者可以周期性调用,然后调用
*								bsp_key_GetKey从按键FIFO中获取扫描到的按键
*********************************************************************************************************
*/
void bsp_key_Scan(void)
{
	uint8_t i = 0;
	
	/*  循环扫描每一个按键  */
	for(i = 0; i < KEY_COUNT; i++)
	{
		bsp_key_Detect(i);
	}
}

/********************************************  END OF FILE  *******************************************/






