/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-12
  * Brief: 本文件为开发板上的按键驱动函数
  *******************************************************************************************************
  * History
  *		1.Author:	Vector
	* 		Date: 2018-2-12
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_key.h"

static KeyFIFO_TypeDef		 bsp_key_fifo;   /* 按键FIFO结构体 */
static Key_TypeDef		     bsp_key[BSP_KEY_COUNT];  /* 单个按键控制结构体 */


static uint8_t IsKeyUpPress(void) { return drv_gpio_ReadPin(KEY_UP_PIN);}
static uint8_t IsKeyOkPress(void) { return drv_gpio_ReadPin(KEY_OK_PIN);}
static uint8_t IsKeyDownPress(void) { return drv_gpio_ReadPin(KEY_DOWN_PIN);}

/*
*********************************************************************************************************
*                               bsp_key_GPIOInit           
*
* Description: 初始化按键功能的GPIO引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 此函数为本文件的私有函数,外部禁止调用
*********************************************************************************************************
*/
static void bsp_key_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = KEY_UP_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_HDrv = DISABLE;
	
	drv_gpio_Init(&GPIO_InitStructure);
	drv_gpio_WritePin(KEY_UP_PIN, GPIO_PIN_SET);
	
	GPIO_InitStructure.GPIO_Pin = KEY_OK_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
	drv_gpio_WritePin(KEY_OK_PIN, GPIO_PIN_SET);
	
	GPIO_InitStructure.GPIO_Pin = KEY_DOWN_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
	drv_gpio_WritePin(KEY_DOWN_PIN, GPIO_PIN_SET);
}


/*
*********************************************************************************************************
*                              bsp_key_FIFOInit            
*
* Description: 初始化按键FIFO
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 此函数为本文件的私有函数,外部禁止调用
*********************************************************************************************************
*/
static void bsp_key_FIFOInit(void)
{
		uint8_t i = 0;
	
	bsp_key_fifo.Read = 0;  
	bsp_key_fifo.Write = 0;
	bsp_key_fifo.Fifo[0] = 0;
	bsp_key_fifo.IsConfig = 1;  
	
	for(i = 0; i < BSP_KEY_COUNT; i++)
	{
		bsp_key[i].LongTime = KEY_LONG_TIME;
		bsp_key[i].Count = KEY_FILTER_TIME / 2;
		bsp_key[i].State = KEY_UNPRESS;
		bsp_key[i].RepeatCount = 0;
		bsp_key[i].RepeatSpeed = 0;
	}
	
	bsp_key[KEY_ID_UP].IsKeyPressFunc = IsKeyUpPress;
	bsp_key[KEY_ID_OK].IsKeyPressFunc = IsKeyOkPress;
	bsp_key[KEY_ID_DOWN].IsKeyPressFunc = IsKeyDownPress;
}


/*
*********************************************************************************************************
*                                    bsp_key_Config      
*
* Description: 初始化按键功能
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
	bsp_key_GPIOInit();
	bsp_key_FIFOInit();
}

/*
*********************************************************************************************************
*                                       bsp_key_PutKeyToFIFO   
*
* Description: put一个按键值到按键FIFO里
*             
* Arguments  : 1> KeyValue:按键值
*
* Reutrn     : None
*
* Note(s)    : KeyValue的值应该小于等于9，因为默认按键只有3个，每个按键三种状态
*********************************************************************************************************
*/
void bsp_key_PutKeyToFIFO(uint8_t KeyValue)
{
	if(KeyValue > 9) return ;
	
	bsp_key_fifo.Fifo[bsp_key_fifo.Write++] = KeyValue;  /*  将按键值写入FIFO  */
	
	if(bsp_key_fifo.Write >= KEY_FIFO_SIZE)   /*  如果FIFO写满了，则从头开始  */
		bsp_key_fifo.Write = 0;
}

/*
*********************************************************************************************************
*                                bsp_key_ClearFIFO          
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
void bsp_key_ClearFIFO(void)
{
	uint8_t i = 0;
	for(i = 0; i < KEY_FIFO_SIZE; i++)
	{
		bsp_key_fifo.Fifo[i] = KEY_NONE;
	}
	bsp_key_fifo.Read = bsp_key_fifo.Write = 0;
}

/*
*********************************************************************************************************
*                               bsp_key_GetKey           
*
* Description: 从按键FIFO中读取一个按键状态
*             
* Arguments  : None.
*
* Reutrn     : 按键值
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_key_GetKey(void)
{
	uint8_t key;	
	if(bsp_key_fifo.Read == bsp_key_fifo.Write) /*  FIFO为空  */
	{
		key = KEY_NONE;
	}
	else
	{
		key = bsp_key_fifo.Fifo[bsp_key_fifo.Read];
		if( ++bsp_key_fifo.Read >= KEY_FIFO_SIZE) 		/*  环形队列  */
			bsp_key_fifo.Read = 0;
	}
	return key;
}

/*
*********************************************************************************************************
*                                          
*
* Description: 获取按键的状态
*             
* Arguments  : 1> KeyId:按键ID
*
* Reutrn     : 按键状态
*
* Note(s)    : KeyId在bsp_key.h中定义
*********************************************************************************************************
*/
uint8_t bsp_key_GetKeyState(KEY_ID_ENUM KeyId)
{
	return bsp_key[KeyId].State;
}


/*
*********************************************************************************************************
*                                    bsp_key_Detect      
*
* Description: 检测按键状态
*             
* Arguments  : 1> Id:想检测的按键ID
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_Detect(uint8_t Id)
{
	Key_TypeDef *pKey;   /*  先声明一个按键结构体  */
	
	pKey = &bsp_key[Id];		/*  获取相应ID的按键结构体  */
	
	if(pKey->IsKeyPressFunc() == KEY_PRESS)   /*  如果按键状态有改变  */
	{
		
		if(pKey->Count < KEY_FILTER_TIME) 	/*  软件消抖  */
			pKey->Count  = KEY_FILTER_TIME;
		else if(pKey->Count < 2 * KEY_FILTER_TIME) 
			pKey->Count ++;
		else									/*  软件消抖完成  */
		{
			if(pKey->State == KEY_UNPRESS)	/*  改变按键状态,同时更新FIFO  */
			{
				
				pKey->State = KEY_PRESS;
				bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 1));
			}
			
			if(pKey->LongTime > 0)		/*  该变量大于0时说明了开启了长按检测  */
			{
				if(++pKey->LongCount == pKey->LongTime)	/*  发送了长按事件  */
				{
					bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 3));	/*  推送按键值到按键FIFO  */
				}
			}
			else  
			{
				if(pKey->RepeatSpeed > 0)
				{
					if( ++ pKey->RepeatCount >= pKey->RepeatSpeed)
					{
						pKey->RepeatCount = 0;
						bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 1));
					}
				} /*  end if(pKey->RepeatSpeed > 0)  */
			} /*  end (pKey->LongTime > 0) else  */
		} /*  end else  */
	}
	else		/*  按键没有被按下  */
	{
		if(pKey->Count > KEY_FILTER_TIME)
			pKey->Count  = KEY_FILTER_TIME;
		else if(pKey->Count != 0)
			pKey->Count -- ;
		else
		{
			if(pKey->State == KEY_PRESS)	/*  改变按键状态,同时更新FIFO  */
			{
				pKey->State = KEY_UNPRESS;
				bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 2));
			}
		}
		
		pKey->RepeatCount = 0;
		pKey->LongCount = 0;
	}
}

/*
*********************************************************************************************************
*                                 bsp_key_Scan         
*
* Description: 按键状态扫描函数，扫描按键状态
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
extern uint8_t TimerTaskRunMutexSignal;
# include "bsp_led.h"
void bsp_key_Scan(void)
{
	uint8_t i = 0;
	uint8_t key = 0;
	
//	if(TimerTaskRunMutexSignal == 1) return ;
//	TimerTaskRunMutexSignal = 1;
	drv_gpio_WritePin(KEY_UP_PIN, GPIO_PIN_SET);
	drv_gpio_WritePin(KEY_OK_PIN, GPIO_PIN_SET);
	drv_gpio_WritePin(KEY_DOWN_PIN, GPIO_PIN_SET);
	for(i = 0; i < BSP_KEY_COUNT; i++)
	{
		bsp_key_Detect(i);
	}
	
//	key = bsp_key_GetKey();
//	if(key == KEY_UP_PRESS) bsp_led_Toggle(1);
//	else if(key == KEY_OK_PRESS) bsp_led_Toggle(2);
//	else if(key == KEY_DOWN_PRESS) bsp_led_Toggle(3);
//	TimerTaskRunMutexSignal = 0;
}


/********************************************  END OF FILE  *******************************************/


