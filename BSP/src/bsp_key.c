/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-12
  * Brief: ���ļ�Ϊ�������ϵİ�����������
  *******************************************************************************************************
  * History
  *		1.Author:	Vector
	* 		Date: 2018-2-12
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_key.h"

static KeyFIFO_TypeDef		 bsp_key_fifo;   /* ����FIFO�ṹ�� */
static Key_TypeDef		     bsp_key[BSP_KEY_COUNT];  /* �����������ƽṹ�� */


static uint8_t IsKeyUpPress(void) { return drv_gpio_ReadPin(KEY_UP_PIN);}
static uint8_t IsKeyOkPress(void) { return drv_gpio_ReadPin(KEY_OK_PIN);}
static uint8_t IsKeyDownPress(void) { return drv_gpio_ReadPin(KEY_DOWN_PIN);}

/*
*********************************************************************************************************
*                               bsp_key_GPIOInit           
*
* Description: ��ʼ���������ܵ�GPIO����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �˺���Ϊ���ļ���˽�к���,�ⲿ��ֹ����
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
* Description: ��ʼ������FIFO
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �˺���Ϊ���ļ���˽�к���,�ⲿ��ֹ����
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
* Description: ��ʼ����������
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
* Description: putһ������ֵ������FIFO��
*             
* Arguments  : 1> KeyValue:����ֵ
*
* Reutrn     : None
*
* Note(s)    : KeyValue��ֵӦ��С�ڵ���9����ΪĬ�ϰ���ֻ��3����ÿ����������״̬
*********************************************************************************************************
*/
void bsp_key_PutKeyToFIFO(uint8_t KeyValue)
{
	if(KeyValue > 9) return ;
	
	bsp_key_fifo.Fifo[bsp_key_fifo.Write++] = KeyValue;  /*  ������ֵд��FIFO  */
	
	if(bsp_key_fifo.Write >= KEY_FIFO_SIZE)   /*  ���FIFOд���ˣ����ͷ��ʼ  */
		bsp_key_fifo.Write = 0;
}

/*
*********************************************************************************************************
*                                bsp_key_ClearFIFO          
*
* Description: ��հ���FIFO
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
* Description: �Ӱ���FIFO�ж�ȡһ������״̬
*             
* Arguments  : None.
*
* Reutrn     : ����ֵ
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_key_GetKey(void)
{
	uint8_t key;	
	if(bsp_key_fifo.Read == bsp_key_fifo.Write) /*  FIFOΪ��  */
	{
		key = KEY_NONE;
	}
	else
	{
		key = bsp_key_fifo.Fifo[bsp_key_fifo.Read];
		if( ++bsp_key_fifo.Read >= KEY_FIFO_SIZE) 		/*  ���ζ���  */
			bsp_key_fifo.Read = 0;
	}
	return key;
}

/*
*********************************************************************************************************
*                                          
*
* Description: ��ȡ������״̬
*             
* Arguments  : 1> KeyId:����ID
*
* Reutrn     : ����״̬
*
* Note(s)    : KeyId��bsp_key.h�ж���
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
* Description: ��ⰴ��״̬
*             
* Arguments  : 1> Id:����İ���ID
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_key_Detect(uint8_t Id)
{
	Key_TypeDef *pKey;   /*  ������һ�������ṹ��  */
	
	pKey = &bsp_key[Id];		/*  ��ȡ��ӦID�İ����ṹ��  */
	
	if(pKey->IsKeyPressFunc() == KEY_PRESS)   /*  �������״̬�иı�  */
	{
		
		if(pKey->Count < KEY_FILTER_TIME) 	/*  �������  */
			pKey->Count  = KEY_FILTER_TIME;
		else if(pKey->Count < 2 * KEY_FILTER_TIME) 
			pKey->Count ++;
		else									/*  ����������  */
		{
			if(pKey->State == KEY_UNPRESS)	/*  �ı䰴��״̬,ͬʱ����FIFO  */
			{
				
				pKey->State = KEY_PRESS;
				bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 1));
			}
			
			if(pKey->LongTime > 0)		/*  �ñ�������0ʱ˵���˿����˳������  */
			{
				if(++pKey->LongCount == pKey->LongTime)	/*  �����˳����¼�  */
				{
					bsp_key_PutKeyToFIFO((uint8_t)(3 * Id + 3));	/*  ���Ͱ���ֵ������FIFO  */
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
	else		/*  ����û�б�����  */
	{
		if(pKey->Count > KEY_FILTER_TIME)
			pKey->Count  = KEY_FILTER_TIME;
		else if(pKey->Count != 0)
			pKey->Count -- ;
		else
		{
			if(pKey->State == KEY_PRESS)	/*  �ı䰴��״̬,ͬʱ����FIFO  */
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
* Description: ����״̬ɨ�躯����ɨ�谴��״̬
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


