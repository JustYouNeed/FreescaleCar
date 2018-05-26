/**
  *******************************************************************************************************
  * File Name: bsp_key.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-23
  * Brief: ���ļ��ṩ���йز��������ĺ���,֧�ְ����Ķ���״̬���
	*					1.�������¼��
	*					2.�����������
	*					3.����������
	*					4.���������Զ�����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-23
	*			Mod: �������ļ�
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


/*  ��ⰴ���Ƿ񰴼��ĺ���  */
static uint8_t IsKeyUpPress(void) { return (drv_gpio_ReadPin(KEY_UP_PIN) == 1)?0:1;}
static uint8_t IsKeyOkPress(void) { return (drv_gpio_ReadPin(KEY_OK_PIN) == 1)?0:1;}
static uint8_t IsKeyDownPress(void) { return (drv_gpio_ReadPin(KEY_DOWN_PIN) == 1)?0:1;}


/*  ����FIFO  */
static KeyFIFO_TypeDef		 bsp_key_fifo;   /* the key fifo struct */

/*  �����ṹ����,��Ӧ�����ϵ���������  */
static Key_TypeDef		     bsp_key[KEY_COUNT];  /* struct of each key */


/*
*********************************************************************************************************
*                           bsp_key_GPIOConfig               
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
* Description: ��ʼ������FIFO
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
	
	/*  ��հ���FIFO  */
	bsp_key_fifo.Read = 0;  
	bsp_key_fifo.Write = 0; 
	
	/*  ѭ����ʼ��ÿһ������FIFO  */
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
* Description: ��ʼ������
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
* Description: ���ð�������
*             
* Arguments  : 1> KeyID: ����ID,��bsp_key.h�ж���
*							 2> LongTime: �����¼�ʱ��
*							 3> RepeatSpeed: ���������ٶ�
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
* Description: �Ӱ���FIFO�л�ȡһ������ֵ
*             
* Arguments  : None.
*
* Reutrn     : ����ֵ,��ֵ��bsp_key.h�ж���
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
* Description: ����һ������ֵ������FIFO��
*             
* Arguments  : 1.> KeyValue: ����ֵ
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
* Description: ��ȡһ��������״̬
*             
* Arguments  : 1> KeyId: ����ID,��ֵ��bsp_key.h�ж���
*
* Reutrn     : ��Ӧ������״̬
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
* Description: ��հ���FIFO
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
* Description: ��ⰴ��,������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ��ڲ�����,�ⲿ��ֹ����
*********************************************************************************************************
*/
void bsp_key_Detect(uint8_t Id)
{
	Key_TypeDef *pKey;
	
	pKey = &bsp_key[Id];	/*  �Ȼ�ȡ�������Ľṹ��  */
	
	/*  �ж��Ƿ���  */
	if(pKey->IsKeyPressFunc())
	{
		
		if(pKey->FilterCount < KEY_FILTER_TIME) 
			pKey->FilterCount  = KEY_FILTER_TIME;
		else if(pKey->FilterCount < 2 * KEY_FILTER_TIME) 
			pKey->FilterCount ++;
		else
		{
			if(pKey->State == 0)	/*  ����ϸ�ʱ�̵İ���״̬Ϊ����״̬  */
			{
				/*  ����������  */				
				pKey->State = 1;
				
				/*  ���Ͱ���ֵ������FIFO  */
				bsp_key_PutKey((uint8_t)(3 * Id + 1));
				# if KEY_TONE
					bsp_beep_KeyTone();
				# endif
			}
			
			/*  ����ʱ�������,˵�������˳�����⹦��  */
			if(pKey->LongTime > 0)
			{
				/*  ��⵽�˳���  */
				if(pKey->LongCount < pKey->LongTime)
				{
					if(++pKey->LongCount == pKey->LongTime)
					{
						/*  ���ͳ�����Ϣ������FIFO  */
						bsp_key_PutKey((uint8_t)(3 * Id + 3));
						# if KEY_TONE
							bsp_beep_KeyTone();
						# endif
					}
				}
				else	/*  ����Ѿ������˳���ʱ��,�򿴿��Ƿ����˰�������  */
				{
					/*  �����˰�����������  */
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
	else	/*  û�б������򽫰���״̬����Ϊ����״̬,�����͵�FIFO  */
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
* Description: ����ɨ�躯��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ������Ϊ����ɨ�躯��,��ʹ��bsp_key_GetKey֮ǰӦ�ȵ��ñ�����,���߿��������Ե���,Ȼ�����
*								bsp_key_GetKey�Ӱ���FIFO�л�ȡɨ�赽�İ���
*********************************************************************************************************
*/
void bsp_key_Scan(void)
{
	uint8_t i = 0;
	
	/*  ѭ��ɨ��ÿһ������  */
	for(i = 0; i < KEY_COUNT; i++)
	{
		bsp_key_Detect(i);
	}
}

/********************************************  END OF FILE  *******************************************/






