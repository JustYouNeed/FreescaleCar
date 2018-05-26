/**
  *******************************************************************************************************
  * File Name: bsp_i2c.c
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-4
  * Brief: ���ļ��ṩ���й�IICͨ�ŵĵײ�����,���ģ��,��оƬ�޹�,����������,ʹ��ʱ��ҪΪÿһ��IICͨ��
	*				 ��������һ��IIC_TypeDef�ṹ��,���ڿ���ͨ��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �����ļ�,��ɻ�������
	*
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: �޸���ʱ����Ϊ��,ʹ������������ֲ
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*
*********************************************************************************************************
*                               bsp_i2c_Start           
*
* Description: IICͨ�������ź�
*             
* Arguments  : 1.IIC: IICͨ�ſ��ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_i2c_Start(IIC_TypeDef IIC)
{
	IIC.set_sda_out();
	
	IIC.set_sda_high();
	IIC.set_scl_high();
	
	IIC_Delayus(4);
	IIC.set_sda_low();
	IIC_Delayus(4);
	IIC.set_scl_low();
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_i2c_Stop(IIC_TypeDef IIC)
{
	IIC.set_sda_out();
	IIC.set_scl_low();
	IIC.set_sda_low();
	
	IIC_Delayus(4);
	
	IIC.set_scl_high();
	IIC.set_sda_high();
	
	IIC_Delayus(4);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_i2c_Ack(IIC_TypeDef IIC)
{
	IIC.set_scl_low();
	
	IIC.set_sda_out();
	
	IIC.set_sda_low();
	
	IIC_Delayus(5);
	
	IIC.set_scl_high();
	
	IIC_Delayus(5);
	
	IIC.set_scl_low();
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_i2c_NoAck(IIC_TypeDef IIC)
{
	IIC.set_scl_low();
	IIC.set_sda_out();
	IIC.set_sda_high();
	
	IIC_Delayus(5);
	
	IIC.set_scl_high();
	
	IIC_Delayus(5);
	
	IIC.set_scl_low();
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_i2c_WaitAck(IIC_TypeDef IIC)
{
	uint8_t ErrTime = 0;
	
	IIC.set_sda_in();
	IIC.set_sda_high();
	IIC_Delayus(1);
	IIC.set_scl_high();
	IIC_Delayus(1);
	
	while(IIC.read_sda())
	{
		ErrTime ++;
		if(ErrTime > 250)
		{
			bsp_i2c_Stop(IIC);
			return 1;
		}
	}
	IIC.set_scl_low();
	return 0;
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_i2c_SendByte(IIC_TypeDef IIC, uint8_t byte)
{
	uint8_t cnt = 0x0;
	
	IIC.set_sda_out();
	
	IIC.set_scl_low();
	
	for(; cnt < 8; cnt ++)
	{
		if((byte&0x80)>>7 == 1) IIC.set_sda_high();
		else if((byte&0x80)>>7 == 0) IIC.set_sda_low();
		byte <<= 1;
		
		IIC_Delayus(5);
		IIC.set_scl_high();
		IIC_Delayus(5);
		IIC.set_scl_low();
		IIC_Delayus(2);
	}
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 1.IIC: IIC���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_i2c_ReadByte(IIC_TypeDef IIC, uint8_t ack)
{
	uint8_t cnt = 0x0, rec = 0x0;
	
	IIC.set_sda_in();
	
	for(; cnt < 8; cnt ++)
	{
		IIC.set_scl_low();
		IIC_Delayus(5);
		IIC.set_scl_high();
		
		rec <<= 1;
		if(IIC.read_sda() == 1) rec ++;
		
		IIC_Delayus(3);
	}
	if(!ack)
		bsp_i2c_NoAck(IIC);
	else
		bsp_i2c_Ack(IIC);
	return rec;
}

/********************************************  END OF FILE  *******************************************/


