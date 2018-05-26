/**
  *******************************************************************************************************
  * File Name: bsp_i2c.c
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-4
  * Brief: 本文件提供了有关IIC通信的底层驱动,软件模拟,与芯片无关,函数可重入,使用时需要为每一个IIC通信
	*				 单独声明一个IIC_TypeDef结构体,用于控制通信
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: 建立文件,完成基本函数
	*
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 修改延时函数为宏,使驱动更容易移植
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
* Description: IIC通信启动信号
*             
* Arguments  : 1.IIC: IIC通信控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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
* Arguments  : 1.IIC: IIC控制结构体
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


