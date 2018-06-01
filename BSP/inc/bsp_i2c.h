/**
  *******************************************************************************************************
  * File Name: bsp_i2c.h
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-4
  * Brief: 本文件提供了IIC通信的软件模拟驱动
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: 建立本文件
	*		
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 增加延时函数宏,使驱动更容易移植
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_IIC_H
# define __BSP_IIC_H


# define IIC_DelayMs(x)		bsp_tim_DelayMs(x)
# define IIC_Delayus(x)		bsp_tim_DelayUs(x)

/*  函数指针  */
typedef uint8_t (*pFun)(void);
typedef void (*pFunction)(void);

/*  IIC通信控制结构体  */
typedef struct{
	pFunction set_sda_in;			/*  设置SDA为输入函数指针  */
	pFunction set_sda_out;		/*  设置SDA为输出函数指针  */
	pFunction set_sda_high;		/*  设置SDA输出高电平函数指针  */
	pFunction set_sda_low;		/*  设置SDA输出低电平函数指针  */
	pFun read_sda;						/*  读SDA函数指针  */
	pFunction set_scl_high;		/*  设置SCL输出高函数指针  */
	pFunction set_scl_low;		/*  设置SCL输出低电平函数指针  */
}IIC_TypeDef, *pIIC_TypeDef;


void bsp_i2c_Start(IIC_TypeDef IIC);
void bsp_i2c_Stop(IIC_TypeDef IIC);
void bsp_i2c_Ack(IIC_TypeDef IIC);
void bsp_i2c_NoAck(IIC_TypeDef IIC);
uint8_t bsp_i2c_WaitAck(IIC_TypeDef IIC);
void bsp_i2c_SendByte(IIC_TypeDef IIC, uint8_t byte);
uint8_t bsp_i2c_ReadByte(IIC_TypeDef IIC, uint8_t ack);


# endif

/********************************************  END OF FILE  *******************************************/





