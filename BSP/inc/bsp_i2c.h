/**
  *******************************************************************************************************
  * File Name: bsp_i2c.h
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-3-4
  * Brief: ���ļ��ṩ��IICͨ�ŵ����ģ������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �������ļ�
	*		
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: ������ʱ������,ʹ������������ֲ
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_IIC_H
# define __BSP_IIC_H


# define IIC_DelayMs(x)		bsp_tim_DelayMs(x)
# define IIC_Delayus(x)		bsp_tim_DelayUs(x)

/*  ����ָ��  */
typedef uint8_t (*pFun)(void);
typedef void (*pFunction)(void);

/*  IICͨ�ſ��ƽṹ��  */
typedef struct{
	pFunction set_sda_in;			/*  ����SDAΪ���뺯��ָ��  */
	pFunction set_sda_out;		/*  ����SDAΪ�������ָ��  */
	pFunction set_sda_high;		/*  ����SDA����ߵ�ƽ����ָ��  */
	pFunction set_sda_low;		/*  ����SDA����͵�ƽ����ָ��  */
	pFun read_sda;						/*  ��SDA����ָ��  */
	pFunction set_scl_high;		/*  ����SCL����ߺ���ָ��  */
	pFunction set_scl_low;		/*  ����SCL����͵�ƽ����ָ��  */
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





