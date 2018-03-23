# ifndef __BSP_IIC_H
# define __BSP_IIC_H

# include "bsp.h"

typedef uint8_t (*pFun)(void);
typedef void (*pFunction)(void);
typedef struct{
	pFunction set_sda_in;
	pFunction set_sda_out;
	pFunction set_sda_high;
	pFunction set_sda_low;
	pFun read_sda;
	pFunction set_scl_high;
	pFunction set_scl_low;
}IIC_TypeDef, *pIIC_TypeDef;

void bsp_i2c_Start(IIC_TypeDef i2c_structure);
void bsp_i2c_Stop(IIC_TypeDef i2c_structure);
void bsp_i2c_Ack(IIC_TypeDef i2c_structure);
void bsp_i2c_NoAck(IIC_TypeDef i2c_structure);
uint8_t bsp_i2c_WaitAck(IIC_TypeDef i2c_structure);
void bsp_i2c_SendByte(IIC_TypeDef i2c_structure, uint8_t byte);
uint8_t bsp_i2c_ReadByte(IIC_TypeDef i2c_structure, uint8_t ack);


# endif




