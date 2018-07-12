/**
  *******************************************************************************************************
  * File Name: drv_spi.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: 本文件声明了有关SPI操作的函数宏定义以及相关变量
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-17
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __DRV_SPI_H
# define __DRV_SPI_H

/*  SPI通道引脚说明  */
/*
			SPIx							SPI0  									   		SPI1
			Channel		0									1           0                 1
			SCK				B2								E0          D0                G4
			MOSI			B3								E1					D1								G5
			MISO			B4								E2					D2								G6
			CS				B5								E3					D3								G7
*/


/*  SPI通信方向宏定义  */
# define SPI_Direction_2Lines_FullDuplex ((uint8_t)0x00)
# define SPI_Direction_2Lines_RxOnly     ((uint8_t)0x01)
# define SPI_Direction_1Line_Rx          ((uint8_t)0x02)
# define SPI_Direction_1Line_Tx          ((uint8_t)0x03)

/*  SPI模式宏定义  */
# define SPI_Mode_Master                 ((uint8_t)0x10)
# define SPI_Mode_Slave                  ((uint8_t)0x00)

/*  时钟极性宏定义  */
# define SPI_CPOL_Low                    ((uint8_t)0x00)
# define SPI_CPOL_High                   ((uint8_t)0x01)

/*  时钟相位宏定义  */
# define SPI_CPHA_1Edge                  ((uint8_t)0x00)
# define SPI_CPHA_2Edge                  ((uint8_t)0x01)

/*  片选模式宏定义  */
# define SPI_NSS_Soft                    ((uint8_t)0x00)
# define SPI_NSS_Hard                    ((uint8_t)0x01)

/*  LSB宏定义  */
# define SPI_FirstBit_MSB                ((uint8_t)0x01)
# define SPI_FirstBit_LSB                ((uint8_t)0x00)

/*  SPI时钟分频因子宏定义  */
# define SPI_BaudRateDivider_1					 ((uint8_t)0x00)
# define SPI_BaudRateDivider_2					 ((uint8_t)0x01)
# define SPI_BaudRateDivider_3					 ((uint8_t)0x02)
# define SPI_BaudRateDivider_4					 ((uint8_t)0x03)
# define SPI_BaudRateDivider_5				   ((uint8_t)0x04)
# define SPI_BaudRateDivider_6					 ((uint8_t)0x05)
# define SPI_BaudRateDivider_7					 ((uint8_t)0x06)
# define SPI_BaudRateDivider_8					 ((uint8_t)0x07)

/*  SPI时钟预分频因子宏定义  */
# define SPI_BaudRatePrescaler_2         ((uint8_t)0x00)
# define SPI_BaudRatePrescaler_4         ((uint8_t)0x01)
# define SPI_BaudRatePrescaler_8         ((uint8_t)0x02)
# define SPI_BaudRatePrescaler_16        ((uint8_t)0x03)
# define SPI_BaudRatePrescaler_32        ((uint8_t)0x04)
# define SPI_BaudRatePrescaler_64        ((uint8_t)0x05)
# define SPI_BaudRatePrescaler_128       ((uint8_t)0x06)
# define SPI_BaudRatePrescaler_256       ((uint8_t)0x07)
# define SPI_BaudRatePrescaler_512       ((uint8_t)0x08)

/*  SPI通道宏定义  */
# define SPI_Channel_0									 ((uint8_t)0x00)
# define SPI_Channel_1									 ((uint8_t)0x01)

typedef struct
{
  uint8_t SPI_Direction;		/*  传输方向,全双工或单线双向模式  */
  uint8_t SPI_Mode;        /*  SPI操作模式,主机或从机模式  */    
  uint8_t SPI_CPOL;				/*  时钟极性  */
  uint8_t SPI_CPHA;				/*  时钟相位  */
  uint8_t SPI_NSS; 				/*  软件片先或者硬件片选  */
  uint8_t SPI_BaudRatePrescaler;		/*  通信波特率预分频因子  */	
	uint8_t SPI_BaudRateDivider;			/*  通信波特率分频因子  */
  uint8_t SPI_FirstBit;		/*  MSB或者LSB  */ 
	uint8_t SPI_Channel;		/*  SPI通道  */
}SPI_InitTypeDef;


/*  SPI读取/写入超时时间  */
# define SPI_RECV_TIMEOUT		200


void drv_spi_Init(SPI_Type *SPIx, SPI_InitTypeDef *SPI_InitStruct);
uint8_t drv_spi_ReadWriteByte(SPI_Type *SPIx, uint8_t byte);
void drv_spi_SetChannel(SPI_Type *SPIx, uint8_t SPI_Channel_x);
void drv_spi_Cmd(SPI_Type *SPIx, FunctionalState NewState);
#endif


/********************************************  END OF FILE  *******************************************/


