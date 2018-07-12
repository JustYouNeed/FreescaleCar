/**
  *******************************************************************************************************
  * File Name: drv_spi.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-17
  * Brief: ���ļ��������й�SPI�����ĺ����궨���Լ���ر���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-17
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

# ifndef __DRV_SPI_H
# define __DRV_SPI_H

/*  SPIͨ������˵��  */
/*
			SPIx							SPI0  									   		SPI1
			Channel		0									1           0                 1
			SCK				B2								E0          D0                G4
			MOSI			B3								E1					D1								G5
			MISO			B4								E2					D2								G6
			CS				B5								E3					D3								G7
*/


/*  SPIͨ�ŷ���궨��  */
# define SPI_Direction_2Lines_FullDuplex ((uint8_t)0x00)
# define SPI_Direction_2Lines_RxOnly     ((uint8_t)0x01)
# define SPI_Direction_1Line_Rx          ((uint8_t)0x02)
# define SPI_Direction_1Line_Tx          ((uint8_t)0x03)

/*  SPIģʽ�궨��  */
# define SPI_Mode_Master                 ((uint8_t)0x10)
# define SPI_Mode_Slave                  ((uint8_t)0x00)

/*  ʱ�Ӽ��Ժ궨��  */
# define SPI_CPOL_Low                    ((uint8_t)0x00)
# define SPI_CPOL_High                   ((uint8_t)0x01)

/*  ʱ����λ�궨��  */
# define SPI_CPHA_1Edge                  ((uint8_t)0x00)
# define SPI_CPHA_2Edge                  ((uint8_t)0x01)

/*  Ƭѡģʽ�궨��  */
# define SPI_NSS_Soft                    ((uint8_t)0x00)
# define SPI_NSS_Hard                    ((uint8_t)0x01)

/*  LSB�궨��  */
# define SPI_FirstBit_MSB                ((uint8_t)0x01)
# define SPI_FirstBit_LSB                ((uint8_t)0x00)

/*  SPIʱ�ӷ�Ƶ���Ӻ궨��  */
# define SPI_BaudRateDivider_1					 ((uint8_t)0x00)
# define SPI_BaudRateDivider_2					 ((uint8_t)0x01)
# define SPI_BaudRateDivider_3					 ((uint8_t)0x02)
# define SPI_BaudRateDivider_4					 ((uint8_t)0x03)
# define SPI_BaudRateDivider_5				   ((uint8_t)0x04)
# define SPI_BaudRateDivider_6					 ((uint8_t)0x05)
# define SPI_BaudRateDivider_7					 ((uint8_t)0x06)
# define SPI_BaudRateDivider_8					 ((uint8_t)0x07)

/*  SPIʱ��Ԥ��Ƶ���Ӻ궨��  */
# define SPI_BaudRatePrescaler_2         ((uint8_t)0x00)
# define SPI_BaudRatePrescaler_4         ((uint8_t)0x01)
# define SPI_BaudRatePrescaler_8         ((uint8_t)0x02)
# define SPI_BaudRatePrescaler_16        ((uint8_t)0x03)
# define SPI_BaudRatePrescaler_32        ((uint8_t)0x04)
# define SPI_BaudRatePrescaler_64        ((uint8_t)0x05)
# define SPI_BaudRatePrescaler_128       ((uint8_t)0x06)
# define SPI_BaudRatePrescaler_256       ((uint8_t)0x07)
# define SPI_BaudRatePrescaler_512       ((uint8_t)0x08)

/*  SPIͨ���궨��  */
# define SPI_Channel_0									 ((uint8_t)0x00)
# define SPI_Channel_1									 ((uint8_t)0x01)

typedef struct
{
  uint8_t SPI_Direction;		/*  ���䷽��,ȫ˫������˫��ģʽ  */
  uint8_t SPI_Mode;        /*  SPI����ģʽ,������ӻ�ģʽ  */    
  uint8_t SPI_CPOL;				/*  ʱ�Ӽ���  */
  uint8_t SPI_CPHA;				/*  ʱ����λ  */
  uint8_t SPI_NSS; 				/*  ���Ƭ�Ȼ���Ӳ��Ƭѡ  */
  uint8_t SPI_BaudRatePrescaler;		/*  ͨ�Ų�����Ԥ��Ƶ����  */	
	uint8_t SPI_BaudRateDivider;			/*  ͨ�Ų����ʷ�Ƶ����  */
  uint8_t SPI_FirstBit;		/*  MSB����LSB  */ 
	uint8_t SPI_Channel;		/*  SPIͨ��  */
}SPI_InitTypeDef;


/*  SPI��ȡ/д�볬ʱʱ��  */
# define SPI_RECV_TIMEOUT		200


void drv_spi_Init(SPI_Type *SPIx, SPI_InitTypeDef *SPI_InitStruct);
uint8_t drv_spi_ReadWriteByte(SPI_Type *SPIx, uint8_t byte);
void drv_spi_SetChannel(SPI_Type *SPIx, uint8_t SPI_Channel_x);
void drv_spi_Cmd(SPI_Type *SPIx, FunctionalState NewState);
#endif


/********************************************  END OF FILE  *******************************************/


