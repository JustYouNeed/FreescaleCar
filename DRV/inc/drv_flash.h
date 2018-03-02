/**
  *******************************************************************************************************
  * File Name: drv_flash.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ�˶�KEA128�ڲ�FLASH�Ĳ�������,���ֳ���ο���ɿƼ�
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

# include "derivative.h"

// flash commands 
# define FLASH_ERASE_VERITF_ALL_BLOCKS             0x01  // ����������������
# define FLASH_ERASE_VERITF_BLOCKS                 0x02  // �����������ݿ�
# define FLASH_ERASE_VERITF_SECTION          			 0x03  // ��������Flash ��
# define FLASH_READ_ONCE                           0x04  // ��ȡһ��
# define FLASH_PROGRAM                       			 0x06  // ���Flash
# define FLASH_PROGRAM_ONCE                        0x07  // ���һ��
# define FLASH_ERASE_ALL_BLOCKS                    0x08  // ������������
# define FLASH_ERASE_BLOCKS        			           0x09  // ����Flash ����
# define FLASH_ERASE_SECTOR			                   0x0A  // ����Flash ����
# define FLASH_UNSECURE                            0x0B  // ���ܵ�Flash
# define FLASH_VERITF_BACKDOOR_ACCESS_KEY          0x0C  // ������ŷ�����Կ
# define FLASH_SET_USER_MARGIN_LEVEL               0x0D  // �����û�ԣ��ˮƽ
# define FLASH_SET_FACTORY_MARGIN_LEVEL            0x0E  // ���ó���ԣ��ˮƽ
# define FLASH_CONFIGURE_NVM                       0x0F  // ����NVM


# define FLASH_SECTOR_SIZE					(512)
# define FLASH_SECTOR_NUM						(256)
# define FLASH_ALGIN_ADDR						(4)


# define drv_flash_ReadSector(SectorNum, Offset, DataType)	(*(DataType *)((uint32_t)(((SectorNum)*FLASH_SECTOR_SIZE) + (Offset))))	

typedef uint32_t FLASH_WRITE_TYPE;


void drv_flash_Init(void);
uint32_t drv_flash_GetSectorSize(void);
uint8_t drv_flash_EraseSector(uint32_t SectorNum);
uint8_t drv_flash_WriteSector(uint32_t SectorNum, const uint8_t *Data, uint32_t DataLength, uint32_t Offset);


/********************************************  END OF FILE  *******************************************/

