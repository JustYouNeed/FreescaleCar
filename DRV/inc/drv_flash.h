/**
  *******************************************************************************************************
  * File Name: drv_flash.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 本文件提供了对KEA128内部FLASH的操作函数,部分程序参考逐飞科技
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
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
# define FLASH_ERASE_VERITF_ALL_BLOCKS             0x01  // 擦除检验所有区块
# define FLASH_ERASE_VERITF_BLOCKS                 0x02  // 擦除检验数据块
# define FLASH_ERASE_VERITF_SECTION          			 0x03  // 擦除检验Flash 段
# define FLASH_READ_ONCE                           0x04  // 读取一次
# define FLASH_PROGRAM                       			 0x06  // 编程Flash
# define FLASH_PROGRAM_ONCE                        0x07  // 编程一次
# define FLASH_ERASE_ALL_BLOCKS                    0x08  // 擦除所有区块
# define FLASH_ERASE_BLOCKS        			           0x09  // 擦除Flash 区块
# define FLASH_ERASE_SECTOR			                   0x0A  // 擦除Flash 扇区
# define FLASH_UNSECURE                            0x0B  // 解密的Flash
# define FLASH_VERITF_BACKDOOR_ACCESS_KEY          0x0C  // 检验后门访问密钥
# define FLASH_SET_USER_MARGIN_LEVEL               0x0D  // 设置用户裕量水平
# define FLASH_SET_FACTORY_MARGIN_LEVEL            0x0E  // 设置出厂裕量水平
# define FLASH_CONFIGURE_NVM                       0x0F  // 配置NVM


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

