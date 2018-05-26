/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_FLASH_H
# define __BSP_FLASH_H


# define W25Q80 	0XEF13 	
# define W25Q16 	0XEF14
# define W25Q32 	0XEF15
# define W25Q64 	0XEF16
# define W25Q128	0XEF17

extern uint16_t W25QXX_TYPE;					//∂®“ÂW25QXX–æ∆¨–Õ∫≈

# define W25X_WriteEnable		0x06 
# define W25X_WriteDisable		0x04 
# define W25X_ReadStatusReg		0x05 
# define W25X_WriteStatusReg		0x01 
# define W25X_ReadData			0x03 
# define W25X_FastReadData		0x0B 
# define W25X_FastReadDual		0x3B 
# define W25X_PageProgram		0x02 
# define W25X_BlockErase			0xD8 
# define W25X_SectorErase		0x20 
# define W25X_ChipErase			0xC7 
# define W25X_PowerDown			0xB9 
# define W25X_ReleasePowerDown	0xAB 
# define W25X_DeviceID			0xAB 
# define W25X_ManufactDeviceID	0x90 
# define W25X_JedecDeviceID		0x9F 

# define FLASH_CS_PORT		PORTA
# define FLASH_CS_PIN			GPIO_Pin_A0
# define FLASH_SELECT()		drv_gpio_WritePin(FLASH_CS_PIN, GPIO_PIN_RESET)
# define FLASH_UNSELECT()	drv_gpio_WritePin(FLASH_CS_PIN, GPIO_PIN_SET)

void 			bsp_flash_Config(void);
uint16_t 	bsp_flash_ReadID(void);
uint8_t 	bsp_flash_ReadSR(void);
void 			bsp_flash_WriteSR(uint8_t sr);
void 			bsp_flash_WriteCmd(FunctionalState state);
void 			bsp_flash_WritePage(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_WriteNoCheck(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_Read(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_Write(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_EraseChip(void);
void 			bsp_flash_EraseSector(uint32_t dstAddr);
void 			bsp_flash_WaitBusy(void);
void 			bsp_flash_PowerCmd(FunctionalState state);

# endif

/********************************************  END OF FILE  *******************************************/
