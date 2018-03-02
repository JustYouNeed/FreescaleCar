/**
  *******************************************************************************************************
  * File Name: drv_flash.c
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
# include "drv_flash.h"

/*  此处参考逐飞科技KEA128库  */
volatile uint8_t s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8_t *reg);
flash_run_entry_t s_flash_run_entry;



/*
*********************************************************************************************************
*                       drv_flash_StartCmd                   
*
* Description: 本文件私有函数,开始FLASH命令
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_flash_StartCmd(void)
{
		DISABLE_INT();
    
    FTMRE->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;
    
    s_flash_run_entry = (flash_run_entry_t)((uint32_t)s_flash_command_run + 1);
    s_flash_run_entry(&FTMRE->FSTAT);
    
    ENABLE_INT();
    
    if(FTMRE->FSTAT & (FTMRE_FSTAT_ACCERR_MASK | FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_MGSTAT_MASK)) return 1;	//出现错误
    return 0;
}


/*
*********************************************************************************************************
*                         drv_flash_Init                 
*
* Description: 初始化KEA128内部FLASH,使之可以操作
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void drv_flash_Init(void)
{
	uint16_t clk = 0;
	
	drv_rcc_ClockCmd(RCC_PeriphClock_FLASH, ENABLE);
	while(!(FTMRE->FSTAT & FTMRE_FSTAT_CCIF_MASK));
	
	clk = (SystemBusClock / 1000000) - 1;
	FTMRE->FCLKDIV = FTMRE_FCLKDIV_FDIV(clk) | FTMRE_FCLKDIV_FDIVLCK_MASK;//设置flash分频系数

	FTMRE->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;     //清除状态标识
}

/*
*********************************************************************************************************
*                         drv_flash_GetSectorSize                 
*
* Description: 获取FLASH扇区大小
*             
* Arguments  : None.
*
* Reutrn     : 1> 扇区大小
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t drv_flash_GetSectorSize(void)
{
	return FLASH_SECTOR_SIZE;
}


/*
*********************************************************************************************************
*                        drv_flash_EraseSector                  
*
* Description: 擦除FLASH扇区
*             
* Arguments  : 1> SectorNum: 需要擦除的扇区编号
*
* Reutrn     : 1> 0: 函数执行成功
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t drv_flash_EraseSector(uint32_t SectorNum)
{
	uint32_t addr;

	addr = (uint32_t)SectorNum*FLASH_SECTOR_SIZE;

	FTMRE->FCCOBIX = 0;
	FTMRE->FCCOBHI = FLASH_ERASE_SECTOR;
	FTMRE->FCCOBLO = addr>>16;

	FTMRE->FCCOBIX = 1;
	FTMRE->FCCOBHI = (addr&0xffff)>>8;
	FTMRE->FCCOBLO = addr&0xff;

	drv_flash_StartCmd();

	return 0;
}

/*
*********************************************************************************************************
*                        drv_flash_WriteSector                  
*
* Description: 将数据写入到FLASH扇区
*             
* Arguments  : 1> SectorNum: 需要写入的扇区编号
*              2> Data: 需要写入的数据首地址
*              3> DataLength: 需要写入的数据长度
*              3> Offset: 数据在扇区地址中的偏移量
*
* Reutrn     : 1> 0: 函数执行成功
*
* Note(s)    : 1.由于KEA128内部FLASH的特性,要求每一次写入的数据量必须为4的整数倍
*							 2.扇区地址最好从最后一个扇区开始
*********************************************************************************************************
*/
uint8_t drv_flash_WriteSector(uint32_t SectorNum, const uint8_t *Data, uint32_t DataLength, uint32_t Offset)
{
	uint32_t addr = 0, i = 0;

	/*  计算出扇区地址  */
	addr = (uint32_t)SectorNum * FLASH_SECTOR_SIZE + Offset;

	/*  循环写入数据  */
	for(; i < DataLength; i += FLASH_ALGIN_ADDR)
	{
		FTMRE->FCCOBIX = 0;
		FTMRE->FCCOBHI = FLASH_PROGRAM;
		FTMRE->FCCOBLO = addr>>16;
		
		FTMRE->FCCOBIX = 1;
		FTMRE->FCCOBHI = (addr&0xFFFF)>>8;
		FTMRE->FCCOBLO = addr&0xFF;
		
		FTMRE->FCCOBIX = 2;
		FTMRE->FCCOBLO = Data[0];
		FTMRE->FCCOBHI = Data[1];
		
		FTMRE->FCCOBIX = 3;
		FTMRE->FCCOBLO = Data[2];
		FTMRE->FCCOBHI = Data[3];

		Data += FLASH_ALGIN_ADDR;
		addr += FLASH_ALGIN_ADDR;

		drv_flash_StartCmd();
	}
	return 0;
}


/********************************************  END OF FILE  *******************************************/




