/**
  *******************************************************************************************************
  * File Name: drv_flash.c
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
# include "drv_flash.h"

/*  �˴��ο���ɿƼ�KEA128��  */
volatile uint8_t s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8_t *reg);
flash_run_entry_t s_flash_run_entry;



/*
*********************************************************************************************************
*                       drv_flash_StartCmd                   
*
* Description: ���ļ�˽�к���,��ʼFLASH����
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
    
    if(FTMRE->FSTAT & (FTMRE_FSTAT_ACCERR_MASK | FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_MGSTAT_MASK)) return 1;	//���ִ���
    return 0;
}


/*
*********************************************************************************************************
*                         drv_flash_Init                 
*
* Description: ��ʼ��KEA128�ڲ�FLASH,ʹ֮���Բ���
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
	FTMRE->FCLKDIV = FTMRE_FCLKDIV_FDIV(clk) | FTMRE_FCLKDIV_FDIVLCK_MASK;//����flash��Ƶϵ��

	FTMRE->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;     //���״̬��ʶ
}

/*
*********************************************************************************************************
*                         drv_flash_GetSectorSize                 
*
* Description: ��ȡFLASH������С
*             
* Arguments  : None.
*
* Reutrn     : 1> ������С
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
* Description: ����FLASH����
*             
* Arguments  : 1> SectorNum: ��Ҫ�������������
*
* Reutrn     : 1> 0: ����ִ�гɹ�
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
* Description: ������д�뵽FLASH����
*             
* Arguments  : 1> SectorNum: ��Ҫд����������
*              2> Data: ��Ҫд��������׵�ַ
*              3> DataLength: ��Ҫд������ݳ���
*              3> Offset: ������������ַ�е�ƫ����
*
* Reutrn     : 1> 0: ����ִ�гɹ�
*
* Note(s)    : 1.����KEA128�ڲ�FLASH������,Ҫ��ÿһ��д�������������Ϊ4��������
*							 2.������ַ��ô����һ��������ʼ
*********************************************************************************************************
*/
uint8_t drv_flash_WriteSector(uint32_t SectorNum, const uint8_t *Data, uint32_t DataLength, uint32_t Offset)
{
	uint32_t addr = 0, i = 0;

	/*  �����������ַ  */
	addr = (uint32_t)SectorNum * FLASH_SECTOR_SIZE + Offset;

	/*  ѭ��д������  */
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




