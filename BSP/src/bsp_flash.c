/**
  ******************************************************************************
  * File Name:
  * Author:   
  * Version:  
  * Date:
  * Brief:
  ******************************************************************************
  * History
  *
  *
  ******************************************************************************
  */


/*
  ******************************************************************************
  *                              INCLUDE FILES
  ******************************************************************************
*/
# include "bsp.h"

uint16_t W25QXX_TYPE = W25Q128;



/*
*********************************************************************************************************
*                                          bsp_flash_Config
*
* Description: W25Q128 FlashоƬ��ʼ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_flash_Config(void)
{
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*  FLASHƬѡ���ų�ʼ��  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_A0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  SPI��ʼ��  */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_BaudRateDivider = SPI_BaudRateDivider_5;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	drv_spi_Init(SPI0, &SPI_InitStruct);
	
	FLASH_UNSELECT();		/*  Ƭѡȡ��  */
}


/*
*********************************************************************************************************
*                                          bsp_flash_ReadID
*
* Description: ��ȡFlashоƬID����ͨ����ȡIDֵ�ж��Ƿ��ʼ���ɹ�
*             
* Arguments  : None.
*
* Reutrn     : оƬIDֵ
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint16_t bsp_flash_ReadID(void)
{
	uint16_t id = 0x00;
	FLASH_SELECT();
	drv_spi_ReadWriteByte(SPI0, 0x90);
	drv_spi_ReadWriteByte(SPI0, 0x00);
	drv_spi_ReadWriteByte(SPI0, 0x00);
	drv_spi_ReadWriteByte(SPI0, 0x00);
	
	id |= drv_spi_ReadWriteByte(SPI0, 0xff)<<8;
	id |= drv_spi_ReadWriteByte(SPI0, 0xff);
	
	FLASH_UNSELECT();
	return id;
}



/*
*********************************************************************************************************
*                                          bsp_flash_ReadSR
*
* Description: ��ȡFlashоƬ�Ĵ���
*             
* Arguments  : None.
*
* Reutrn     : �Ĵ���ֵ
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_flash_ReadSR(void)
{
	uint8_t byte = 0x0;
	
	FLASH_SELECT();
	drv_spi_ReadWriteByte(SPI0, W25X_ReadStatusReg);
	byte = drv_spi_ReadWriteByte(SPI0, 0xff);
	FLASH_UNSELECT();
	
	return byte;
}


/*
*********************************************************************************************************
*                                       bsp_flash_WriteSR   
*
* Description: дFlash�Ĵ���ֵ
*             
* Arguments  : sr:Ҫд���ֵ
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_flash_WriteSR(uint8_t sr)
{
	FLASH_SELECT();
	drv_spi_ReadWriteByte(SPI0, W25X_WriteStatusReg);
	drv_spi_ReadWriteByte(SPI0, sr);
	FLASH_UNSELECT();
}



/*
*********************************************************************************************************
*                                        bsp_flash_WriteCmd  
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/

void bsp_flash_WriteCmd(FunctionalState state)
{
	FLASH_SELECT();
	if(state == DISABLE)
		drv_spi_ReadWriteByte(SPI0, W25X_WriteDisable);
	else if(state == ENABLE)
		drv_spi_ReadWriteByte(SPI0, W25X_WriteEnable);
	FLASH_UNSELECT();
}


/*
*********************************************************************************************************
*                                    bsp_flash_WritePage      
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_WritePage(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint16_t i;  
	bsp_flash_WriteCmd(ENABLE);                  //SET WEL 
	FLASH_SELECT();                            //ʹ������   
	drv_spi_ReadWriteByte(SPI0, W25X_PageProgram);      //����дҳ����   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>16)); //����24bit��ַ    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)addr);   
	for(i=0;i<len;i++)drv_spi_ReadWriteByte(SPI0,buff[i]);//ѭ��д��  
	FLASH_UNSELECT();                            //ȡ��Ƭѡ 
	bsp_flash_WaitBusy();					   //�ȴ�д�����
}


/*
*********************************************************************************************************
*                               bsp_flash_WriteNoCheck           
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_WriteNoCheck(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint16_t pageremain;	   
	pageremain=256-addr%256; //��ҳʣ����ֽ���		 	    
	if(len<=pageremain)pageremain=len;//������256���ֽ�
	while(1)
	{	   
		bsp_flash_WritePage(buff,addr,pageremain);
		if(len==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			buff+=pageremain;
			addr+=pageremain;	

			len-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(len>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=len; 	  //����256���ֽ���
		}
	}
}

/*
*********************************************************************************************************
*                              bsp_flash_Read            
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_Read(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint16_t i;   										    
	FLASH_SELECT();                            //ʹ������   
	drv_spi_ReadWriteByte(SPI0, W25X_ReadData);         //���Ͷ�ȡ����   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>16));  //����24bit��ַ    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)addr);   
	for(i=0;i<len;i++)
	{ 
			buff[i]=drv_spi_ReadWriteByte(SPI0, 0XFF);   //ѭ������  
	}
	FLASH_UNSELECT();  
}


/*
*********************************************************************************************************
*                                          
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
uint8_t W25QXX_BUF[4096];
void bsp_flash_Write(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;      
	
 	secpos = addr/4096;//������ַ  
	secoff = addr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
	
 	if(len <= secremain) secremain = len;//������4096���ֽ�
	while(1) 
	{	
		bsp_flash_Read(W25QXX_BUF, secpos*4096, 4096);//������������������
		for(i = 0; i < secremain; i++)//У������
		{
			if(W25QXX_BUF[secoff+i] != 0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			bsp_flash_EraseSector(secpos);//�����������
			for(i = 0; i < secremain; i++)	   //����
			{
				W25QXX_BUF[i+secoff] = buff[i];	  
			}
			bsp_flash_WriteNoCheck(W25QXX_BUF, secpos*4096,4096);//д����������  

		}else bsp_flash_WriteNoCheck(buff, addr, secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(len == secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

			buff += secremain;  //ָ��ƫ��
			addr += secremain;//д��ַƫ��	   
			len -= secremain;				//�ֽ����ݼ�
			if(len > 4096)secremain = 4096;	//��һ����������д����
			else secremain = len;			//��һ����������д����
		}	 
	}
}


/*
*********************************************************************************************************
*                          bsp_flash_EraseChip                
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_EraseChip(void)
{
	bsp_flash_WriteCmd(ENABLE);                  //SET WEL 
	bsp_flash_WaitBusy();   
	FLASH_SELECT();                            //ʹ������   
	drv_spi_ReadWriteByte(SPI0, W25X_ChipErase);        //����Ƭ��������  
	FLASH_UNSELECT();                            //ȡ��Ƭѡ     	      
	bsp_flash_WaitBusy();   				   //�ȴ�оƬ��������
}


/*
*********************************************************************************************************
*                                  bsp_flash_EraseSector        
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_EraseSector(uint32_t dstAddr)
{
	dstAddr*=4096;
	bsp_flash_WriteCmd(ENABLE);                  //SET WEL 	 
	bsp_flash_WaitBusy();   
	FLASH_SELECT();                            //ʹ������   
	drv_spi_ReadWriteByte(SPI0, W25X_SectorErase);      //������������ָ�� 
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((dstAddr)>>16));  //����24bit��ַ    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((dstAddr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)dstAddr);  
	FLASH_UNSELECT();                            //ȡ��Ƭѡ     	      
	bsp_flash_WaitBusy();   				   //�ȴ��������
}


/*
*********************************************************************************************************
*                                 bsp_flash_WaitBusy         
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_WaitBusy(void)
{
	while((bsp_flash_ReadSR() & 0x01)==0x01);   // �ȴ�BUSYλ���
}



/*
*********************************************************************************************************
*                                bsp_flash_PowerCmd          
*
* Description:
*             
* Arguments  :
*
* Reutrn     :
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_flash_PowerCmd(FunctionalState state)
{
	FLASH_SELECT();
	if(state == DISABLE)
		drv_spi_ReadWriteByte(SPI0, W25X_PowerDown);
	else if(state == ENABLE)
		drv_spi_ReadWriteByte(SPI0, W25X_ReleasePowerDown);
	FLASH_UNSELECT();
	bsp_tim_DelayUs(3);
}
