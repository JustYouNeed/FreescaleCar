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
* Description: W25Q128 Flash芯片初始化
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
	
	/*  FLASH片选引脚初始化  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_A0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  SPI初始化  */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_BaudRateDivider = SPI_BaudRateDivider_5;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	drv_spi_Init(SPI0, &SPI_InitStruct);
	
	FLASH_UNSELECT();		/*  片选取消  */
}


/*
*********************************************************************************************************
*                                          bsp_flash_ReadID
*
* Description: 读取Flash芯片ID，可通过读取ID值判断是否初始化成功
*             
* Arguments  : None.
*
* Reutrn     : 芯片ID值
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
* Description: 读取Flash芯片寄存器
*             
* Arguments  : None.
*
* Reutrn     : 寄存器值
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
* Description: 写Flash寄存器值
*             
* Arguments  : sr:要写入的值
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
	FLASH_SELECT();                            //使能器件   
	drv_spi_ReadWriteByte(SPI0, W25X_PageProgram);      //发送写页命令   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>16)); //发送24bit地址    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)addr);   
	for(i=0;i<len;i++)drv_spi_ReadWriteByte(SPI0,buff[i]);//循环写数  
	FLASH_UNSELECT();                            //取消片选 
	bsp_flash_WaitBusy();					   //等待写入结束
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
	pageremain=256-addr%256; //单页剩余的字节数		 	    
	if(len<=pageremain)pageremain=len;//不大于256个字节
	while(1)
	{	   
		bsp_flash_WritePage(buff,addr,pageremain);
		if(len==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			buff+=pageremain;
			addr+=pageremain;	

			len-=pageremain;			  //减去已经写入了的字节数
			if(len>256)pageremain=256; //一次可以写入256个字节
			else pageremain=len; 	  //不够256个字节了
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
	FLASH_SELECT();                            //使能器件   
	drv_spi_ReadWriteByte(SPI0, W25X_ReadData);         //发送读取命令   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>16));  //发送24bit地址    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((addr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)addr);   
	for(i=0;i<len;i++)
	{ 
			buff[i]=drv_spi_ReadWriteByte(SPI0, 0XFF);   //循环读数  
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
	
 	secpos = addr/4096;//扇区地址  
	secoff = addr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
	
 	if(len <= secremain) secremain = len;//不大于4096个字节
	while(1) 
	{	
		bsp_flash_Read(W25QXX_BUF, secpos*4096, 4096);//读出整个扇区的内容
		for(i = 0; i < secremain; i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i] != 0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			bsp_flash_EraseSector(secpos);//擦除这个扇区
			for(i = 0; i < secremain; i++)	   //复制
			{
				W25QXX_BUF[i+secoff] = buff[i];	  
			}
			bsp_flash_WriteNoCheck(W25QXX_BUF, secpos*4096,4096);//写入整个扇区  

		}else bsp_flash_WriteNoCheck(buff, addr, secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(len == secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

			buff += secremain;  //指针偏移
			addr += secremain;//写地址偏移	   
			len -= secremain;				//字节数递减
			if(len > 4096)secremain = 4096;	//下一个扇区还是写不完
			else secremain = len;			//下一个扇区可以写完了
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
	FLASH_SELECT();                            //使能器件   
	drv_spi_ReadWriteByte(SPI0, W25X_ChipErase);        //发送片擦除命令  
	FLASH_UNSELECT();                            //取消片选     	      
	bsp_flash_WaitBusy();   				   //等待芯片擦除结束
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
	FLASH_SELECT();                            //使能器件   
	drv_spi_ReadWriteByte(SPI0, W25X_SectorErase);      //发送扇区擦除指令 
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((dstAddr)>>16));  //发送24bit地址    
	drv_spi_ReadWriteByte(SPI0, (uint8_t)((dstAddr)>>8));   
	drv_spi_ReadWriteByte(SPI0, (uint8_t)dstAddr);  
	FLASH_UNSELECT();                            //取消片选     	      
	bsp_flash_WaitBusy();   				   //等待擦除完成
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
	while((bsp_flash_ReadSR() & 0x01)==0x01);   // 等待BUSY位清空
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
