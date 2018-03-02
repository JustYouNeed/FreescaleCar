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
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/

# include "bsp_oled.h"
# include "oledfont.h"
# include "bsp_timer.h"
# include "stdio.h"

/*
*********************************************************************************************************
*                                     bsp_oled_GPIOConfig     
*
* Description: 初始化OLED屏幕引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 本文件私有函数,外部禁止调用
*********************************************************************************************************
*/
void bsp_oled_GPIOConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_HDrv = DISABLE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = OLED_SCK_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	drv_gpio_Init(&GPIO_InitStructure);
 	
	GPIO_InitStructure.GPIO_Pin = OLED_DIN_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = OLED_CS_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = OLED_RST_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = OLED_DC_PIN;
	drv_gpio_Init(&GPIO_InitStructure);
}


/*
*********************************************************************************************************
*                                     bsp_oled_Config     
*
* Description: 初始化OLED屏幕
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_Config(void)
{
	bsp_oled_GPIOConfig();
	
	OLED_RST_Set();
	bsp_tim_DelayMs(100);
	OLED_RST_Clr();
	bsp_tim_DelayMs(100);
	OLED_RST_Set(); 
					  
	bsp_oled_WRByte(0xAE,OLED_CMD);//--turn off oled panel
	bsp_oled_WRByte(0x00,OLED_CMD);//---set low column address
	bsp_oled_WRByte(0x10,OLED_CMD);//---set high column address
	bsp_oled_WRByte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	bsp_oled_WRByte(0x81,OLED_CMD);//--set contrast control register
	bsp_oled_WRByte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	bsp_oled_WRByte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	bsp_oled_WRByte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	bsp_oled_WRByte(0xA6,OLED_CMD);//--set normal display
	bsp_oled_WRByte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	bsp_oled_WRByte(0x3f,OLED_CMD);//--1/64 duty
	bsp_oled_WRByte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	bsp_oled_WRByte(0x00,OLED_CMD);//-not offset
	bsp_oled_WRByte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	bsp_oled_WRByte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	bsp_oled_WRByte(0xD9,OLED_CMD);//--set pre-charge period
	bsp_oled_WRByte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	bsp_oled_WRByte(0xDA,OLED_CMD);//--set com pins hardware configuration
	bsp_oled_WRByte(0x12,OLED_CMD);
	bsp_oled_WRByte(0xDB,OLED_CMD);//--set vcomh
	bsp_oled_WRByte(0x40,OLED_CMD);//Set VCOM Deselect Level
	bsp_oled_WRByte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	bsp_oled_WRByte(0x02,OLED_CMD);//
	bsp_oled_WRByte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	bsp_oled_WRByte(0x14,OLED_CMD);//--set(0x10) disable
	bsp_oled_WRByte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	bsp_oled_WRByte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	bsp_oled_WRByte(0xAF,OLED_CMD);//--turn on oled panel
	
	bsp_oled_WRByte(0xAF,OLED_CMD); /*display ON*/ 
	bsp_oled_Clear();
	bsp_oled_SetPos(0,0); 	
}


/*
*********************************************************************************************************
*                                bsp_oled_WRByte          
*
* Description: 向SSD1106写入一个字节。
*             
* Arguments  : 1> dat:要写入的数据/命令
*              2> cmd:数据/命令标志 0,表示命令;1,表示数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_WRByte(uint8_t dat,uint8_t cmd)
{	
	uint8_t i;
	
	if(cmd)					/*  命令  */
	  OLED_DC_Set();
	else 						/*  数据  */
	  OLED_DC_Clr();		  
	
	OLED_CS_Clr();	/*  选中  */
	
	for(i=0;i<8;i++)	/*  循环发送  */
	{			  
		OLED_SCK_Clr();
		if(dat&0x80)
		   OLED_DIN_Set();
		else 
		   OLED_DIN_Clr();
		OLED_SCK_Set();
		dat<<=1;   
	}				 		  
	OLED_CS_Set();
	OLED_DC_Set();   	  
} 

/*
*********************************************************************************************************
*                                bsp_oled_SetPos          
*
* Description: 设置显示指针位置
*             
* Arguments  : 1> x: X坐标
*              2> y: Y坐标
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_SetPos(uint8_t x, uint8_t y) 
{ 
	bsp_oled_WRByte(0xb0+y,OLED_CMD);
	bsp_oled_WRByte(((x&0xf0)>>4)|0x10,OLED_CMD);
	bsp_oled_WRByte((x&0x0f)|0x01,OLED_CMD); 
}   	  

/*
*********************************************************************************************************
*                                     bsp_oled_DisplayON     
*
* Description: 开启OLED显示
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_DisplayON(void)
{
	bsp_oled_WRByte(0X8D,OLED_CMD);  //SET DCDC命令
	bsp_oled_WRByte(0X14,OLED_CMD);  //DCDC ON
	bsp_oled_WRByte(0XAF,OLED_CMD);  //DISPLAY ON
}


/*
*********************************************************************************************************
*                                   bsp_oled_DisplayOFF
*
* Description: 关闭OLED显示
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_DisplayOFF(void)
{
	bsp_oled_WRByte(0X8D,OLED_CMD);  //SET DCDC命令
	bsp_oled_WRByte(0X10,OLED_CMD);  //DCDC OFF
	bsp_oled_WRByte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 

/*
*********************************************************************************************************
*                               bsp_oled_Clear           
*
* Description: 清屏函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_Clear(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		bsp_oled_WRByte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		bsp_oled_WRByte (0x00,OLED_CMD);      //设置显示位置—列低地址
		bsp_oled_WRByte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)bsp_oled_WRByte(0,OLED_DATA); 
	} //更新显示
}


/*
*********************************************************************************************************
*                                 bsp_oled_ShowChar         
*
* Description: 在指定位置显示一个字符
*             
* Arguments  : 1> x: X坐标
*							 2> y: Y坐标
*              3> chr: 要显示的字符
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>Max_Column-1){x=0;y=y+2;}
	if(FRONT_SIZE ==16)
	{
		bsp_oled_SetPos(x,y);	
		for(i=0;i<8;i++)
		bsp_oled_WRByte(F8X16[c*16+i],OLED_DATA);
		bsp_oled_SetPos(x,y+1);
		for(i=0;i<8;i++)
		bsp_oled_WRByte(F8X16[c*16+i+8],OLED_DATA);
	}
	else if(FRONT_SIZE == 12)
	{	
		bsp_oled_SetPos(x,y+1);
		for(i=0;i<6;i++)
		bsp_oled_WRByte(F6x8[c][i],OLED_DATA);
	}
	else if(FRONT_SIZE == 24)
	{
		bsp_oled_SetPos(x,y);
		for(i = 0;i<12;i++)
			bsp_oled_WRByte(OLED_Font12X24[c*36+i],OLED_DATA);
		bsp_oled_SetPos(x,y+1);
		for(i = 0;i<12;i++)
			bsp_oled_WRByte(OLED_Font12X24[i+c*36+12],OLED_DATA);
		bsp_oled_SetPos(x,y+2);
		for(i = 0;i<12;i++)
			bsp_oled_WRByte(OLED_Font12X24[i+c*36+12+12],OLED_DATA);
	}
}

/*
*********************************************************************************************************
*                               oled_pow           
*
* Description: 求m的n次方
*             
* Arguments  : 1> m: 底数
*              2> n: 指数
*
* Reutrn     : 1> m^n的计算结果
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}				  


void bsp_oled_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				bsp_oled_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}else enshow=1; 
		 	 
		}
	 	bsp_oled_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
} 

/*
*********************************************************************************************************
*                                  bsp_oled_ShowInteger        
*
* Description: 在指定位置显示一个整数
*             
* Arguments  : 1> x,y: 显示坐标
*              2> num: 要显示的数字
*              3> size: 字体大小
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_oled_ShowInteger(uint8_t x,uint8_t y, int num,uint8_t size)
{         	
	uint8_t len;
	uint32_t numtemp = 0;
	uint8_t xpos = x;
	
	if(num < 0) 	/*  数字为负数先显示负号  */
	{
		bsp_oled_ShowChar(x, y, '-');
		xpos += size/2;
		num = - num;
	}
	

	numtemp = num;
	while(numtemp)		/*  求数字位宽		*/
	{
		numtemp /= 10;
		len ++;
	}
	
	if(num == 0) len = 1;
	
	bsp_oled_ShowNum(xpos, y, num, len, size);
} 

/*
*********************************************************************************************************
*                                  bsp_oled_ShowFloat        
*
* Description: 在指定位置显示一个浮点数
*             
* Arguments  : 1> x,y: 显示坐标
*              2> num: 要显示的数字
*              3> size: 字体大小
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/	

void bsp_oled_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t size)
{
//	uint8_t numwidth;
	uint8_t xpos = 0;
	float temp;
	int tempnum = 0;
	int integer = 0;
  int f = 0;


	bsp_oled_ShowInteger(x, y, (int)num, size);
	
	if(num < 0)	xpos += size/2;
	
	integer = (int)num;				/*  求取整数部分  */
	tempnum = integer;
	while(tempnum)
	{
		tempnum /= 10;
		xpos += size/2;
	}
	temp = num;	
	xpos += size/2;
	f = (int)(temp * 100)%100;	/*  小数部分  */
	if(f < 0) f = -f;
	
	if(f < 10)
	{
		bsp_oled_SetPos(xpos, y);
		bsp_oled_ShowInteger(xpos, y, 0, size);
		xpos += size/2;
		bsp_oled_SetPos(xpos, y);
		bsp_oled_ShowInteger(xpos, y, (int)f, size);
	}
	else
	{
		bsp_oled_SetPos(xpos, y);
		bsp_oled_ShowInteger(xpos, y, f, size);
	}
	xpos -= size/2;
	bsp_oled_ShowChar(xpos, y, '.');
}
//显示一个字符号串
void bsp_oled_ShowString(uint8_t x,uint8_t y,uint8_t *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		bsp_oled_ShowChar(x,y,chr[j]);
			if(FRONT_SIZE == 24)x+=12;
		else x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
void bsp_oled_ShowChinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	bsp_oled_SetPos(x,y);	
    for(t=0;t<FRONT_SIZE;t++)
		{
				bsp_oled_WRByte(OLED_Font12[2*no][t],OLED_DATA);
				adder+=1;
     }	
		bsp_oled_SetPos(x,y+1);	
    for(t=0;t<FRONT_SIZE;t++)
			{	
				bsp_oled_WRByte(OLED_Font12[2*no+1][t],OLED_DATA);
				adder+=1;
      }					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void bsp_oled_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		bsp_oled_SetPos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	bsp_oled_WRByte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


void bsp_oled_SetFontSize(uint8_t size)
{
	FRONT_SIZE = size;
}

/********************************************  END OF FILE  *******************************************/
