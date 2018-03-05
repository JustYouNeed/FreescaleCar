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

# ifndef __BSP_OLED_H
# define __BSP_OLED_H

# include "derivative.h"


//OLED模式设置
//0:4线串行模式
//1:并行8080模式
# define OLED_MODE 0

# define XLevelL		0x00
# define XLevelH		0x10
# define Max_Column	128
# define Max_Row		64
# define Brightness	0xFF 
# define X_WIDTH 	128
# define Y_WIDTH 	64	    			

/*-----------------OLED端口定义----------------*/

# define OLED_CS_PIN		GPIO_Pin_C7
# define OLED_CS_Clr()  drv_gpio_WritePin(OLED_CS_PIN, GPIO_PIN_RESET)
# define OLED_CS_Set()  drv_gpio_WritePin(OLED_CS_PIN, GPIO_PIN_SET)

# define OLED_RST_PIN		GPIO_Pin_I3
# define OLED_RST_Clr() drv_gpio_WritePin(OLED_RST_PIN, GPIO_PIN_RESET)
# define OLED_RST_Set() drv_gpio_WritePin(OLED_RST_PIN, GPIO_PIN_SET)


# define OLED_DC_PIN		GPIO_Pin_C6
# define OLED_DC_Clr()  drv_gpio_WritePin(OLED_DC_PIN, GPIO_PIN_RESET)
# define OLED_DC_Set()  drv_gpio_WritePin(OLED_DC_PIN, GPIO_PIN_SET)


# define OLED_SCK_PIN		GPIO_Pin_E3
# define OLED_SCK_Clr()  drv_gpio_WritePin(OLED_SCK_PIN, GPIO_PIN_RESET)
# define OLED_SCK_Set()  drv_gpio_WritePin(OLED_SCK_PIN, GPIO_PIN_SET)


# define OLED_DIN_PIN		GPIO_Pin_I2
# define OLED_DIN_Clr()  drv_gpio_WritePin(OLED_DIN_PIN, GPIO_PIN_RESET)
# define OLED_DIN_Set()  drv_gpio_WritePin(OLED_DIN_PIN, GPIO_PIN_SET)

# define OLED_CMD  0	//写命令
# define OLED_DATA 1	//写数据

static uint8_t FRONT_SIZE = 16;

uint32_t oled_pow(uint8_t m,uint8_t n);
void bsp_oled_Config(void);
void bsp_oled_WRByte(uint8_t dat,uint8_t cmd);	    
void bsp_oled_DisplayON(void);
void bsp_oled_DisplayOFF(void);
void bsp_oled_Clear(void);
void bsp_oled_SetFontSize(uint8_t size);
void bsp_oled_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void bsp_oled_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void bsp_oled_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void bsp_oled_ShowInteger(uint8_t x,uint8_t y, int num, uint8_t size);
void bsp_oled_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t size);
void bsp_oled_ShowString(uint8_t x,uint8_t y, uint8_t *p);	 
void bsp_oled_SetPos(unsigned char x, unsigned char y);
void bsp_oled_ShowChinese(uint8_t x,uint8_t y,uint8_t no);
void bsp_oled_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);



# endif


/********************************************  END OF FILE  *******************************************/

